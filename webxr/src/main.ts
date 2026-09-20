import {
  DoubleSide,
  EnvironmentRaycastTarget,
  FollowBehavior,
  Follower,
  Group,
  Mesh,
  MeshBasicMaterial,
  Object3D,
  PokeInteractable,
  RaycastSpace,
  RayDisplayMode,
  RayInteractable,
  RayPointer,
  RingGeometry,
  UIKitMLAsset,
  World,
  createSystem,
} from '@iwsdk/core';
import projectOptions from 'virtual:iwsdk-project';
import { TelemetryFeed } from './feed';
import { PinField, type PinDefinition } from './pins';
import { loadTerrain, type TerrainMap } from './terrain';
import { SpeechTranscriber, type TranscriberStatus } from './transcriber';
import './style.css';

const container = document.querySelector<HTMLDivElement>('#scene-container');
if (!container) throw new Error('Missing #scene-container.');

/** Site exported by tools/prepare_terrain.py, served from public/terrain/<site>. */
const SITE = new URLSearchParams(location.search).get('site') ?? 'fort_ross';
/** Edge of the square terrain map on the table, in meters. */
const MAP_WIDTH_M = 0.9;

/** Names match the assets the bridge reports (arctic-sim roles, plus the tracked vessel). */
const PIN_DEFINITIONS: ReadonlyArray<PinDefinition> = [
  { name: 'quadcopter', kind: 'copter', color: 0x3498db },
  { name: 'fixed-wing', kind: 'plane', color: 0xf39c12 },
  { name: 'tower-1', kind: 'tower', color: 0x2ecc71 },
  { name: 'tower-2', kind: 'tower', color: 0x2ecc71 },
  { name: 'boat', kind: 'boat', color: 0xe74c3c },
];

let settingsRoot: Object3D | null = null;
let settingsVisible = true;
let placementSystem: MapPlacementSystem | null = null;
let terrainMap: TerrainMap | null = null;

/** Whether the main loop may run (still to be defined). Turns on when the user confirms the map. */
let ready = false;
let mapPlaced = false;

let placementHud: UIKitMLAsset | null = null;
let placementMessage: ReturnType<UIKitMLAsset['requireElementById']> | null = null;
let placementActions: ReturnType<UIKitMLAsset['requireElementById']> | null = null;

/** Keeps the map-placement HUD in sync with `ready`, `mapPlaced` and the settings panel. */
function refreshPlacementHud(): void {
  if (!placementHud) return;
  placementHud.visible = !ready && !settingsVisible;
  placementMessage?.setProperties({
    text: mapPlaced ? 'Map placed. Confirm or undo.' : 'Place the map using the index trigger',
  });
  placementActions?.setProperties({ display: mapPlaced ? 'flex' : 'none' });
}

function setSettingsVisible(visible: boolean): void {
  settingsVisible = visible;
  if (settingsRoot) settingsRoot.visible = visible;
  refreshPlacementHud();
}

function setReady(value: boolean): void {
  ready = value;
  refreshPlacementHud();
}

class MapPlacementSystem extends createSystem({
  targets: { required: [EnvironmentRaycastTarget] },
}) {
  private mapMarker: Object3D | null = null;
  private pins: PinField | null = null;
  private targets: Array<{ entity: any; hand: 'left' | 'right' }> = [];

  init(): void {
    placementSystem = this;

    // The terrain lies on the surface (local Y is the surface normal). Pins are added to it later.
    const marker = new Group();
    if (terrainMap) marker.add(terrainMap.group);
    marker.visible = false;
    this.world.createTransformEntity(marker);
    this.mapMarker = marker;

    const reticleMaterial = new MeshBasicMaterial({
      color: 0x5b5bd6,
      transparent: true,
      opacity: 0.8,
      side: DoubleSide,
    });

    for (const [space, hand] of [[RaycastSpace.Left, 'left'], [RaycastSpace.Right, 'right']] as const) {
      const reticle = new Mesh(new RingGeometry(0.08, 0.1, 32), reticleMaterial);
      reticle.rotation.x = -Math.PI / 2;
      const target = this.world.createTransformEntity(reticle);
      target.addComponent(EnvironmentRaycastTarget, { space, maxDistance: 10 });
      this.targets.push({ entity: target, hand });
    }
  }

  placeMap(reticle: Object3D): void {
    if (!this.mapMarker) return;
    this.mapMarker.position.copy(reticle.position);
    this.mapMarker.quaternion.copy(reticle.quaternion);
    this.mapMarker.visible = true;
    mapPlaced = true;
    refreshPlacementHud();
  }

  /** Parent the pins to the map so they move in map space. */
  attachPins(pins: PinField): void {
    this.pins = pins;
    this.mapMarker?.add(pins.root);
  }

  clearMap(): void {
    if (this.mapMarker) this.mapMarker.visible = false;
    mapPlaced = false;
    refreshPlacementHud();
  }

  update(delta: number): void {
    const leftGrip = this.input.xr.gamepads.left?.getButtonDown('xr-standard-squeeze') === true;
    const rightGrip = this.input.xr.gamepads.right?.getButtonDown('xr-standard-squeeze') === true;
    if (leftGrip || rightGrip) {
      setSettingsVisible(!settingsVisible);
    }

    if (this.pins) {
      const { head, raySpaces } = this.world.player;
      const rays: Object3D[] = [];
      if (this.input.xr.gamepads.left) rays.push(raySpaces.left);
      if (this.input.xr.gamepads.right) rays.push(raySpaces.right);
      this.pins.update(delta, ready, !settingsVisible, head, rays);
    }

    for (const { entity: target, hand } of this.targets) {
      const hit = target.getValue(EnvironmentRaycastTarget, 'xrHitTestResult');
      const reticle = target.object3D;
      // The trigger belongs to the settings panel while it is open. Otherwise it places the map,
      // until one is placed (then Confirm/Undo take over) or we are ready.
      const canPlace = !settingsVisible && !ready && !mapPlaced;
      if (reticle) reticle.visible = Boolean(hit) && canPlace;
      if (!canPlace) continue;

      const selectStart = hand === 'right'
        ? this.input.xr.gamepads.right?.getSelectStart() === true
        : this.input.xr.gamepads.left?.getSelectStart() === true;
      if (!hit || !selectStart || !reticle) continue;
      this.placeMap(reticle);
    }
  }
}

terrainMap = await loadTerrain(`${import.meta.env.BASE_URL}terrain/${SITE}`, { widthM: MAP_WIDTH_M });

const world = await World.create(container, projectOptions);

// By default the SDK only draws a controller ray while it hovers something interactive.
// Show it all the time. MultiPointer keeps its RayPointer private, hence the cast.
for (const hand of ['left', 'right'] as const) {
  const { ray } = world.input.xr.multiPointers[hand] as unknown as { ray: { visual: RayPointer } };
  ray.visual.rayDisplayMode = RayDisplayMode.Visible;
}
world.registerSystem(MapPlacementSystem);

const settings = await world.assets.instantiate<UIKitMLAsset>('settings');
const settingsEntity = world.createTransformEntity(settings);
settingsRoot = settingsEntity.object3D ?? null;
settingsEntity.object3D!.position.set(0, 1.45, -1.5);
settingsEntity.object3D!.scale.setScalar(0.25);
settingsEntity.addComponent(Follower, {
  target: world.player.head,
  offsetPosition: [0, 0, -1.5],
  behavior: FollowBehavior.PivotY,
  speed: 5,
  tolerance: 0.3,
  maxAngle: 30,
});
settingsEntity.addComponent(RayInteractable);
settingsEntity.addComponent(PokeInteractable);

placementHud = await world.assets.instantiate<UIKitMLAsset>('placement');
const placementEntity = world.createTransformEntity(placementHud);
placementEntity.object3D!.scale.setScalar(0.25);
placementEntity.addComponent(Follower, {
  target: world.player.head,
  offsetPosition: [0, 0.35, -1.2],
  behavior: FollowBehavior.FaceTarget,
  speed: 6,
  tolerance: 0.15,
  maxAngle: 20,
});
placementEntity.addComponent(RayInteractable);
placementEntity.addComponent(PokeInteractable);
placementMessage = placementHud.requireElementById('placement-message');
placementActions = placementHud.requireElementById('placement-actions');
placementHud.requireElementById('undo-map').addEventListener('click', () => placementSystem?.clearMap());
placementHud.requireElementById('confirm-map').addEventListener('click', () => setReady(true));
refreshPlacementHud();

const pinLabels = await Promise.all(
  PIN_DEFINITIONS.map(() => world.assets.instantiate<UIKitMLAsset>('orbLabel')),
);
const pins = new PinField(PIN_DEFINITIONS, pinLabels, terrainMap);
const attachPins = (field: PinField): void => placementSystem?.attachPins(field);
attachPins(pins);
new TelemetryFeed(undefined, 250, (assets) => pins.setAssets(assets), (status) => console.info(`[feed] ${status}`)).start();

const enterAr = settings.requireElementById('enter-ar');
enterAr.addEventListener('click', () => world.launchXR());

const rescanRoom = settings.requireElementById('rescan-room');
const regenerateRoom = async (): Promise<void> => {
  const session = world.session as (XRSession & {
    initiateRoomCapture?: () => Promise<void> | void;
  }) | undefined;

  if (!session?.initiateRoomCapture) {
    console.warn('Room capture is only available while the Quest AR session is active.');
    return;
  }

  try {
    await session.initiateRoomCapture();
  } catch (error) {
    console.error('Room capture failed.', error);
  }
};
rescanRoom.addEventListener('click', regenerateRoom);

const closeSettings = settings.requireElementById('close-settings');
closeSettings.addEventListener('click', () => {
  setSettingsVisible(false);
});

const speakButton = settings.requireElementById('speak');
const speechStatus = settings.requireElementById('speech-status');
const transcript = settings.requireElementById('transcript');
const apiKey = import.meta.env.VITE_ELEVENLABS_API_KEY;

const updateTranscription = (status: TranscriberStatus, message?: string): void => {
  switch (status) {
    case 'listening':
      speechStatus.setProperties({ text: 'Listening… press again to stop' });
      break;
    case 'processing':
      speechStatus.setProperties({ text: 'Transcribing…' });
      break;
    case 'done':
      speechStatus.setProperties({ text: 'You said:' });
      transcript.setProperties({ text: `"${message ?? ''}"` });
      break;
    case 'error':
      speechStatus.setProperties({ text: `Error: ${message ?? 'unknown error'}` });
      break;
    default:
      speechStatus.setProperties({ text: 'Press Speak and talk' });
  }
};

if (apiKey) {
  const transcriber = new SpeechTranscriber(apiKey, updateTranscription);
  speakButton.addEventListener('click', () => {
    if (transcriber.active) {
      void transcriber.stopListening();
    } else {
      void transcriber.startListening();
    }
  });
} else {
  speechStatus.setProperties({ text: 'Unavailable: set VITE_ELEVENLABS_API_KEY' });
  speakButton.setProperties({ disabled: true });
}
