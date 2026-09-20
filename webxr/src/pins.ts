import {
  CylinderGeometry,
  Group,
  MathUtils,
  Mesh,
  MeshBasicMaterial,
  Object3D,
  SphereGeometry,
  UIKitMLAsset,
  Vector3,
} from '@iwsdk/core';
import type { AssetKind, FeedAsset } from './feed';
import type { TerrainMap } from './terrain';

export interface PinDefinition {
  /** Matches the asset name the bridge sends. */
  name: string;
  kind: AssetKind;
  color: number;
}

const PIN_RADIUS = 0.011;
const STEM_RADIUS = 0.0009;
const BEAM_HEIGHT = 0.5;
const BEAM_RADIUS = 0.002;
const BEAM_OPACITY = 0.9;
const LABEL_SCALE = 0.13;
const SHOW_DISTANCE = 0.7; // headset closer than this reveals the label...
const HIDE_DISTANCE = 0.9; // ...and it stays until the headset is farther than this
const POINT_RADIUS = 0.08; // how close a controller ray must pass to count as pointing
const SMOOTHING = 8; // 1/s. Higher tracks the feed more tightly, lower hides its 4 Hz steps
const STALE_S = 5; // an asset silent this long is hidden rather than drawn where it last was

const tmpHead = new Vector3();
const tmpDirection = new Vector3();
const tmpPoint = new Vector3();
const tmpToPoint = new Vector3();
const tmpLabel = new Vector3();

const KIND_LABEL: Record<AssetKind, string> = {
  copter: 'Quadcopter',
  plane: 'Fixed-wing',
  tower: 'Tower',
  boat: 'Target vessel',
};

class Pin {
  readonly group = new Group();
  private readonly stem: Mesh;
  private readonly beam: Mesh;
  private readonly label: UIKitMLAsset;
  private readonly target = new Vector3();
  private latest: FeedAsset | null = null;
  private placed = false;
  private inBounds = false;
  private descriptionDirty = false;
  active = false;

  constructor(
    private readonly definition: PinDefinition,
    label: UIKitMLAsset,
    private readonly terrain: TerrainMap,
  ) {
    this.group.add(new Mesh(new SphereGeometry(PIN_RADIUS, 24, 16), new MeshBasicMaterial({ color: definition.color })));

    // A thin post down to the ground shows where an aircraft is over the terrain. Drawn in map
    // space, so it is parented to the field root and not to the pin (which moves in 3D).
    this.stem = new Mesh(
      new CylinderGeometry(STEM_RADIUS, STEM_RADIUS, 1, 6, 1, true),
      new MeshBasicMaterial({ color: definition.color, transparent: true, opacity: 0.6 }),
    );

    // Thin white beam, only shown while the pin is selected.
    this.beam = new Mesh(
      new CylinderGeometry(BEAM_RADIUS, BEAM_RADIUS, BEAM_HEIGHT, 8, 1, true),
      new MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: BEAM_OPACITY, depthWrite: false }),
    );
    this.beam.position.y = PIN_RADIUS + BEAM_HEIGHT / 2;
    this.beam.visible = false;
    this.group.add(this.beam);

    this.label = label;
    this.label.scale.setScalar(LABEL_SCALE);
    // The label is anchored at its bottom edge, so it sits right on the beam tip.
    this.label.position.y = PIN_RADIUS + BEAM_HEIGHT;
    this.label.visible = false;
    label.requireElementById('orb-name').setProperties({ text: definition.name.toUpperCase() });
    label.requireElementById('orb-description').setProperties({ text: 'No position yet' });
    this.group.add(this.label);

    this.group.visible = false;
    this.stem.visible = false;
  }

  get stemMesh(): Mesh {
    return this.stem;
  }

  setAsset(asset: FeedAsset): void {
    this.latest = asset;
    this.terrain.localPosition(asset.lat, asset.lon, asset.alt, this.target);
    const world = this.terrain.georef.toWorld(asset.lat, asset.lon);
    this.inBounds = this.terrain.georef.contains(world.x, world.y);
    this.descriptionDirty = true;
  }

  /** Move toward the latest fix, and hide the pin if the asset has gone quiet. */
  track(deltaSeconds: number, sinceUpdateS: number): void {
    const asset = this.latest;
    const fresh = asset !== null && this.inBounds && asset.age + sinceUpdateS < STALE_S;
    this.group.visible = fresh;
    this.stem.visible = fresh;
    if (!fresh) return;

    if (!this.placed) {
      this.group.position.copy(this.target);
      this.placed = true;
    } else {
      this.group.position.lerp(this.target, 1 - Math.exp(-SMOOTHING * deltaSeconds));
    }

    // Stem from the terrain surface up to the pin.
    const p = this.group.position;
    const ground = this.terrain.groundHeight(p.x, p.z);
    const height = Math.max(p.y - ground, 0.0001);
    this.stem.scale.set(1, height, 1);
    this.stem.position.set(p.x, ground + height / 2, p.z);
  }

  /** Refresh the label text. Only worth doing while the label is showing. */
  refreshDescription(): void {
    if (!this.active || !this.descriptionDirty || !this.latest) return;
    this.descriptionDirty = false;
    const a = this.latest;
    this.label.requireElementById('orb-description').setProperties({
      text: `${KIND_LABEL[this.definition.kind]}\n${a.lat.toFixed(5)}, ${a.lon.toFixed(5)}\nAlt ${a.alt.toFixed(0)} m`,
    });
  }

  /** True if any ray passes close to the pin. Once selected, the beam counts too. */
  isPointedAt(origins: Vector3[], directions: Vector3[]): boolean {
    const heights = this.active ? [0, PIN_RADIUS + BEAM_HEIGHT / 2, PIN_RADIUS + BEAM_HEIGHT] : [0];
    for (const height of heights) {
      this.group.localToWorld(tmpPoint.set(0, height, 0));
      for (let i = 0; i < origins.length; i++) {
        tmpToPoint.subVectors(tmpPoint, origins[i]);
        const along = tmpToPoint.dot(directions[i]);
        if (along <= 0) continue;
        if (tmpToPoint.lengthSq() - along * along <= POINT_RADIUS * POINT_RADIUS) return true;
      }
    }
    return false;
  }

  setActive(active: boolean): void {
    this.active = active;
    this.label.visible = active;
    this.beam.visible = active;
    if (active) this.descriptionDirty = true;
  }

  /** Turn the label to face the headset, staying upright. */
  faceHead(head: Vector3): void {
    this.label.getWorldPosition(tmpLabel);
    this.label.lookAt(head.x, tmpLabel.y, head.z);
  }
}

/** Live asset pins on the terrain. Getting close or pointing at one shows a beam and a label with its GPS fix. */
export class PinField {
  /** Add this to the map marker so the pins move in map space. */
  readonly root = new Group();
  private readonly pins = new Map<string, Pin>();
  private sinceUpdateS = 0;

  constructor(definitions: readonly PinDefinition[], labels: readonly UIKitMLAsset[], terrain: TerrainMap) {
    definitions.forEach((definition, i) => {
      const pin = new Pin(definition, labels[i], terrain);
      this.pins.set(definition.name, pin);
      this.root.add(pin.group, pin.stemMesh);
    });
    this.root.visible = false;
  }

  /** Feed a new batch of fixes. Unknown asset names are ignored. */
  setAssets(assets: readonly FeedAsset[]): void {
    this.sinceUpdateS = 0;
    for (const asset of assets) this.pins.get(asset.name)?.setAsset(asset);
  }

  /**
   * @param enabled Pins only show and move while true (i.e. once the app is ready).
   * @param showLabels Labels can be suppressed, e.g. while the settings panel is open.
   * @param rays Controller ray spaces currently tracked; their -Z axis is the pointing direction.
   */
  update(deltaSeconds: number, enabled: boolean, showLabels: boolean, head: Object3D, rays: Object3D[]): void {
    this.root.visible = enabled;
    if (!enabled) return;

    head.getWorldPosition(tmpHead);
    const origins = rays.map((ray) => ray.getWorldPosition(new Vector3()));
    const directions = rays.map((ray) => tmpDirection.set(0, 0, -1).transformDirection(ray.matrixWorld).clone());

    const dt = MathUtils.clamp(deltaSeconds, 0, 0.1);
    this.sinceUpdateS += deltaSeconds;
    for (const pin of this.pins.values()) {
      pin.track(dt, this.sinceUpdateS);
      if (!pin.group.visible) {
        pin.setActive(false);
        continue;
      }
      pin.group.getWorldPosition(tmpPoint);
      const near = tmpPoint.distanceTo(tmpHead) < (pin.active ? HIDE_DISTANCE : SHOW_DISTANCE);
      pin.setActive(showLabels && (near || pin.isPointedAt(origins, directions)));
      if (pin.active) {
        pin.faceHead(tmpHead);
        pin.refreshDescription();
      }
    }
  }
}
