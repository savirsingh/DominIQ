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
import { CameraPanel } from './camera-panel';
import type { AssetKind, FeedAsset } from './feed';
import { PulseRings } from './glow';
import type { OpenStream } from './mjpeg';
import type { TerrainMap } from './terrain';

/** A tracked controller: where it points, and whether its index trigger went down this frame. */
export interface PinPointer {
  space: Object3D;
  select: boolean;
}

export interface PinDefinition {
  /** Matches the asset name the bridge sends. */
  name: string;
  kind: AssetKind;
  color: number;
  /** True if the asset carries a camera. Locking such a pin opens its live stream in a panel. */
  camera?: boolean;
  /** Pulsing rings and a halo around the pin, to make it stand out. */
  glow?: boolean;
}

export interface PinFieldOptions {
  /** Where an asset's MJPEG stream lives. Only called for assets whose definition has `camera`. */
  cameraUrl?: (name: string) => string;
  /** Replaces the network stream, for tests. */
  openStream?: OpenStream;
  /** Raise tags that would overlap so they stack instead. On by default. */
  avoidTagOverlap?: boolean;
}

const PIN_RADIUS = 0.011;
const STEM_RADIUS = 0.0009;
const BEAM_HEIGHT = 0.3; // how far above the pin the tag floats (was 0.5)
const BEAM_RADIUS = 0.002;
const BEAM_OPACITY = 0.9;
const BEAM_COLOR = 0xffffff;
const LOCKED_BEAM_COLOR = 0xffd54a; // a locked pin's beam turns gold
const LABEL_SCALE = 0.13;
// Gap between the info label and the camera video above it. 0 = flush.
const PANEL_MARGIN = 0;
// UIKit lays the label out in pixels; this is its meters-per-pixel unless the document says otherwise.
const UIKIT_PIXEL_SIZE = 0.01;
// Tag layout. Two tags whose pins are closer than TAG_WIDTH on the map would overlap, so one is raised
// until it clears the other by TAG_GAP. Adjust these to taste.
const TAG_WIDTH = 0.42; // the label is about 0.36 m wide; the rest is margin
const TAG_GAP = 0.01; // space between one asset's tag and the next one stacked above it
// Label height used only until UIKit has laid the label out; after that the real height is measured.
const LABEL_HEIGHT_FALLBACK = 0.15;
const MAX_LIFT = 1.0; // never raise a tag more than this above its normal height
const LIFT_SMOOTHING = 10; // 1/s. How quickly a tag glides to its new height
const SHOW_DISTANCE = 0.7; // headset closer than this reveals the label...
const HIDE_DISTANCE = 0.9; // ...and it stays until the headset is farther than this
const POINT_RADIUS = 0.08; // how close a controller ray must pass to count as pointing
const HOVER_STICKINESS = 0.7; // the pin already hovered wins unless another is this much closer to the ray
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
  private readonly body: Mesh;
  private readonly glow: PulseRings | null;
  private readonly stem: Mesh;
  private readonly beam: Mesh;
  private readonly beamMaterial: MeshBasicMaterial;
  private readonly label: UIKitMLAsset;
  private readonly target = new Vector3();
  private latest: FeedAsset | null = null;
  private placed = false;
  private inBounds = false;
  private descriptionDirty = false;
  private shown = false;
  private panel: CameraPanel | null = null;
  private lift = 0; // extra height of the tag above its normal one, now
  private targetLift = 0; // ...and where the layout wants it
  private justShown = false;
  /** The one pin the pointer (or, failing that, the headset) is selecting right now. */
  hovered = false;
  /** Toggled by the index trigger. A locked pin keeps its beam and label up. */
  locked = false;

  constructor(
    private readonly definition: PinDefinition,
    label: UIKitMLAsset,
    private readonly terrain: TerrainMap,
    private readonly cameraUrl: string | null = null,
    private readonly openStream?: OpenStream,
  ) {
    this.body = new Mesh(new SphereGeometry(PIN_RADIUS, 24, 16), new MeshBasicMaterial({ color: definition.color }));
    this.group.add(this.body);
    this.glow = definition.glow ? new PulseRings(definition.color) : null;
    if (this.glow) this.group.add(this.glow);

    // A thin post down to the ground shows where an aircraft is over the terrain. Drawn in map
    // space, so it is parented to the field root and not to the pin (which moves in 3D).
    this.stem = new Mesh(
      new CylinderGeometry(STEM_RADIUS, STEM_RADIUS, 1, 6, 1, true),
      new MeshBasicMaterial({ color: definition.color, transparent: true, opacity: 0.6 }),
    );

    // Thin beam, only shown while the pin is hovered or locked. Gold when locked.
    this.beamMaterial = new MeshBasicMaterial({
      color: BEAM_COLOR,
      transparent: true,
      opacity: BEAM_OPACITY,
      depthWrite: false,
    });
    this.beam = new Mesh(new CylinderGeometry(BEAM_RADIUS, BEAM_RADIUS, BEAM_HEIGHT, 8, 1, true), this.beamMaterial);
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
    if (this.definition.kind === 'boat') {
      // Its position is an estimate and can land a little inland. Keep it on the surface, not under it.
      this.target.y = Math.max(this.target.y, this.terrain.groundHeight(this.target.x, this.target.z));
    }
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

    if (this.glow) {
      this.glow.update(deltaSeconds);
      this.body.scale.setScalar(1 + 0.3 * this.glow.pulse); // the pin breathes with the rings
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
    if (!this.shown || !this.descriptionDirty || !this.latest) return;
    this.descriptionDirty = false;
    const a = this.latest;
    const hint = this.locked ? 'LOCKED - trigger to release' : 'Trigger to lock';
    this.label.requireElementById('orb-description').setProperties({
      text: `${KIND_LABEL[this.definition.kind]}\n${a.lat.toFixed(5)}, ${a.lon.toFixed(5)}\nAlt ${a.alt.toFixed(0)} m\n${hint}`,
    });
  }

  /**
   * How far a ray passes from the pin, in meters (Infinity if it points away). While the pin's beam
   * and label are up they count too, so the selection doesn't drop as the ray drifts off the sphere.
   */
  rayDistance(origin: Vector3, direction: Vector3): number {
    const heights = this.shown ? [0, PIN_RADIUS + this.beamLength / 2, PIN_RADIUS + this.beamLength] : [0];
    let best = Infinity;
    for (const height of heights) {
      this.group.localToWorld(tmpPoint.set(0, height, 0));
      tmpToPoint.subVectors(tmpPoint, origin);
      const along = tmpToPoint.dot(direction);
      if (along <= 0) continue;
      best = Math.min(best, Math.sqrt(Math.max(tmpToPoint.lengthSq() - along * along, 0)));
    }
    return best;
  }

  /** Show or hide the beam and label: up while hovered or locked, and never while a panel needs the space. */
  applyVisibility(showLabels: boolean): void {
    const on = showLabels && (this.hovered || this.locked);
    this.beamMaterial.color.setHex(this.locked ? LOCKED_BEAM_COLOR : BEAM_COLOR);
    if (on !== this.shown) this.descriptionDirty = true;
    if (on && !this.shown) this.justShown = true;
    this.shown = on;
    this.label.visible = on;
    this.beam.visible = on;
    if (this.panel) this.panel.group.visible = on && this.locked;
  }

  toggleLock(): void {
    this.locked = !this.locked;
    this.descriptionDirty = true;
    this.syncStream();
  }

  /** The camera runs only while the pin is locked. */
  private syncStream(): void {
    if (!this.cameraUrl) return;
    if (this.locked) {
      if (!this.panel) {
        this.panel = new CameraPanel(this.definition.name, this.openStream);
        this.panel.group.position.y = this.panelY;
        this.group.add(this.panel.group);
      }
      this.panel.start(this.cameraUrl);
    } else {
      this.panel?.stop();
    }
  }

  get isShown(): boolean {
    return this.shown;
  }

  get name(): string {
    return this.definition.name;
  }

  /** Length of the beam from the pin to the tag: the normal height plus any layout lift. */
  get beamLength(): number {
    return BEAM_HEIGHT + this.lift;
  }

  /** Map-space height of the tag's bottom edge with no lift. */
  get tagBottomAtRest(): number {
    return this.group.position.y + PIN_RADIUS + BEAM_HEIGHT;
  }

  /** The label's real height in map meters, measured from UIKit's layout (an estimate until it has one). */
  get labelHeight(): number {
    const callout = this.label.getElementById('callout');
    const size = callout?.size.value;
    if (!callout || !size) return LABEL_HEIGHT_FALLBACK;
    const pixelSize = callout.properties.value.pixelSize;
    return size[1] * (typeof pixelSize === 'number' ? pixelSize : UIKIT_PIXEL_SIZE) * LABEL_SCALE;
  }

  /** Where the camera panel's bottom edge sits: on top of the label, with PANEL_MARGIN between them. */
  private get panelY(): number {
    return PIN_RADIUS + this.beamLength + this.labelHeight + PANEL_MARGIN;
  }

  /** How tall the tag is: the label, plus the camera panel stacked on it while that is showing. */
  get tagHeight(): number {
    const panel = this.panel?.group.visible ? PANEL_MARGIN + this.panel.height : 0;
    return this.labelHeight + panel;
  }

  setTargetLift(lift: number): void {
    this.targetLift = lift;
  }

  /** Glide toward the layout's height, and move the beam, label and camera panel with it. */
  applyLift(deltaSeconds: number): void {
    if (this.justShown) {
      this.lift = this.targetLift; // a tag that just appeared goes straight to its place
      this.justShown = false;
    } else {
      this.lift += (this.targetLift - this.lift) * (1 - Math.exp(-LIFT_SMOOTHING * deltaSeconds));
      if (Math.abs(this.targetLift - this.lift) < 1e-4) this.lift = this.targetLift;
    }
    const length = this.beamLength;
    this.beam.scale.y = length / BEAM_HEIGHT;
    this.beam.position.y = PIN_RADIUS + length / 2;
    this.label.position.y = PIN_RADIUS + length;
    if (this.panel) this.panel.group.position.y = this.panelY;
  }

  /** Turn the label to face the headset, staying upright. */
  faceHead(head: Vector3): void {
    this.label.getWorldPosition(tmpLabel);
    this.label.lookAt(head.x, tmpLabel.y, head.z);
    if (this.panel?.group.visible) {
      this.panel.group.getWorldPosition(tmpLabel);
      this.panel.group.lookAt(head.x, tmpLabel.y, head.z);
      this.panel.update();
    }
  }
}

/** Live asset pins on the terrain. Getting close or pointing at one shows a beam and a label with its GPS fix. */
export class PinField {
  /** Add this to the map marker so the pins move in map space. */
  readonly root = new Group();
  private readonly pins = new Map<string, Pin>();
  private sinceUpdateS = 0;
  private armed = false;
  private readonly avoidTagOverlap: boolean;

  constructor(
    definitions: readonly PinDefinition[],
    labels: readonly UIKitMLAsset[],
    terrain: TerrainMap,
    options: PinFieldOptions = {},
  ) {
    this.avoidTagOverlap = options.avoidTagOverlap ?? true;
    definitions.forEach((definition, i) => {
      const cameraUrl = definition.camera && options.cameraUrl ? options.cameraUrl(definition.name) : null;
      const pin = new Pin(definition, labels[i], terrain, cameraUrl, options.openStream);
      this.pins.set(definition.name, pin);
      this.root.add(pin.group, pin.stemMesh);
    });
    this.root.visible = false;
  }

  /** What the operator is pointing at and has pinned open, by asset name. Sent with each voice question. */
  get focus(): { hovered: string | null; locked: string[] } {
    let hovered: string | null = null;
    const locked: string[] = [];
    for (const pin of this.pins.values()) {
      if (pin.hovered && pin.isShown) hovered = pin.name;
      if (pin.locked) locked.push(pin.name);
    }
    return { hovered, locked };
  }

  /** Feed a new batch of fixes. Unknown asset names are ignored. */
  setAssets(assets: readonly FeedAsset[]): void {
    this.sinceUpdateS = 0;
    for (const asset of assets) this.pins.get(asset.name)?.setAsset(asset);
  }

  /**
   * @param enabled Pins only show and move while true (i.e. once the app is ready).
   * @param showLabels Labels can be suppressed, e.g. while the settings panel is open.
   * @param pointers Tracked controllers. Their -Z axis is the pointing direction; a `select` on one
   *   toggles the lock of the pin that controller points at.
   */
  update(deltaSeconds: number, enabled: boolean, showLabels: boolean, head: Object3D, pointers: PinPointer[]): void {
    this.root.visible = enabled;
    // A trigger press that closes a panel or confirms the map must not also lock a pin. Accept
    // presses only if the pins were already live (and the panel already gone) on the previous frame.
    const acceptSelect = this.armed && enabled && showLabels;
    this.armed = enabled && showLabels;
    if (!enabled) return;

    head.getWorldPosition(tmpHead);
    const origins = pointers.map((p) => p.space.getWorldPosition(new Vector3()));
    const directions = pointers.map((p) => tmpDirection.set(0, 0, -1).transformDirection(p.space.matrixWorld).clone());

    const dt = MathUtils.clamp(deltaSeconds, 0, 0.1);
    this.sinceUpdateS += deltaSeconds;
    const visiblePins: Pin[] = [];
    for (const pin of this.pins.values()) {
      pin.track(dt, this.sinceUpdateS);
      if (pin.group.visible) visiblePins.push(pin);
      else pin.hovered = false;
    }

    // What each controller points at, and what is hovered: ONE pin at a time. A ray wins; failing
    // that, the pin nearest the headset within reach.
    const bestPerPointer: Array<Pin | null> = pointers.map(() => null);
    const bestDistance: number[] = pointers.map(() => POINT_RADIUS);
    let hovered: Pin | null = null;
    let hoveredScore = Infinity;
    for (const pin of visiblePins) {
      let nearest = Infinity;
      for (let i = 0; i < pointers.length; i++) {
        const distance = pin.rayDistance(origins[i], directions[i]);
        if (distance <= bestDistance[i]) {
          bestDistance[i] = distance;
          bestPerPointer[i] = pin;
        }
        nearest = Math.min(nearest, distance);
      }
      if (nearest <= POINT_RADIUS) {
        const score = nearest * (pin.hovered ? HOVER_STICKINESS : 1);
        if (score < hoveredScore) {
          hovered = pin;
          hoveredScore = score;
        }
      }
    }
    if (!hovered) {
      let nearestHead = Infinity;
      for (const pin of visiblePins) {
        pin.group.getWorldPosition(tmpPoint);
        const distance = tmpPoint.distanceTo(tmpHead);
        if (distance < (pin.hovered ? HIDE_DISTANCE : SHOW_DISTANCE) && distance < nearestHead) {
          hovered = pin;
          nearestHead = distance;
        }
      }
    }
    for (const pin of this.pins.values()) pin.hovered = showLabels && pin === hovered;

    if (acceptSelect) {
      pointers.forEach((pointer, i) => {
        if (pointer.select) bestPerPointer[i]?.toggleLock();
      });
    }

    const shown: Pin[] = [];
    for (const pin of this.pins.values()) {
      pin.applyVisibility(showLabels && pin.group.visible);
      if (pin.isShown) shown.push(pin);
    }
    this.layoutTags(shown);
    for (const pin of this.pins.values()) {
      pin.applyLift(dt);
      if (pin.isShown) {
        pin.faceHead(tmpHead);
        pin.refreshDescription();
      }
    }
  }

  /**
   * Give every showing tag a height at which it overlaps no other. Tags whose pins are within
   * TAG_WIDTH of each other on the map are stacked, each raised just above whatever it would touch.
   * Locked tags are placed first, so they stay put and a hovered tag finds room above them.
   * Pins that are far apart are untouched. The result does not depend on where the viewer stands,
   * so tags don't shuffle as you walk around the map.
   */
  private layoutTags(shown: readonly Pin[]): void {
    for (const pin of this.pins.values()) pin.setTargetLift(0);
    if (!this.avoidTagOverlap) return;

    const order = [...shown].sort((a, b) => Number(b.locked) - Number(a.locked) || a.name.localeCompare(b.name));
    const placed: Array<{ pin: Pin; bottom: number; top: number }> = [];
    for (const pin of order) {
      const rest = pin.tagBottomAtRest;
      const height = pin.tagHeight;
      const near = placed.filter(
        (other) => Math.hypot(other.pin.group.position.x - pin.group.position.x, other.pin.group.position.z - pin.group.position.z) < TAG_WIDTH,
      );
      // Raise until clear of every near tag whose height range it would share. Each step lifts it
      // above one of them, so this ends within near.length steps.
      let bottom = rest;
      for (let step = 0; step <= near.length; step++) {
        const hit = near.find((o) => bottom < o.top + TAG_GAP && bottom + height + TAG_GAP > o.bottom);
        if (!hit) break;
        bottom = hit.top + TAG_GAP;
      }
      const lift = Math.min(bottom - rest, MAX_LIFT);
      pin.setTargetLift(lift);
      placed.push({ pin, bottom: rest + lift, top: rest + lift + height });
    }
  }
}
