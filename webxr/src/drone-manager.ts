import * as THREE from 'three';

interface DroneState {
  id: number;
  mesh: THREE.Group;
  selected: boolean;
  flying: boolean;
  baseY: number;
  flyTime: number;
  originalColors: Map<THREE.Mesh, THREE.Color>;
}

export class DroneManager {
  private drones: DroneState[] = [];
  private nextId = 0;
  private template: THREE.Group | null = null;
  private scene: THREE.Scene;
  private _selected: DroneState | null = null;

  constructor(scene: THREE.Scene) {
    this.scene = scene;
  }

  setTemplate(model: THREE.Group) {
    this.template = model;
  }

  get ready() {
    return this.template !== null;
  }

  get count() {
    return this.drones.length;
  }

  get hasSelection() {
    return this._selected !== null;
  }

  spawn(position: THREE.Vector3): boolean {
    if (!this.template) return false;

    const clone = this.template.clone();
    const originalColors = new Map<THREE.Mesh, THREE.Color>();

    clone.traverse((child) => {
      if (child instanceof THREE.Mesh) {
        child.material = (child.material as THREE.Material).clone();
        if (child.material instanceof THREE.MeshStandardMaterial) {
          originalColors.set(child, child.material.color.clone());
        }
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });

    clone.position.copy(position);
    clone.visible = true;
    this.scene.add(clone);

    this.drones.push({
      id: this.nextId++,
      mesh: clone,
      selected: false,
      flying: false,
      baseY: position.y,
      flyTime: 0,
      originalColors,
    });

    return true;
  }

  selectByRay(origin: THREE.Vector3, direction: THREE.Vector3): boolean {
    const raycaster = new THREE.Raycaster(origin, direction);
    const meshes: THREE.Object3D[] = [];

    for (const drone of this.drones) {
      drone.mesh.traverse((child) => {
        if (child instanceof THREE.Mesh) meshes.push(child);
      });
    }

    const hits = raycaster.intersectObjects(meshes, false);
    if (hits.length === 0) {
      this.deselectAll();
      return false;
    }

    const hitObj = hits[0].object;
    for (const drone of this.drones) {
      let found = false;
      drone.mesh.traverse((child) => {
        if (child === hitObj) found = true;
      });
      if (found) {
        if (drone === this._selected) {
          this.deselectAll();
          return true;
        }
        this.deselectAll();
        this.applySelection(drone);
        return true;
      }
    }

    return false;
  }

  private applySelection(drone: DroneState) {
    drone.selected = true;
    this._selected = drone;
    drone.mesh.traverse((child) => {
      if (child instanceof THREE.Mesh && child.material instanceof THREE.MeshStandardMaterial) {
        child.material.emissive.setHex(0x00ffaa);
        child.material.emissiveIntensity = 0.4;
      }
    });
  }

  deselectAll() {
    if (!this._selected) return;
    this._selected.mesh.traverse((child) => {
      if (child instanceof THREE.Mesh && child.material instanceof THREE.MeshStandardMaterial) {
        child.material.emissive.setHex(0x000000);
        child.material.emissiveIntensity = 0;
      }
    });
    this._selected.selected = false;
    this._selected = null;
  }

  deleteSelected(): boolean {
    if (!this._selected) return false;
    this.scene.remove(this._selected.mesh);
    this.drones = this.drones.filter((d) => d !== this._selected);
    this._selected = null;
    return true;
  }

  deleteAll() {
    for (const drone of this.drones) {
      this.scene.remove(drone.mesh);
    }
    this.drones = [];
    this._selected = null;
  }

  scaleSelected(factor: number): boolean {
    if (!this._selected) return false;
    this._selected.mesh.scale.multiplyScalar(factor);
    return true;
  }

  rotateSelected(radians: number): boolean {
    if (!this._selected) return false;
    this._selected.mesh.rotation.y += radians;
    return true;
  }

  moveSelected(position: THREE.Vector3): boolean {
    if (!this._selected) return false;
    this._selected.mesh.position.copy(position);
    this._selected.baseY = position.y;
    return true;
  }

  toggleFlySelected(): boolean {
    if (!this._selected) return false;
    this._selected.flying = !this._selected.flying;
    if (!this._selected.flying) {
      this._selected.mesh.position.y = this._selected.baseY;
    }
    return true;
  }

  landSelected(): boolean {
    if (!this._selected) return false;
    this._selected.flying = false;
    this._selected.mesh.position.y = this._selected.baseY;
    return true;
  }

  colorSelected(hex: number): boolean {
    if (!this._selected) return false;
    this._selected.mesh.traverse((child) => {
      if (child instanceof THREE.Mesh && child.material instanceof THREE.MeshStandardMaterial) {
        child.material.color.setHex(hex);
      }
    });
    return true;
  }

  resetColorSelected(): boolean {
    if (!this._selected) return false;
    const drone = this._selected;
    drone.mesh.traverse((child) => {
      if (child instanceof THREE.Mesh) {
        const orig = drone.originalColors.get(child);
        if (orig && child.material instanceof THREE.MeshStandardMaterial) {
          child.material.color.copy(orig);
        }
      }
    });
    return true;
  }

  getSelectedMesh(): THREE.Group | null {
    return this._selected?.mesh ?? null;
  }

  update(dt: number) {
    for (const drone of this.drones) {
      if (drone.flying) {
        drone.flyTime += dt;
        drone.mesh.position.y = drone.baseY + 0.3 + Math.sin(drone.flyTime * 1.5) * 0.15;
        drone.mesh.rotation.y += dt * 0.5;
      }
    }
  }
}
