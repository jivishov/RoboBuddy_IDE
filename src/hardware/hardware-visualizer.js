import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {CanonicalRobotRig} from '../canonical-rig.js';
export class HardwareVisualizer {
  async init(canvas) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({canvas, antialias: true});
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.scene = new THREE.Scene(); this.scene.background = new THREE.Color('#eeefe8');
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x68776a, 2.5));
    const light = new THREE.DirectionalLight(0xffffff, 3); light.position.set(500, 900, 400); this.scene.add(light);
    this.rig = await CanonicalRobotRig.load('so101'); this.scene.add(this.rig.root);
    const box = new THREE.Box3().setFromObject(this.rig.root);
    const center = box.getCenter(new THREE.Vector3()), size = box.getSize(new THREE.Vector3());
    const radius = Math.max(size.x, size.y, size.z, 200);
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(radius * 4, radius * 4), new THREE.MeshStandardMaterial({color: '#dce2d7', roughness: 1}));
    floor.rotation.x = -Math.PI / 2; floor.position.y = box.min.y - 1; this.scene.add(floor);
    this.camera = new THREE.PerspectiveCamera(40, 1, 1, 10000);
    this.camera.position.copy(center).add(new THREE.Vector3(radius * 1.2, radius * .85, radius * 1.7));
    this.controls = new OrbitControls(this.camera, canvas); this.controls.target.copy(center); this.controls.enableDamping = true;
    this.resizeObserver = new ResizeObserver(() => this.resize()); this.resizeObserver.observe(canvas.parentElement);
    canvas.hidden = false; this.resize();
    this.renderer.setAnimationLoop(() => {this.controls.update(); this.renderer.render(this.scene, this.camera);});
  }
  resize() {
    if (!this.renderer) return;
    const {width, height} = this.canvas.parentElement.getBoundingClientRect();
    this.renderer.setSize(width, height, false); this.camera.aspect = width / Math.max(1, height); this.camera.updateProjectionMatrix();
  }
  update(pose) { if (this.rig && pose) this.rig.applyPhysicalState(pose); }
}

