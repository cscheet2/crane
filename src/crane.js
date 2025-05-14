import { scene, renderer, camera, loader } from './globals.js';

loader.load(
  '/models/tower_crane.glb',
  (gltf) => { scene.add(gltf.scene); },
  undefined,
  (error) => { console.error('failed loading crane: ', error); }
);