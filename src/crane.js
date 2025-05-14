import { scene, renderer, camera, loader } from './globals.js';
console.log(':3');
loader.load(
  'models/tower_crane.glb',
  (gltf) => { 
    gltf.scene.position.set(0, 0, 0);
    gltf.scene.scale.set(0.05, 0.05, 0.05);
    scene.add(gltf.scene); 
  },
  undefined,
  (error) => { 
    console.error('failed loading crane: ', error); 
  }
);