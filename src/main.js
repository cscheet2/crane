import $ from 'jquery';
import * as THREE from 'three';
import { scene, renderer, camera, loader } from './globals.js';
import './crane.js';

$(function() {
  scene.background = new THREE.Color(0xffffff);
  renderer.setSize(window.innerWidth, window.innerHeight);
  camera.position.set(0, 2, 5);
  camera.lookAt(0, 0, 0);

  const cube_geometry = new THREE.BoxGeometry(1, 3, 1);
  const cube_material = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
  const cube = new THREE.Mesh(cube_geometry, cube_material);
  // scene.add(cube);

  const cube_edges = new THREE.EdgesGeometry(cube_geometry);
  const line_material = new THREE.LineBasicMaterial({ color: 0x000000 });
  const line_segments = new THREE.LineSegments(cube_edges, line_material);
  // scene.add(line_segments);

  const light = new THREE.DirectionalLight(0xffffff, 1);
  light.position.set(10, 10, 10);
  scene.add(light);

  renderer.setAnimationLoop(() => {
    renderer.render(scene, camera);
    cube.rotation.x += 0.01;
    cube.rotation.y += 0.01;
    line_segments.rotation.x = cube.rotation.x;
    line_segments.rotation.y = cube.rotation.y;
  });
});
