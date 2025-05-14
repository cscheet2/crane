import $ from 'jquery';
import * as THREE from 'three';

$(function() {
  const canvas = $('#crane')[0] as HTMLCanvasElement;

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
  const renderer = new THREE.WebGLRenderer({ canvas });

  renderer.setSize(window.innerWidth, window.innerHeight);
  camera.position.z = 5;

  const cube_geometry = new THREE.BoxGeometry(1, 1, 1);
  const cube_material = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
  const cube = new THREE.Mesh(cube_geometry, cube_material);
  scene.add(cube);

  const cube_edges = new THREE.EdgesGeometry(cube_geometry);
  const line_material = new THREE.LineBasicMaterial({ color: 0x000000 });
  const line_segments = new THREE.LineSegments(cube_edges, line_material);
  scene.add(line_segments)

  renderer.setAnimationLoop(() => {
    renderer.render(scene, camera);
    cube.rotation.x += 0.01;
    cube.rotation.y += 0.01;
    line_segments.rotation.x = cube.rotation.x;
    line_segments.rotation.y = cube.rotation.y;
  });
});
