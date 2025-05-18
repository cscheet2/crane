import $ from 'jquery';
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

export const canvas = $('#crane')[0];
export const scene = new THREE.Scene();
export const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
export const renderer = new THREE.WebGLRenderer({ canvas });
export const loader = new GLTFLoader();

scene.background = new THREE.Color(0xffffff);
renderer.setSize(window.innerWidth, window.innerHeight);
camera.position.set(0, 2, 5);
camera.lookAt(0, 0, 0);
