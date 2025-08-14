// Set up 3D scene
const width = document.body.clientWidth;
const height = document.body.clientHeight;
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
const renderer = new THREE.WebGLRenderer();

// Set up the renderer (this is like setting up your TV)
renderer.setSize(width, height);
renderer.setClearColor(0x87CEEB); // Sky blue background
document.body.appendChild(renderer.domElement);

// Simple lighting
const ambientLight = new THREE.AmbientLight(0xffffff, 1.0);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 1.0);
directionalLight.position.set(10, 10, 10);
scene.add(directionalLight);

// Set up camera
camera.position.set(0.75, 0.75, 0.75);
camera.lookAt(0.25, 0.5, -0.25);
camera.updateProjectionMatrix();

// Add axes helper so we can see something
const axesHelper = new THREE.AxesHelper(0.5);
scene.add(axesHelper);