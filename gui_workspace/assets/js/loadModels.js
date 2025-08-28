// Load models
const loader = new THREE.GLTFLoader();

const SCALE = 1000;
const SCALE_FACTOR = 2;

const ROBOT_LINKS = {};

loadModel(
    'models/Robot_Base.glb',
    [0, 0, 0],
    [0, 0, 0],
    'base'
);

loadModel(
    'models/Link_1.glb',
    [0, 0.0694 * SCALE_FACTOR, 0],
    [0, 0, 0],
    'link1'
);

loadModel(
    'models/Link_2.glb',
    [0, 0.10436 * SCALE_FACTOR, 0],
    [-Math.PI/2, 0, Math.PI/2],
    'link2'
);

loadModel(
    'models/Link_3.glb',
    [0, 0.28436 * SCALE_FACTOR, 0],
    [-Math.PI/2, 0, Math.PI/2],
    'link3'
);

loadModel(
    'models/Link_4.glb',
    [0, 0.44436 * SCALE_FACTOR, 0],
    [Math.PI/2, 0, Math.PI/2],
    'link4'
);

loadModel(
    'models/Gripper.glb',
    [0, 0.51656 * SCALE_FACTOR, 0],
    [Math.PI/2, 0, Math.PI/2],
    'gripper'
);

loadModel(
    'models/Gripper_finger.glb',
    [0, 0.51656 * SCALE_FACTOR, 0],
    [Math.PI/2, 0, Math.PI/2],
    'rightFinger'
);

loadModel(
    'models/Gripper_finger.glb',
    [0, 0.51656 * SCALE_FACTOR, 0],
    [Math.PI/2, 0, -Math.PI/2],
    'leftFinger'
);


function loadModel(modelPath, position, rotation, name) {
    loader.load(
        modelPath,
        (gltf) => initializeModel(gltf, position, rotation, name),
        undefined,
        (error) => console.error('Error loading mode: ', error)
    )
}

function initializeModel (gltf, position, rotation, name) {
    const model = gltf.scene;
    scene.add(model);

    ROBOT_LINKS[name] = model;

    model.position.set(...position);
    model.rotation.set(...rotation);
    model.scale.set(SCALE * SCALE_FACTOR, SCALE * SCALE_FACTOR, SCALE * SCALE_FACTOR);

    const axesHelper = new THREE.AxesHelper(0.1);
    model.add(axesHelper);
}