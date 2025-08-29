// Load models
const loader = new THREE.GLTFLoader();

const SCALE = 1000;
const SCALE_FACTOR = 1;

ROBOT_LINKS = {}

/* GLTF loading MUST be asynchronous.
 * The Promise object constructor automatically takes a resolve and reject parameter.
 * Resolve means the Promise is complete and now has a value so you can continue.
 * GLTF is the entire file data (including textures, etc).
 * gltf.scene is only the ROOT of that file (the actual 3D model).
 * */

// This function loads the root data (3D model only) from a single gltf.
const loadGLTF = (filePath) => {
    return new Promise((resolve, reject) => {
        loader.load(
            filePath,
            (model) => {resolve(model.scene)},
            undefined,
            (error) => reject(error)
        )
    })
}

// Load all parts in parallel. This is faster than doing individual await loadGLTF() calls.
async function loadRobot() {
    const [base, link1, link2, link3, link4, gripper, leftFinger, rightFinger] = await Promise.all([
        loadGLTF('models/Robot_Base.glb'),
        loadGLTF('models/Link_1.glb'),
        loadGLTF('models/Link_2.glb'),
        loadGLTF('models/Link_3.glb'),
        loadGLTF('models/Link_4.glb'),
        loadGLTF('models/Gripper.glb'),
        loadGLTF('models/Gripper_finger.glb'),
        loadGLTF('models/Gripper_finger.glb')
    ])

    base.scale.set(SCALE, SCALE, SCALE);
    setupPart(base, scene, 'base', [0, 0, 0], [0, 0, 0]);
    setupPart(link1, base, 'link1', [0, 0.075 / SCALE, 0], [0, 0, 0]);
    setupPart(link2, link1, 'link2', [0, 0.02996 / SCALE, 0], [-Math.PI/2, 0, Math.PI/2]);
    setupPart(link3, link2, 'link3', [0, 0, 0.180 / SCALE], [0, 0, 0]);
    setupPart(link4, link3, 'link4', [0, 0, 0.160 / SCALE], [Math.PI, 0, 0]);
    setupPart(gripper, link4, 'gripper', [0, 0, -0.0722 / SCALE], [0, 0, 0]);
    setupPart(leftFinger, gripper, 'leftFinger', [0.001 / SCALE, 0, 0], [0, 0, 0]);
    setupPart(rightFinger, gripper, 'rightFinger', [-0.001 / SCALE, 0, 0], [0, 0, Math.PI]);
}

function setupPart(model, parent, name, position, rotation) {
    parent.add(model);
    model.position.set(...position);
    model.rotation.set(...rotation);
    ROBOT_LINKS[name] = model;
}

loadRobot();
