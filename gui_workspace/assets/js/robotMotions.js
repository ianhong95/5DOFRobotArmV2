/* Functions for actuating the robot arm in the visualizer.
 * All angles are in RADIANS.
 * When sending data over QWebChannel from C++ to JS, the data arrives as a STRING regardless
 * of how you sent it. Hence, we need parseFloat() to convert it into a float.
 * 
 * How tween works:
 *  // 1. Create a tween for an object
    const tween = new TWEEN.Tween(object);

    // 2. Define the target values and duration
    tween.to({ property: targetValue }, durationInMilliseconds);

    // 3. Start the tween
    tween.start();

    // 4. Update the tween in your animation loop
    TWEEN.update(); */

const TWEEN_DURATION = 1000;

function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene,camera);
    TWEEN.update();
}

/* --- RELATIVE ANGLE MOVES --- */

function moveJ1(angle) {
    // Negative rotation about Y = clockwise
    angle = parseFloat(angle);
    new TWEEN.Tween(ROBOT_LINKS['link1'].rotation).to({y: -angle}, TWEEN_DURATION).start();
}

function moveJ2(angle) {
    // Positive rotation about Y = clockwise (forward direction)
    angle = parseFloat(angle);
    new TWEEN.Tween(ROBOT_LINKS['link2'].rotation).to({y: angle}, TWEEN_DURATION).start();
}

function moveJ3(angle) {
    // Positive rotation about X = clockwise (forward direction)
    angle = parseFloat(angle);
    new TWEEN.Tween(ROBOT_LINKS['link3'].rotation).to({x: angle}, TWEEN_DURATION).start();
}

function moveJ4(angle) {
    // Positive rotation about X = clockwise (forward direction)
    angle = parseFloat(angle);
    new TWEEN.Tween(ROBOT_LINKS['link4'].rotation).to({x: angle}, TWEEN_DURATION).start();
}

function rotateGripper(angle) {
    // Positive rotation about Z = clockwise
    angle = parseFloat(angle);
    new TWEEN.Tween(ROBOT_LINKS['gripper'].rotation).to({z: angle}, TWEEN_DURATION).start();
}

/* --- ABSOLUTE ANGLE MOVES --- */

function setJ1(angle) {
    angle = parseFloat(angle);
    ROBOT_LINKS['link1'].rotation.set(0, -angle, 0);
}

function setJ2(angle) {
    angle = parseFloat(angle);
    ROBOT_LINKS['link2'].rotation.set(0, angle, 0);
}

function setJ3(angle) {
    angle = parseFloat(angle);
    ROBOT_LINKS['link3'].rotation.set(angle, 0, 0);
}

function setJ4(angle) {
    angle = parseFloat(angle);
    ROBOT_LINKS['link4'].rotation.set(angle, 0, 0);
}

function setJ5(angle) {
    angle = parseFloat(angle);
    ROBOT_LINKS['gripper'].rotation.set(0, 0, angle);
}

/* --- GRIPPER MOVES --- */

function closeGripper(distance=0.001) {
    // Right finger: positive x = close
    // Left finger: negative x = close
    distance = parseFloat(distance);
    ROBOT_LINKS['leftFinger'].position.set(-distance / SCALE, 0, 0);
    ROBOT_LINKS['rightFinger'].position.set(distance / SCALE, 0, 0);
}

function openGripper(distance=0.007) {
    // Right finger: negative x = open
    // Left finger: positive x = open
    distance = parseFloat(distance);
    ROBOT_LINKS['leftFinger'].position.set(distance / SCALE, 0, 0);
    ROBOT_LINKS['rightFinger'].position.set(-distance / SCALE, 0, 0);
}

/* --- FULL ARM MOVES --- */

function moveToPosition(angles) {
    setJ1(angles[0]);
    setJ2(angles[1]);
    setJ3(angles[2]);
    setJ4(angles[3]);
    setJ5(angles[4]);
}

function home(angles) {
    moveJ1(angles[0] * Math.PI / 180);
    moveJ2(angles[1] * Math.PI / 180);
    moveJ3(angles[2] * Math.PI / 180);
    moveJ4(angles[3] * Math.PI / 180);
    rotateGripper(angles[4] * Math.PI / 180);
}

animate();