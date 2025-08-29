/* Functions for actuating the robot arm in the visualizer.
 * All angles are in RADIANS.
 * When sending data over QWebChannel from C++ to JS, the data arrives as a STRING regardless
 * of how you sent it. Hence, we need parseFloat() to convert it into a float.*/

/* --- RELATIVE ANGLE MOVES --- */

function moveJ1(angle) {
    // Negative rotation about Y = clockwise
    angle = parseFloat(angle);
    ROBOT_LINKS['link1'].rotation.y -= angle;
}

function moveJ2(angle) {
    // Positive rotation about Y = clockwise (forward direction)
    angle = parseFloat(angle);
    ROBOT_LINKS['link2'].rotation.y += angle;
}

function moveJ3(angle) {
    // Positive rotation about X = clockwise (forward direction)
    angle = parseFloat(angle);
    ROBOT_LINKS['link3'].rotation.x += angle;
}

function moveJ4(angle) {
    // Negative rotation about X = clockwise (forward direction)
    angle = parseFloat(angle);
    ROBOT_LINKS['link4'].rotation.x += angle;
}

function rotateGripper(angle) {
    // Positive rotation about Z = clockwise
    angle = parseFloat(angle);
    ROBOT_LINKS['gripper'].rotation.z += angle;
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
    ROBOT_LINKS['link5'].rotation.set(0, 0, angle);
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
    moveJ1(angles[0]);
    moveJ2(angles[1]);
    moveJ3(angles[2]);
    moveJ4(angles[3]);
    rotateGripper(angles[4]);
}
