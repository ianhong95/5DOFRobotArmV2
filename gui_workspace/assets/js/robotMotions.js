/* Functions for actuating the robot arm in the visualizer.
 * All angles are in RADIANS.
 * TODO: set scaling so that position is in millimeters. */

function moveJ1(angle) {
    // Negative rotation about Y = clockwise
    ROBOT_LINKS['link1'].rotation.y -= angle;
}

function moveJ2(angle) {
    // Positive rotation about Y = clockwise (forward direction)
    ROBOT_LINKS['link2'].rotation.y += angle;
}

function moveJ3(angle) {
    // Positive rotation about Y = clockwise (forward direction)
    ROBOT_LINKS['link3'].rotation.y += angle;
}

function moveJ4(angle) {
    // Negative rotation about Y = clockwise (forward direction)
    ROBOT_LINKS['link4'].rotation.y -= angle;
}

function rotateGripper(angle) {
    // Positive rotation about Z = clockwise
    ROBOT_LINKS['gripper'].rotation.z += angle;
}

function closeGripper(distance=0.05) {
    // Right finger: negative z = close
    // Left finger: positive z = close
    ROBOT_LINKS['leftFinger'].position.z += distance;
    ROBOT_LINKS['rightFinger'].position.z -= distance;
}

function openGripper(distance=0.05) {
    // Right finger: positive z = open
    // Left finger: negative z = open
    ROBOT_LINKS['leftFinger'].position.z -= distance;
    ROBOT_LINKS['rightFinger'].position.z += distance;
}

function moveToPosition(angles) {
    moveJ1(angles[0]);
    moveJ2(angles[1]);
    moveJ3(angles[2]);
    moveJ4(angles[3]);
    rotateGripper(angles[4]);
}

function home(angles) {
    moveJ1(angles[0]);
    moveJ2(angles[1]);
    moveJ3(angles[2]);
    moveJ4(angles[3]);
    rotateGripper(angles[4]);
}
