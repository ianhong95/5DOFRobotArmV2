#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QTcpSocket>
#include <QAbstractItemModel>

#include "robotarmclient.h"
#include "protocolconstants.h"
#include "protocolparser.h"
#include "messagetypes.h"
#include "messagehandler.h"
#include "robot3dview.h"

#include <cmath>

/*
    TODO:
        - Add camera feed
        - Add 3D view
        - Add lights/indicators
        - Set speed/acceleration using slider?
        - Implement gripper
*/

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    client = new RobotArmClient(this);
    parser = new ProtocolParser();
    teachPanel = ui->teachPanel;
    robot3DView = ui->visualizerWidget;

    // Open the video stream (adjust the string as needed)
    cap.open("tcp://robot-pi.local:60001", cv::CAP_FFMPEG);

    // Create and start timer
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateFrame);
    timer->start(30); // ~30 FPS

    connect(client, &RobotArmClient::connError, this, &MainWindow::handleClientConnError);
    connect(client, &RobotArmClient::connStatusChanged, this, &MainWindow::handleConnChanged);
    connect(client, &RobotArmClient::jointAnglesRecvd, this, &MainWindow::updateJointAngles);
    connect(client, &RobotArmClient::xyzPositionRecvd, this, &MainWindow::updateXYZPosition);
    connect(client, &RobotArmClient::savePosRespRecvd, this, &MainWindow::addSavedPosition);

    connect(teachPanel, &TeachPanel::saveCurrentPositionRequested, this, &MainWindow::saveCurrentPositionRequested);
    connect(teachPanel, &TeachPanel::moveToPositionRequested, this, &MainWindow::moveToPosition);
    connect(teachPanel, &TeachPanel::playCurrentSeqRequested, this, &MainWindow::playCurrentSequence);

    connect(robot3DView, &Robot3DView::onWebsocketConnected, this, &MainWindow::websocketConnected);
    connect(robot3DView, &Robot3DView::onWebsocketDisconnected, this, &MainWindow::websocketDisconnected);

    // Assign values to member variables
    connErrorMsg = QString::fromUtf8("Error: No socket connection.");
}

MainWindow::~MainWindow() {
    delete ui;
    delete client;
    delete parser;
    cap.release();
}

void MainWindow::updateFrame() {
    cv::Mat frame;
    if (cap.isOpened() && cap.read(frame)) {
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        QImage img((uchar*)frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
        ui->videoLabel->setPixmap(QPixmap::fromImage(img));
    }
}

/* --- Joint slider moved --- */

void MainWindow::on_J1Slider_sliderMoved(int position) {
    std::vector<float> jointAngle;
    ui->J1SpinBox->setValue(position);
    ui->visualizerWidget->moveJ1((float)position * M_PI / 180.0);

    jointAngle.insert(jointAngle.begin(), (float) position);
    std::vector<uint8_t> moveJ1Message = parser->encodeMessage(ProtocolConstants::RobotMessageType::SetJ1, jointAngle);
    client->sendMessage(moveJ1Message);
}

void MainWindow::on_J2Slider_sliderMoved(int position) {
    std::vector<float> jointAngle;
    ui->J2SpinBox->setValue(position);
    ui->visualizerWidget->moveJ2((float)position * M_PI / 180.0);

    jointAngle.insert(jointAngle.begin(), (float) position);
    std::vector<uint8_t> moveJ2Message = parser->encodeMessage(ProtocolConstants::RobotMessageType::SetJ2, jointAngle);
    client->sendMessage(moveJ2Message);
}

void MainWindow::on_J3Slider_sliderMoved(int position) {
    std::vector<float> jointAngle;
    ui->J3SpinBox->setValue(position);
    ui->visualizerWidget->moveJ3((float)position * M_PI / 180.0);

    jointAngle.insert(jointAngle.begin(), (float) position);
    std::vector<uint8_t> moveJ3Message = parser->encodeMessage(ProtocolConstants::RobotMessageType::SetJ3, jointAngle);
    client->sendMessage(moveJ3Message);
}

void MainWindow::on_J4Slider_sliderMoved(int position) {
    std::vector<float> jointAngle;
    ui->J4SpinBox->setValue(position);
    ui->visualizerWidget->moveJ4((float)position * M_PI / 180.0);

    jointAngle.insert(jointAngle.begin(), (float) position);
    std::vector<uint8_t> moveJ4Message = parser->encodeMessage(ProtocolConstants::RobotMessageType::SetJ4, jointAngle);
    client->sendMessage(moveJ4Message);
}

void MainWindow::on_J5Slider_sliderMoved(int position) {
    std::vector<float> jointAngle;
    ui->J5SpinBox->setValue(position);
    ui->visualizerWidget->rotateGripper((float)position * M_PI / 180.0);

    jointAngle.insert(jointAngle.begin(), (float) position);
    std::vector<uint8_t> moveJ5Message = parser->encodeMessage(ProtocolConstants::RobotMessageType::SetJ5, jointAngle);
    client->sendMessage(moveJ5Message);
}

/* --- Joint spinbox editing finished --- */

void MainWindow::on_J1SpinBox_editingFinished() {
    float angleInRadians = ui->J1SpinBox->value() * M_PI / 180.0;
    ui->visualizerWidget->moveJ1(angleInRadians);
}

void MainWindow::on_J2SpinBox_editingFinished() {
    ui->visualizerWidget->moveJ2((ui->J2SpinBox->value() * M_PI / 180.0));
}

void MainWindow::on_J3SpinBox_editingFinished() {
    ui->visualizerWidget->moveJ3((ui->J3SpinBox->value() * M_PI / 180.0));
}

void MainWindow::on_J4SpinBox_editingFinished() {
    ui->visualizerWidget->moveJ4((ui->J4SpinBox->value() * M_PI / 180.0));
}

void MainWindow::on_J5SpinBox_editingFinished() {
    ui->visualizerWidget->rotateGripper((ui->J5SpinBox->value() * M_PI / 180.0));
}

/* --- Joint spinbox value changed --- */

void MainWindow::on_J1SpinBox_valueChanged(double arg1) {
    ui->J1Slider->setValue(arg1);
}

void MainWindow::on_J2SpinBox_valueChanged(double arg1) {
    ui->J2Slider->setValue(arg1);
}

void MainWindow::on_J3SpinBox_valueChanged(double arg1) {
    ui->J3Slider->setValue(arg1);
}

void MainWindow::on_J4SpinBox_valueChanged(double arg1) {
    ui->J4Slider->setValue(arg1);
}

void MainWindow::on_J5SpinBox_valueChanged(double arg1) {
    ui->J5Slider->setValue(arg1);
}

/* --- Shortcut buttons clicked --- */

void MainWindow::on_connectButton_clicked() {
    if (client->connectionFlag == false) {
        client->connectToServer("n95-dev.local", 61234);
    }
    else {
        client->disconnectFromServer();
    }

    if (robot3DView->connectionFlag == false) {
        robot3DView->connectToServer("n95-dev.local", 60003);
    } else {
        robot3DView->disconnectFromServer();
    }
}

void MainWindow::on_disableAllButton_clicked() {
    if (client->connectionFlag == true) {
        std::vector<uint8_t> disableMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::Disable);
        client->sendMessage(disableMessage);
        ui->debugTextBrowser->append("Disabled robot.");
    }
    else {
        ui->debugTextBrowser->append(connErrorMsg);
    }
}

void MainWindow::on_homeButton_clicked() {
    if (client->connectionFlag == true) {
        std::vector<uint8_t> homeMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::Home);
        client->sendMessage(homeMessage);
        ui->debugTextBrowser->append("Moving robot to home position.");
    }
    else {
        ui->debugTextBrowser->append(connErrorMsg);
    }
}

/*
 * =======================
 * INCREMENTAL XYZ BUTTONS
 * =======================
 */

void MainWindow::on_negXButton_clicked() {
    if (client->connectionFlag == true) {
        std::vector<float> step_float_vector;
        std::vector<uint8_t> moveXMessage;

        QString step_text = ui->xStepLineEdit->text();

        step_float_vector.insert(step_float_vector.begin(), -step_text.toFloat());

        moveXMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::MoveX, step_float_vector);
        client->sendMessage(moveXMessage);

        ui->debugTextBrowser->append("Move X " + QString::number(step_float_vector[0]));
    }
    else {
        ui->debugTextBrowser->append(connErrorMsg);
    }
}

void MainWindow::on_posXButton_clicked() {

    if (client->connectionFlag == true) {
        std::vector<float> step_float_vector;
        std::vector<uint8_t> moveXMessage;

        QString step_text = ui->xStepLineEdit->text();
        step_float_vector.insert(step_float_vector.begin(), step_text.toFloat());

        moveXMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::MoveX, step_float_vector);
        client->sendMessage(moveXMessage);

        ui->debugTextBrowser->append("Move X " + QString::number(step_float_vector[0]));
    }
    else {
        ui->debugTextBrowser->append(connErrorMsg);
    }
}

void MainWindow::on_negYButton_clicked() {
    if (client->connectionFlag == true) {
        std::vector<float> step_float_vector;
        std::vector<uint8_t> moveYMessage;

        QString step_text = ui->yStepLineEdit->text();
        step_float_vector.insert(step_float_vector.begin(), -step_text.toFloat());

        moveYMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::MoveY, step_float_vector);
        client->sendMessage(moveYMessage);

        ui->debugTextBrowser->append("Move Y " + QString::number(step_float_vector[0]));
    }
    else {
        ui->debugTextBrowser->append(connErrorMsg);
    }
}

void MainWindow::on_posYButton_clicked() {
    if (client->connectionFlag == true) {
        std::vector<float> step_float_vector;
        std::vector<uint8_t> moveYMessage;

        QString step_text = ui->yStepLineEdit->text();
        step_float_vector.insert(step_float_vector.begin(), step_text.toFloat());

        moveYMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::MoveY, step_float_vector);
        client->sendMessage(moveYMessage);

        ui->debugTextBrowser->append("Move Y " + QString::number(step_float_vector[0]));
    }
    else {
        ui->debugTextBrowser->append(connErrorMsg);
    }
}

void MainWindow::on_negZButton_clicked() {
    if (client->connectionFlag == true) {
        std::vector<float> step_float_vector;
        std::vector<uint8_t> moveZMessage;

        QString step_text = ui->zStepLineEdit->text();
        step_float_vector.insert(step_float_vector.begin(), -step_text.toFloat());

        moveZMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::MoveZ, step_float_vector);
        client->sendMessage(moveZMessage);

        ui->debugTextBrowser->append("Move Z " + QString::number(step_float_vector[0]));
    }
    else {
        ui->debugTextBrowser->append(connErrorMsg);
    }
}

void MainWindow::on_posZButton_clicked() {
    if (client->connectionFlag == true) {
        std::vector<float> step_float_vector;
        std::vector<uint8_t> moveZMessage;

        QString step_text = ui->zStepLineEdit->text();
        step_float_vector.insert(step_float_vector.begin(), step_text.toFloat());

        moveZMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::MoveZ, step_float_vector);
        client->sendMessage(moveZMessage);

        ui->debugTextBrowser->append("Move Z " + QString::number(step_float_vector[0]));
    }
    else {
        ui->debugTextBrowser->append(connErrorMsg);
    }
}

void MainWindow::on_clearDebugButton_clicked() {
    ui->debugTextBrowser->clear();
}

/* ============
 * I/O HANDLING
 * ============
 */

void MainWindow::updateJointAngles(JointAngles jointAngles) {
    for (int i = 0; i < 5; ++i) {
        MainWindow::jointAngles[i] = jointAngles.angles[i];
    }

    ui->J1SpinBox->setValue(MainWindow::jointAngles[0]);
    ui->J2SpinBox->setValue(MainWindow::jointAngles[1]);
    ui->J3SpinBox->setValue(MainWindow::jointAngles[2]);
    ui->J4SpinBox->setValue(MainWindow::jointAngles[3]);
    ui->J5SpinBox->setValue(MainWindow::jointAngles[4]);

    ui->debugTextBrowser->append("Joint angles updated.");
}

void MainWindow::updateXYZPosition(XYZPosition xyzPosition) {
    for (int i = 0; i < 3; ++i) {
        MainWindow::xyzPosition[i] = xyzPosition.coordinates[i];
    }

    ui->xSpinBox->setValue(MainWindow::xyzPosition[0]);
    ui->ySpinBox->setValue(MainWindow::xyzPosition[1]);
    ui->zSpinBox->setValue(MainWindow::xyzPosition[2]);

    ui->debugTextBrowser->append("End effector position updated.");
}

/* ========
 * TEACHING
 * ======== */

// Save the robot's current physical position.
void MainWindow::saveCurrentPositionRequested() {
    std::vector<uint8_t> savePositionMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::SaveCurrentPosition);
    client->sendMessage(savePositionMessage);
    ui->debugTextBrowser->append("Requesting server to save position data.");
}

void MainWindow::addSavedPosition(SavedXYZPosition savedPositionData) {
    SavedPosition savedPosition;
    savedPosition.index = savedPositionData.index;
    savedPosition.alias = "None";
    savedPosition.x = savedPositionData.coordinates.coordinates[0];
    savedPosition.y = savedPositionData.coordinates.coordinates[1];
    savedPosition.z = savedPositionData.coordinates.coordinates[2];

    ui->debugTextBrowser->append(QString::number(savedPosition.x));
    ui->debugTextBrowser->append(QString::number(savedPosition.y));
    ui->debugTextBrowser->append(QString::number(savedPosition.z));

    teachPanel->positionTableModel->addRow(savedPosition);
}

void MainWindow::moveToPosition(int positionIndex) {
    std::vector<int> positionIndexVector{positionIndex};

    ui->debugTextBrowser->append("Requesting robot to move to selected position.");

    std::vector<uint8_t> moveToPositionMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::MoveToPosition, positionIndexVector);
    client->sendMessage(moveToPositionMessage);
}

void MainWindow::playCurrentSequence(std::vector<int> savedPositionIds) {
    ui->debugTextBrowser->append("Playing current saved positions.");

    std::vector<uint8_t> playCurrentSequenceMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::PlayCurrentSequence, savedPositionIds);
    client->sendMessage(playCurrentSequenceMessage);
}

/* ==========
 * CONNECTION
 * ========== */

void MainWindow::handleConnChanged(bool connected) {
    if (connected == true) {
        ui->statusLabel->setText("Connected!");
        ui->connectButton->setText("Disconnect");
        ui->debugTextBrowser->append("Robot has been connected!");
    }
    else {
        ui->statusLabel->setText("Not connected");
        ui->connectButton->setText("Connect");
        ui->debugTextBrowser->append("Robot has been disconnected.");
    }
}

void MainWindow::websocketConnected() {
    ui->debugTextBrowser->append("Websocket client connected.");
}

void MainWindow::websocketDisconnected() {
    ui->debugTextBrowser->append("Websocket client disconnected.");
}

/* ===================
 * ERROR HANDLING
 * ================== */

void MainWindow::handleClientConnError(const QString &errorMsg) {
    ui->debugTextBrowser->append("handleClientConnError triggered. Could not establish connection to socket. " + errorMsg);
}

/* ==================
 * FETCHING UPDATES
 * ================== */

void MainWindow::on_updJointAnglesButton_clicked() {
    std::vector<uint8_t> readJointAnglesMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::ReadJointAngles);
    client->sendMessage(readJointAnglesMessage);
    ui->J1SpinBox->setValue(MainWindow::jointAngles[0]);
    ui->J2SpinBox->setValue(MainWindow::jointAngles[1]);
    ui->J3SpinBox->setValue(MainWindow::jointAngles[2]);
    ui->J4SpinBox->setValue(MainWindow::jointAngles[3]);
    ui->J5SpinBox->setValue(MainWindow::jointAngles[4]);
}

void MainWindow::on_updXYZButton_clicked() {
    std::vector<uint8_t> updateXYZMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::UpdateEEPos);
    client->sendMessage(updateXYZMessage);
    ui->xSpinBox->setValue(MainWindow::xyzPosition[0]);
    ui->ySpinBox->setValue(MainWindow::xyzPosition[1]);
    ui->zSpinBox->setValue(MainWindow::xyzPosition[2]);
}

void MainWindow::on_openGripperButton_clicked() {
    ui->visualizerWidget->openGripper();
    std::vector<uint8_t> openGripperMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::OpenGripper);
    client->sendMessage(openGripperMessage);

    ui->gripperStateLbl->setText("Current gripper state: OPEN");
}

void MainWindow::on_closeGripperButton_clicked() {
    ui->visualizerWidget->closeGripper();
    std::vector<uint8_t> closeGripperMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::OpenGripper);
    client->sendMessage(closeGripperMessage);

    ui->gripperStateLbl->setText("Current gripper state: CLOSE");
}

void MainWindow::on_moveJ1Button_clicked() {
    float j1Angle = (ui->J1SpinBox->value()) * M_PI / 180.0;
    ui->visualizerWidget->moveJ1(j1Angle);
}

void MainWindow::on_moveJ2Button_clicked() {
    float j2Angle = (ui->J2SpinBox->value()) * M_PI / 180.0;
    ui->visualizerWidget->moveJ2(j2Angle);
}

void MainWindow::on_moveJ3Button_clicked() {
    float j3Angle = (ui->J3SpinBox->value()) * M_PI / 180.0;
    ui->visualizerWidget->moveJ3(j3Angle);
}

void MainWindow::on_moveJ4Button_clicked() {
    float j4Angle = (ui->J4SpinBox->value()) * M_PI / 180.0;
    ui->visualizerWidget->moveJ4(j4Angle);
}

void MainWindow::on_moveJ5Button_clicked() {
    float gripperAngle = (ui->J5SpinBox->value()) * M_PI / 180.0;
    ui->visualizerWidget->rotateGripper(gripperAngle);
}
