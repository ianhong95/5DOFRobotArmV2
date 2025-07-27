#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QTcpSocket>

#include "robotarmclient.h"
#include "protocolconstants.h"
#include "protocolparser.h"
#include "messagetypes.h"
#include "messagehandler.h"

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

    // Create label to display video
    // videoLabel = new QLabel(this);

    // Open the video stream (adjust the string as needed)
    cap.open("tcp://robot-pi.local:60001", cv::CAP_FFMPEG);

    // Create and start timer
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateFrame);
    timer->start(30); // ~30 FPS

    connect(client, &RobotArmClient::connError, this, &MainWindow::handleClientConnError);
    connect(client, &RobotArmClient::connStatusChanged, this, &MainWindow::handleConnChanged);
    connect(client, &RobotArmClient::jointAnglesRecvd, this, &MainWindow::readJointAngles);
    connect(client, &RobotArmClient::xyzPositionRecvd, this, &MainWindow::readXYZPosition);

    // Assign values to member variables
    connErrorMsg = QString::fromUtf8("Error: No socket connection.");
}

MainWindow::~MainWindow() {
    delete ui;
    delete client;
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

void MainWindow::on_J1Slider_sliderMoved(int position) {
//    ui->J1SpinBox->setText(QString::number(position));
    ui->J1SpinBox->setValue(position);
}

void MainWindow::on_J1SpinBox_editingFinished() {

}

void MainWindow::on_J1SpinBox_valueChanged(double arg1) {
    ui->J1Slider->setValue(arg1);
    // client->sendMessage(QString::number(arg1));
}

void MainWindow::on_connectButton_clicked() {
    if (client->connectionFlag == false) {
        client->connectToServer("robot-pi.local", 61234);
    }
    else {
        client->disconnectFromServer();
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

void MainWindow::readJointAngles(JointAngles jointAngles) {
    for (int i = 0; i < 5; ++i) {
        MainWindow::jointAngles[i] = jointAngles.angles[i];
    }

    ui->J1SpinBox->setValue(MainWindow::jointAngles[0]);
    ui->J2SpinBox->setValue(MainWindow::jointAngles[1]);
    ui->J3SpinBox->setValue(MainWindow::jointAngles[2]);
    ui->J4SpinBox->setValue(MainWindow::jointAngles[3]);
    ui->J5SpinBox->setValue(MainWindow::jointAngles[4]);
}

void MainWindow::readXYZPosition(XYZPosition xyzPosition) {
    for (int i = 0; i < 3; ++i) {
        MainWindow::xyzPosition[i] = xyzPosition.coordinates[i];
    }

    ui->xSpinBox->setValue(MainWindow::xyzPosition[0]);
    ui->ySpinBox->setValue(MainWindow::xyzPosition[1]);
    ui->zSpinBox->setValue(MainWindow::xyzPosition[2]);
}

/* ===========
 * CONNECTION
 * ==========
 */

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

/* ===================
 * ERROR HANDLING
 * ================== */

void MainWindow::handleClientConnError(const QString &errorMsg) {
    ui->debugTextBrowser->append("Could not establish connection to socket. " + errorMsg);
}

void MainWindow::on_updJointAnglesButton_clicked()
{
    std::vector<uint8_t> readJointAnglesMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::ReadJointAngles);
    client->sendMessage(readJointAnglesMessage);
    ui->J1SpinBox->setValue(MainWindow::jointAngles[0]);
    ui->J2SpinBox->setValue(MainWindow::jointAngles[1]);
    ui->J3SpinBox->setValue(MainWindow::jointAngles[2]);
    ui->J4SpinBox->setValue(MainWindow::jointAngles[3]);
    ui->J5SpinBox->setValue(MainWindow::jointAngles[4]);
}


void MainWindow::on_updXYZButton_clicked()
{
    std::vector<uint8_t> updateXYZMessage = parser->encodeMessage(ProtocolConstants::RobotMessageType::UpdateEEPos);
    client->sendMessage(updateXYZMessage);
    ui->xSpinBox->setValue(MainWindow::xyzPosition[0]);
    ui->ySpinBox->setValue(MainWindow::xyzPosition[1]);
    ui->zSpinBox->setValue(MainWindow::xyzPosition[2]);
}

