#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <vector>

#include <QMainWindow>
#include <QLabel>
#include <QTimer>

#include <vector>

#include <opencv2/opencv.hpp>

#include "robotarmclient.h"
#include "teachpanel.h"
#include "ui_mainwindow.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    RobotArmClient *client;
    ProtocolParser *parser;
    TeachPanel *teachPanel;


    QLabel *videoLabel;
    QTimer *timer;
    cv::VideoCapture cap;

    QString connErrorMsg;

    std::vector<float> jointAngles = std::vector<float>(5);
    std::vector<float> xyzPosition = std::vector<float>(3);

private slots:
    // FETCH UPDATE HANDLING
    void updateJointAngles(JointAngles jointAngles);
    void updateXYZPosition(XYZPosition xyzPosition);

    // CONNECTION HANDLING
    void handleConnChanged(bool connected);

    void on_J1Slider_sliderMoved(int position);
    void on_J1SpinBox_editingFinished();
    void on_J1SpinBox_valueChanged(double arg1);
    void on_connectButton_clicked();
    void on_disableAllButton_clicked();
    void on_homeButton_clicked();

    // Incremental XYZ buttons
    void on_negXButton_clicked();
    void on_posXButton_clicked();
    void on_negYButton_clicked();
    void on_posYButton_clicked();
    void on_negZButton_clicked();
    void on_posZButton_clicked();

    void on_clearDebugButton_clicked();

    // OpenCV
    void updateFrame();

    // Error Handling
    void handleClientConnError(const QString &errorMsg);

    // UPDATE BUTTONS
    void on_updJointAnglesButton_clicked();
    void on_updXYZButton_clicked();

    // TEACHING
    void saveCurrentPositionRequested();
    void addSavedPosition(SavedXYZPosition savedPositionData);
    void moveToPosition(int positionIndex);
    void playCurrentSequence(std::vector<int> savedPositionIds);

    // MANUAL CONTROLS
    void on_openGripperButton_clicked();
    void on_closeGripperButton_clicked();
};

#endif // MAINWINDOW_H
