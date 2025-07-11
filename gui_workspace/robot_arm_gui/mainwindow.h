#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "robotarmclient.h"

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
    RobotArmClient* client;

private slots:
    void on_pushButton_clicked();
    void on_J1Slider_sliderMoved(int position);
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_J1SpinBox_editingFinished();
    void on_J1SpinBox_valueChanged(double arg1);
};

#endif // MAINWINDOW_H
