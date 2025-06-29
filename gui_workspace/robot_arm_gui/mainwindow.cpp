#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QTcpSocket>
#include "robotarmclient.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
   client = new RobotArmClient(this);
}

MainWindow::~MainWindow()
{
    delete ui;
//    delete client;
}

void MainWindow::on_pushButton_clicked()
{
    ui->statusLabel->setText("Hi beebo");
    client->sendCommand("f");
}

void MainWindow::on_J1Slider_sliderMoved(int position)
{
//    ui->J1SpinBox->setText(QString::number(position));
    ui->J1SpinBox->setValue(position);
}


void MainWindow::on_pushButton_2_clicked()
{
    ui->statusLabel->setText("Bye beebo");
}


void MainWindow::on_pushButton_3_clicked()
{
    ui->statusLabel->setText("Poop");
    client->connectToServer("127.0.0.1", 61234);
}


void MainWindow::on_J1SpinBox_editingFinished()
{

}


void MainWindow::on_J1SpinBox_valueChanged(double arg1)
{
    ui->J1Slider->setValue(arg1);
    client->sendCommand(QString::number(arg1));
}

