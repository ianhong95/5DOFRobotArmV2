#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    ui->statusLabel->setText("Hi beebo");
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
}


void MainWindow::on_J1SpinBox_editingFinished()
{

}


void MainWindow::on_J1SpinBox_valueChanged(double arg1)
{
    ui->J1Slider->setValue(arg1);
}

