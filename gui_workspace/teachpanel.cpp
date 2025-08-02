#include "teachpanel.h"
#include "ui_teachpanel.h"

#include <iostream>



TeachPanel::TeachPanel(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::TeachPanel)
{
    ui->setupUi(this);
}

TeachPanel::~TeachPanel()
{
    delete ui;
}

void TeachPanel::on_saveCurrentButton_clicked()
{
    emit saveCurrentPositionRequested();
}

