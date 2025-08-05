#include "teachpanel.h"
#include "ui_teachpanel.h"
#include "ui/models/positiontablemodel.h"

#include <iostream>



TeachPanel::TeachPanel(QWidget *parent) : QWidget(parent), ui(new Ui::TeachPanel) {
    ui->setupUi(this);

    positionTableModel = new PositionTableModel(this);

    // Connect the model to the table view
    ui->posMemoryTableView->setModel(positionTableModel);
}

TeachPanel::~TeachPanel() {
    delete ui;
}

void TeachPanel::on_saveCurrentButton_clicked() {
    emit getXYZPosition();
    emit saveCurrentPositionRequested();
}


void TeachPanel::on_moveToButton_clicked() {
    // selectionModel() accesses the selected items only
    QModelIndexList selectedRows = ui->posMemoryTableView->selectionModel()->selectedIndexes();

    if (!selectedRows.isEmpty()) {
        int selectedRow = selectedRows.first().row();

        // model() accesses the entire data model, returns the data source for the view, regardless of what's selected
        int positionEntryIndex = ui->posMemoryTableView->model()->index(selectedRow, 0).data().toInt();

        emit moveToPositionRequested(positionEntryIndex);
    }

}

