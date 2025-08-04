#ifndef TEACHPANEL_H
#define TEACHPANEL_H

#include <QWidget>
#include <QAbstractItemModel>
#include <QAbstractTableModel>

#include "ui/models/positiontablemodel.h"


namespace Ui {
class TeachPanel;
}

class TeachPanel : public QWidget {
    Q_OBJECT

public:
    explicit TeachPanel(QWidget *parent = nullptr);
    ~TeachPanel();

    PositionTableModel *positionTableModel;

private slots:
    void on_saveCurrentButton_clicked();

    void on_moveToButton_clicked();

private:
    Ui::TeachPanel *ui;

signals:
    void saveCurrentPositionRequested();
    void getXYZPosition();
    void moveToPositionRequested(int positionIndex);
};

#endif // TEACHPANEL_H
