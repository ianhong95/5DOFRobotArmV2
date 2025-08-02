#ifndef TEACHPANEL_H
#define TEACHPANEL_H

#include <QWidget>

namespace Ui {
class TeachPanel;
}

class TeachPanel : public QWidget {
    Q_OBJECT

public:
    explicit TeachPanel(QWidget *parent = nullptr);
    ~TeachPanel();

private slots:
    void on_saveCurrentButton_clicked();

private:
    Ui::TeachPanel *ui;

signals:
    void saveCurrentPositionRequested();
};

#endif // TEACHPANEL_H
