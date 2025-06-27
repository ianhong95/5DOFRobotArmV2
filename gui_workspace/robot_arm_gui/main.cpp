#include "mainwindow.h"

#include <QApplication>
#include <QFile>
#include <QTextStream>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Need to initialize the resource, since we're using an external
    // build system and this isn't automatically handled by CMake.
    Q_INIT_RESOURCE(breeze);
    QFile file(":/dark/stylesheet.qss");
    file.open(QFile::ReadOnly | QFile::Text);
    QTextStream stream(&file);
    a.setStyleSheet(stream.readAll());

    MainWindow w;
    w.show();
    return a.exec();
}
