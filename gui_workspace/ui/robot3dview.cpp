#include "robot3dview.h"

#include <QWebEngineProfile>
#include <QWebEngineSettings>
#include <QWebEngineView>
#include <QVBoxLayout>
#include <QCoreApplication>
#include <QDebug>

#include <vector>

Robot3DView::Robot3DView(QWidget *parent)
    : QWidget{parent}
    , m_webView(new QWebEngineView(this))
{
    // Create a box to hold the 3D window
    QVBoxLayout *layout = new QVBoxLayout(this);

    // Put the 3D window inside the box
    layout->addWidget(m_webView);

    // Load HTML file
    QString htmlPath = QCoreApplication::applicationDirPath() + "/../../assets/robot3d.html";
    qDebug() << "Qt version:" << QT_VERSION_STR;

    m_webView->load(QUrl::fromLocalFile(htmlPath));

    auto *devToolsView = new QWebEngineView;
    devToolsView->setAttribute(Qt::WA_DeleteOnClose);
    devToolsView->resize(1000, 700);
    devToolsView->show();

    m_webView->page()->setDevToolsPage(devToolsView->page());

    // TODO: Use QWebChannel and create a virtual robot controller class
}

void Robot3DView::moveJ1(float angle) {
    QString script = QString("moveJ1('%1');").arg(angle);
    m_webView->page()->runJavaScript(script);
}

void Robot3DView::moveJ2(float angle) {
    QString script = QString("moveJ2('%1');").arg(angle);
    m_webView->page()->runJavaScript(script);
}

void Robot3DView::moveJ3(float angle) {
    QString script = QString("moveJ3('%1');").arg(angle);
    m_webView->page()->runJavaScript(script);
}

void Robot3DView::moveJ4(float angle) {
    QString script = QString("moveJ4('%1');").arg(angle);
    m_webView->page()->runJavaScript(script);
}

void Robot3DView::rotateGripper(float angle) {
    QString script = QString("rotateGripper('%1');").arg(angle);
    m_webView->page()->runJavaScript(script);
}

void Robot3DView::openGripper() {
    QString script = QString("openGripper();");
    m_webView->page()->runJavaScript(script);
}

void Robot3DView::closeGripper() {
    QString script = QString("closeGripper();");
    m_webView->page()->runJavaScript(script);
}

void Robot3DView::moveJoints(float j1Angle, float j2Angle, float j3Angle, float j4Angle, float j5Angle) {
    QString script = QString("moveJoints('%1', '%2', '%3', '%4', '%5');").arg(j1Angle).arg(j2Angle).arg(j3Angle).arg(j4Angle).arg(j5Angle);
    m_webView->page()->runJavaScript(script);
}
