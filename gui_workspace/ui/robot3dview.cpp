#include "robot3dview.h"

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
    qDebug() << htmlPath;

    m_webView->load(QUrl::fromLocalFile(htmlPath));
}
