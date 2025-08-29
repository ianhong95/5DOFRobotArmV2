#include "robot3dview.h"

#include <QWebEngineProfile>
#include <QWebEngineSettings>
#include <QWebEngineView>
#include <QVBoxLayout>
#include <QCoreApplication>
#include <QDebug>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QtWebSockets/QWebSocket>
#include <QUrl>

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
    qDebug() << "HTML path:" << htmlPath;

    // Set up message handlers
    setupMessageHandlers();

    m_webView->load(QUrl::fromLocalFile(htmlPath));

    auto *devToolsView = new QWebEngineView;
    devToolsView->setAttribute(Qt::WA_DeleteOnClose);
    devToolsView->resize(1000, 700);
    devToolsView->show();

    m_webView->page()->setDevToolsPage(devToolsView->page());

    connect(&m_webSocket, &QWebSocket::textMessageReceived, this, &Robot3DView::onWebsocketMsgRecvd);
    connect(&m_webSocket, &QWebSocket::connected, this, &Robot3DView::onConnected);
    connect(&m_webSocket, &QWebSocket::disconnected, this, &Robot3DView::onDisconnected);
    connect(&m_webSocket, &QWebSocket::errorOccurred, this, &Robot3DView::onError);
}

void Robot3DView::setupMessageHandlers() {
    Robot3DView::messageHandlers["move_to_position"] = [this](QJsonObject& jsonObj) {
        this->moveToPosition(jsonObj);
    };

    Robot3DView::messageHandlers["home"] = [this](QJsonObject& jsonObj) {
        this->home(jsonObj);
    };
}

void Robot3DView::onWebsocketMsgRecvd(const QString &message) {
    qDebug() << "Websocket message received";
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    QJsonObject jsonObj = doc.object();

    QString messageType = jsonObj["type"].toString();
    QJsonObject jsonPayload = jsonObj["payload"].toObject();

    if (messageHandlers.contains(messageType)) {
        messageHandlers[messageType](jsonPayload);
    }
}

/* --- HANDLE CONNECTION --- */

void Robot3DView::onConnected() {
    Robot3DView::connectionFlag = true;
    qDebug() << "Connected to server";
    emit onWebsocketConnected();
}

void Robot3DView::onDisconnected() {
    Robot3DView::connectionFlag = false;
    qDebug() << "Disconnected from server.";
    emit onWebsocketDisconnected();
}

void Robot3DView::connectToServer(const QString host, quint16 port) {
    Robot3DView::m_webSocket.open(QUrl("ws://" + host + ":" + QString::number(port)));
}

void Robot3DView::disconnectFromServer() {
    Robot3DView::m_webSocket.close();
}

void Robot3DView::onError() {
    qDebug() << "Error occurred";
}

/* --- JOINT MOVES --- */

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

void Robot3DView::moveToPosition(QJsonObject& jsonObj) {

}

void Robot3DView::home(QJsonObject& jsonObj) {
    QJsonArray anglesArray = jsonObj["angles"].toArray();
    QString jsCommand = QString("moveToPosition(%1);").arg(QString::fromUtf8(QJsonDocument(anglesArray).toJson()));
    qDebug() << "Home command: " << jsCommand;

    m_webView->page()->runJavaScript(jsCommand);
}
