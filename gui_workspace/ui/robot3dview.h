#ifndef ROBOT3DVIEW_H
#define ROBOT3DVIEW_H

#include <QObject>
#include <QWidget>
#include <QtWebEngineWidgets/QWebEngineView>
#include <QVBoxLayout>
#include <QDir>
#include <QString>
#include <QCoreApplication>
#include <QDebug>
#include <QWebEngineSettings>
#include <QWebEngineProfile>
#include <QtWebSockets/QWebSocket>

#include <vector>

class Robot3DView : public QWidget
{
    Q_OBJECT
public:
    explicit Robot3DView(QWidget *parent = nullptr);

    QWebSocket m_webSocket;

    void moveJ1(float angle);
    void moveJ2(float angle);
    void moveJ3(float angle);
    void moveJ4(float angle);
    void rotateGripper(float angle);
    void openGripper();
    void closeGripper();
    void moveToPosition(QJsonObject& jsonObj);
    void home(QJsonObject& jsonObj);

    bool connectionFlag = false;
    void connectToServer(const QString host, quint16 port);
    void disconnectFromServer();

private:
    QMap<QString, std::function<void(QJsonObject&)>> messageHandlers;
    void setupMessageHandlers();
    QWebEngineView *m_webView;


private slots:
    void onWebsocketMsgRecvd(const QString& message);
    void onConnected();
    void onError();
    void onDisconnected();

signals:
    void onWebsocketConnected();
    void onWebsocketDisconnected();
};

#endif // ROBOT3DVIEW_H
