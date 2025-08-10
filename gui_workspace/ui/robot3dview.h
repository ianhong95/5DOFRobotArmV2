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

class Robot3DView : public QWidget
{
    Q_OBJECT
public:
    explicit Robot3DView(QWidget *parent = nullptr);

private:
    QWebEngineView *m_webView;

signals:
};

#endif // ROBOT3DVIEW_H
