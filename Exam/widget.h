#ifndef WIDGET_H
#define WIDGET_H

#include "ui_widget.h"
#include "widget3d.h"

#include <QWidget>
#include <QImageReader>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsPixmapItem>
#include <QGraphicsTextItem>
#include <QtDataVisualization>
#include <QUdpSocket>
#include <QJsonDocument>
#include <QJsonObject>

class Widget : public QWidget, private Ui::Widget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();
    QString folder = "Calib/";
    QStringList items;
    QScatterDataArray data;
    int fileSize(QString folder);
    int k;
    int flagMarker = 0;
    int endVideo = 1;
    int videoSize = 0;
    int videoTime = 0;
    int markerPressCount = 0;

    double timeCounter = 0;
    double a = 0;

    bool calibDone = false;


private:
    QMediaPlayer *player;
    QGraphicsScene *scene1;
    QGraphicsPixmapItem *pitchRov;
    QGraphicsPixmapItem *pitchDial;
    QGraphicsTextItem *txtCurrentPitch;
    QGraphicsScene *scene2;
    QGraphicsPixmapItem *rollRov;
    QGraphicsPixmapItem *rollDial;
    QGraphicsTextItem *txtCurrentRoll;
    int n = 1;
    QScatter3DSeries *rovSeries = nullptr;
    QScatter3DSeries *dsSeries = nullptr;
    QUdpSocket *udpSocket = nullptr;
    void initUDP();
    void processUDPData();

    Widget3D *threeDWidget = nullptr;
};
#endif // WIDGET_H
