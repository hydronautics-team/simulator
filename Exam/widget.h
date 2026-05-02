#ifndef WIDGET_H
#define WIDGET_H

#include "ui_widget.h"
#include "widget3d.h"

#include <QWidget>
#include <QImageReader>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsPixmapItem>
#include <QGraphicsTextItem>
#include <QScatter3DSeries>
#include <QScatterDataArray>
#include <QMediaPlayer>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QTableWidget>
#include <QHeaderView>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTimer>

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
    QNetworkAccessManager *httpManager = nullptr;
    QTimer *httpTimer = nullptr;
    QLabel *streamLabel = nullptr;
    QPixmap streamFrame;
};

#endif // WIDGET_H
