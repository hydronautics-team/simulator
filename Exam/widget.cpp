#include "widget.h"
#include "widget3d.h"
#include <QDebug>
#include <QDir>
#include <QTimer>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
{
    setupUi(this);

    QImageReader::setAllocationLimit(256);
    QPixmap pix("land.png");
    pix = pix.scaled(this->size(), Qt::KeepAspectRatioByExpanding);
    QPalette palette;
    palette.setBrush(QPalette::Window, pix);
    this->setPalette(palette);

    // PITCH
    scene1 = new QGraphicsScene(pitchView);
    pitchView->setScene(scene1);
    pitchView->setStyleSheet("background: transparent; border: none;");
    pitchView->setRenderHint(QPainter::Antialiasing);
    QGraphicsEllipseItem *circle = new QGraphicsEllipseItem();
    circle->setRect(0, 0, 500, 500);
    circle->setPos(2.5, 2.5);
    QPen circlePen(Qt::black);
    circle->setBrush(QBrush(QColor(0, 150, 255, 150)));
    circlePen.setWidth(2);
    circle->setPen(circlePen);
    scene1->addItem(circle);
    pitchDial = scene1->addPixmap(QPixmap("anglesPitch.png"));
    pitchRov = scene1->addPixmap(QPixmap("pitchRov.png"));
    pitchDial->setTransform(QTransform::fromScale(0.5, 0.5));
    pitchRov->setTransform(QTransform::fromScale(0.3, 0.3));
    pitchRov->setPos(85, 200);
    txtCurrentPitch = scene1->addText(QString::number(a, 'f', 1) + "°", QFont("Calibri", 24));
    txtCurrentPitch->setDefaultTextColor(QColor(0, 0, 0));
    QTransform t;
    t.translate(pitchDial->pixmap().width()/4-20, pitchDial->pixmap().height()/4-22);
    txtCurrentPitch->setTransform(t);
    pitchRov->setTransformOriginPoint(pitchRov->pixmap().width()/2, pitchRov->pixmap().height()/2);

    // ROLL
    scene2 = new QGraphicsScene(rollView);
    rollView->setScene(scene2);
    rollView->setStyleSheet("background: transparent; border: none;");
    rollView->setRenderHint(QPainter::Antialiasing);
    QGraphicsEllipseItem *circle1 = new QGraphicsEllipseItem();
    circle1->setRect(0, 0, 500, 500);
    circle1->setPos(2.5, 2.5);
    QPen circlePen1(Qt::black);
    circle1->setBrush(QBrush(QColor(0, 150, 255, 150)));
    circlePen1.setWidth(2);
    circle1->setPen(circlePen1);
    scene2->addItem(circle1);
    rollDial = scene2->addPixmap(QPixmap("anglesRoll.png"));
    rollRov = scene2->addPixmap(QPixmap("rollRov.png"));
    rollDial->setTransform(QTransform::fromScale(0.5,0.5));
    rollRov->setTransform(QTransform::fromScale(0.4, 0.4));
    rollRov->setPos(69, 155);
    txtCurrentRoll = scene2->addText(QString::number(a, 'f', 1) + "°", QFont("Times New Roman", 24));
    txtCurrentRoll->setDefaultTextColor(Qt::black);
    QTransform t1;
    t1.translate(rollDial->pixmap().width()/4-20, rollDial->pixmap().height()/4-22);
    txtCurrentRoll->setTransform(t1);
    rollRov->setTransformOriginPoint(rollRov->pixmap().width()/2, rollRov->pixmap().height()/2);

    // 3D
    QWidget *parentContainer = pos3D->parentWidget();
    int index = gridLayout->indexOf(pos3D);
    int row, column, rowSpan, columnSpan;
    gridLayout->getItemPosition(index, &row, &column, &rowSpan, &columnSpan);
    pos3D->setStyleSheet("background-color: rgb(0, 102, 204); border-style: outset;"
                         " border-width: 2px; border-radius: 20px;"
                         " border-color: rgb(0, 0, 180);");
    Widget3D *threeDWidget = new Widget3D(parentContainer);
    gridLayout->addWidget(threeDWidget, row, column, rowSpan, columnSpan);
    rovSeries = threeDWidget->pointROV;
    dsSeries = threeDWidget->pointDS;
    data << QVector3D(2.0f, 2.0f, 2.0f);
    QHeaderView* header = tableWidget->horizontalHeader();
    header->setSectionResizeMode(QHeaderView::Stretch);
    header->setStretchLastSection(false);
    QHeaderView* vHeader = tableWidget->verticalHeader();
    vHeader->setSectionResizeMode(QHeaderView::Stretch);
    rovSeries->dataProxy()->resetArray(&data);

    Video->show();

    Video->setAlignment(Qt::AlignCenter);
    Video->setScaledContents(false);
    Video->setMinimumSize(640, 480);
    Video->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    streamLabel = new QLabel(Video);
    streamLabel->setGeometry(Video->rect());
    streamLabel->setScaledContents(true);
    streamLabel->hide();

    // ====== HTTP ДАННЫЕ + ВИДЕО ИЗ СИМУЛЯТОРА ======
    httpManager = new QNetworkAccessManager(this);
    httpTimer = new QTimer(this);
    connect(httpTimer, &QTimer::timeout, this, [this]() {
        // Данные
        QNetworkReply *d = httpManager->get(QNetworkRequest(QUrl("http://localhost:9090/data")));
        connect(d, &QNetworkReply::finished, this, [this, d]() {
            if (d->error() == QNetworkReply::NoError) {
                QJsonObject obj = QJsonDocument::fromJson(d->readAll()).object();
                QJsonObject m = obj["marker"].toObject();
                double x = m["x"].toDouble(), y = m["y"].toDouble(), z = m["z"].toDouble();
                double roll = m["roll"].toDouble(), pitch = m["pitch"].toDouble(), yaw = m["yaw"].toDouble();
                int markerId = m["id"].toInt();

                // Лейбл детекции
                label->setText(markerId >= 0 ? QString("Маркер ID: %1").arg(markerId) : "Маркер не найден");

                auto setCell = [this](int row, int col, QString val) {
                    QTableWidgetItem *item = new QTableWidgetItem(val);
                    item->setTextAlignment(Qt::AlignCenter);
                    item->setBackground(QColor(98, 160, 234, 255));
                    item->setForeground(QColor(255, 255, 255));
                    QFont font;
                    font.setPointSize(16);
                    item->setFont(font);
                    tableWidget->setItem(row, col, item);
                };

                setCell(0, 0, QString::number(x, 'f', 2));
                setCell(0, 1, QString::number(y, 'f', 2));
                setCell(0, 2, QString::number(z, 'f', 2));
                setCell(0, 3, QString::number(yaw, 'f', 1));

                txtCurrentPitch->setPlainText(QString::number(pitch,'f',1)+"°");
                txtCurrentRoll->setPlainText(QString::number(roll,'f',1)+"°");
                pitchRov->setRotation(-pitch);
                rollRov->setRotation(roll);
                data.clear();
                data << QVector3D(x, z, y);
                if (rovSeries) rovSeries->dataProxy()->resetArray(&data);
            }
            d->deleteLater();
        });
        // Видео
        QNetworkReply *v = httpManager->get(QNetworkRequest(QUrl("http://localhost:9090/video")));
        connect(v, &QNetworkReply::finished, this, [this, v]() {
            if (v->error() == QNetworkReply::NoError) {
                QByteArray jpeg = v->readAll();
                if (jpeg.size() > 0) {
                    streamFrame.loadFromData(jpeg);
                    if (!streamFrame.isNull()) {
                        Video->setPixmap(streamFrame.scaled(Video->size(), Qt::KeepAspectRatio));
                    }
                }
            }
            v->deleteLater();
        });
    });
    httpTimer->start(33);
}

Widget::~Widget() {}

int Widget::fileSize(QString folder) {
    QDir dir(folder);
    return dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries).count();
}
