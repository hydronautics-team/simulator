#include "widget.h"
#include "widget3d.h"
#include <QDebug>
#include <QDir>
#include <QTimer>
#include <QNetworkDatagram>
#include <QTableWidget>
#include <QHeaderView>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
{
    setupUi(this);

    // Создаем задний фон
    QImageReader::setAllocationLimit(256);
    QPixmap pix("land.png");
    pix = pix.scaled(this->size(), Qt::KeepAspectRatioByExpanding);
    QPalette palette;
    palette.setBrush(QPalette::Window, pix);
    this->setPalette(palette);

    QHeaderView* header = tableWidget->horizontalHeader();

    // Устанавливаем режим растягивания для всех столбцов
    header->setSectionResizeMode(QHeaderView::Stretch);

    // Отключаем растягивание последней секции
    header->setStretchLastSection(false);

    QHeaderView* vHeader = tableWidget->verticalHeader();
    vHeader->setSectionResizeMode(QHeaderView::Stretch);

    // Формируем отображение крена и дифферента и добавляем картинки аппарата
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

    //ROLL
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

    // Подключение виджета 3Д положения ПА относительно док-станции
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
    rovSeries->dataProxy()->resetArray(&data);

    // UDP инициализация
    initUDP();
}

Widget::~Widget()
{
    if (udpSocket) {
        udpSocket->close();
        delete udpSocket;
        udpSocket = nullptr;
    }
}

void Widget::initUDP()
{
    udpSocket = new QUdpSocket(this);
    if (udpSocket->bind(QHostAddress::Any, 8888)) {
        qDebug() << "UDP сервер запущен на порту 8888";
        connect(udpSocket, &QUdpSocket::readyRead, this, &Widget::processUDPData);
    } else {
        qDebug() << "Ошибка: не удалось запустить UDP сервер на порту 8888";
    }
}

void Widget::processUDPData()
{
    while (udpSocket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = udpSocket->receiveDatagram();
        QString message = QString::fromUtf8(datagram.data());

        QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
        QJsonObject obj = doc.object();

        // Получаем данные
        double x = obj["x"].toDouble();
        double y = obj["y"].toDouble();
        double z = obj["z"].toDouble();
        double roll = obj["roll"].toDouble();
        double pitch = obj["pitch"].toDouble();
        double yaw = obj["yaw"].toDouble();

        QString type = obj["type"].toString();
        int markerId = obj["Id"].toInt();

        if (type == "ArUco")
        {
            label->setText(QString("Маркер обнаружен! ID: %1").arg(markerId));
        }
        else
        {
            label->setText("Маркер не обнаружен");
        }
        tableWidget->setItem(0, 0, new QTableWidgetItem(QString::number(x, 'f', 2)));
        tableWidget->setItem(0, 1, new QTableWidgetItem(QString::number(y, 'f', 2)));
        tableWidget->setItem(0, 2, new QTableWidgetItem(QString::number(z, 'f', 2)));
        tableWidget->setItem(0, 3, new QTableWidgetItem(QString::number(yaw, 'f', 1)));

        // Обновляем интерфейс (углы)
        txtCurrentPitch->setPlainText(QString::number(pitch, 'f', 1) + "°");
        txtCurrentRoll->setPlainText(QString::number(roll, 'f', 1) + "°");
        pitchRov->setRotation(-pitch);
        rollRov->setRotation(roll);

        // Обновляем 3D-позицию
        data.clear();
        data << QVector3D(x, z, y);
        if (rovSeries) {
            rovSeries->dataProxy()->resetArray(&data);
        }

        // Обновляем 3D-виджет с курсом
        if (threeDWidget) {
            threeDWidget->updateROVPosition(x, z, y, yaw);
            threeDWidget->updateMarkerPosition(0.0f, 0.0f, 0.0f);
        }
    }
}

int Widget::fileSize(QString folder)
{
    QDir dir(folder);
    QStringList items = dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries);
    return items.count();
}
