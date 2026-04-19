#include "widget3d.h"
#include "ui_widget3d.h"
#include <QVBoxLayout>
#include <QtDataVisualization>
#include <QNetworkDatagram>
#include <cmath>

Widget3D::Widget3D(QWidget *parent)
    : QWidget(parent),
    graph(nullptr),
    pointROV(nullptr),
    pointDS(nullptr),
    directionLine(nullptr)
{
    setupUi(this);
    graph = new Q3DScatter();

    // Тема для кастомизации области графа
    Q3DTheme *theme = new Q3DTheme();
    theme->setBackgroundColor(QColor(0, 150, 255, 200));
    theme->setWindowColor(QColor(0, 102, 204, 0));
    theme->setLabelTextColor(QColor(0, 0, 0));
    theme->setLabelBackgroundColor(QColor(0, 200, 255));
    graph->setActiveTheme(theme);

    // Настройка освещения
    graph->setShadowQuality(QAbstract3DGraph::ShadowQualityMedium);

    QWidget *container = QWidget::createWindowContainer(graph);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(container);

    // Маркер (цилиндр) - синяя точка
    pointDS = new QScatter3DSeries();
    pointDS->setBaseColor(QColor(0, 100, 255));
    pointDS->setItemSize(0.25f);
    pointDS->setMesh(QAbstract3DSeries::MeshSphere);

    // Аппарат - красный куб
    pointROV = new QScatter3DSeries();
    pointROV->setBaseColor(Qt::red);
    pointROV->setItemSize(0.3f);
    pointROV->setMesh(QAbstract3DSeries::MeshCube);

    // Линия направления (вектор) - зелёные точки
    directionLine = new QScatter3DSeries();
    directionLine->setBaseColor(Qt::green);
    directionLine->setItemSize(0.1f);
    directionLine->setMesh(QAbstract3DSeries::MeshSphere);

    graph->addSeries(pointDS);
    graph->addSeries(pointROV);
    graph->addSeries(directionLine);

    // Начальные данные для маркера
    QScatterDataArray markerData;
    markerData << QVector3D(0.0f, 0.0f, 0.0f);
    pointDS->dataProxy()->addItems(markerData);

    // Начальные данные для аппарата
    QScatterDataArray rovData;
    rovData << QVector3D(2.0f, 2.0f, 2.0f);
    pointROV->dataProxy()->addItems(rovData);

    // Начальные данные для линии направления
    QScatterDataArray lineData;
    lineData << QVector3D(2.0f, 2.0f, 2.0f)
             << QVector3D(2.5f, 2.0f, 2.0f);
    directionLine->dataProxy()->addItems(lineData);

    // Настройка осей
    graph->axisX()->setRange(-2.0f, 2.0f);
    graph->axisY()->setRange(0.0f, 2.0f);
    graph->axisZ()->setRange(-2.0f, 2.0f);

    // Подписи осей
    graph->axisX()->setTitle("X");
    graph->axisY()->setTitle("Y");
    graph->axisZ()->setTitle("Z");
}

void Widget3D::updateROVPosition(float x, float y, float z, float yaw)
{
    // Обновляем позицию аппарата
    QScatterDataArray rovData;
    rovData << QVector3D(x, y, z);
    pointROV->dataProxy()->resetArray(&rovData);

    // Вычисляем точку направления (впереди аппарата на 0.05 метра по курсу)
    float dirX = x + 0.05f * cos(yaw * M_PI / 180.0);
    float dirZ = z + 0.05f * sin(yaw * M_PI / 180.0);

    // Обновляем линию направления (вектор)
    QScatterDataArray lineData;
    lineData << QVector3D(x, y, z)
             << QVector3D(dirX, y, dirZ);
    directionLine->dataProxy()->resetArray(&lineData);
}

void Widget3D::updateMarkerPosition(float x, float y, float z)
{
    QScatterDataArray markerData;
    markerData << QVector3D(x, y, z);
    pointDS->dataProxy()->resetArray(&markerData);
}

Widget3D::~Widget3D()
{
    if (graph) {
        delete graph;
        graph = nullptr;
    }
}
