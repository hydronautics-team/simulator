#include "widget3d.h"
#include "ui_widget3d.h"
#include <QWidget>
#include <QVBoxLayout>
#include <QtDataVisualization>
#include <QNetworkDatagram>
#include <cmath>

Widget3D::Widget3D(QWidget *parent)
    : QWidget(parent),
    graph(nullptr),
    pointROV(nullptr),
    pointDS(nullptr),
    directionSphere(nullptr)
{
    setupUi(this);
    graph = new Q3DScatter();

    Q3DTheme *theme = new Q3DTheme();
    theme->setBackgroundColor(QColor(0, 150, 255, 200));
    theme->setWindowColor(QColor(0, 102, 204, 0));
    theme->setLabelTextColor(QColor(0, 0, 0));
    theme->setLabelBackgroundColor(QColor(0, 200, 255));
    graph->setActiveTheme(theme);
    graph->setShadowQuality(QAbstract3DGraph::ShadowQualityMedium);

    QWidget *container = QWidget::createWindowContainer(graph);
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(container);

    // Маркер
    pointDS = new QScatter3DSeries();
    pointDS->setBaseColor(QColor(0, 100, 255));
    pointDS->setItemSize(0.25f);
    pointDS->setMesh(QAbstract3DSeries::MeshSphere);

    // Куб
    pointROV = new QScatter3DSeries();
    pointROV->setBaseColor(Qt::red);
    pointROV->setItemSize(0.3f);
    pointROV->setMesh(QAbstract3DSeries::MeshCube);

    // Сфера - создаём сразу
    directionSphere = new QScatter3DSeries();
    directionSphere->setBaseColor(Qt::green);  // Ярко-зелёный, точно видно
    directionSphere->setItemSize(0.3f);  // Такой же размер как куб
    directionSphere->setMesh(QAbstract3DSeries::MeshSphere);

    // Добавляем ВСЕ серии
    graph->addSeries(pointDS);
    graph->addSeries(pointROV);
    graph->addSeries(directionSphere);

    // Начальные данные
    QScatterDataArray *rovData = new QScatterDataArray();
    rovData->append(QVector3D(0.0f, 0.0f, 0.0f));
    pointROV->dataProxy()->resetArray(rovData);

    QScatterDataArray *sphereData = new QScatterDataArray();
    sphereData->append(QVector3D(0.2f, 0.0f, 0.0f));  // Чуть правее куба
    directionSphere->dataProxy()->resetArray(sphereData);

    QScatterDataArray *markerData = new QScatterDataArray();
    markerData->append(QVector3D(0.0f, 0.0f, 0.0f));
    pointDS->dataProxy()->resetArray(markerData);

    // Оси
    graph->axisX()->setRange(-2.0f, 2.0f);
    graph->axisY()->setRange(0.0f, 2.0f);
    graph->axisZ()->setRange(-2.0f, 2.0f);
    graph->axisX()->setTitle("X");
    graph->axisY()->setTitle("Y");
    graph->axisZ()->setTitle("Z");
}

void Widget3D::updateROVPosition(float x, float y, float z, float yaw)
{
    // Обновляем куб
    QScatterDataArray *rovData = new QScatterDataArray();
    rovData->append(QVector3D(x, y, z));
    pointROV->dataProxy()->resetArray(rovData);

    // Вычисляем позицию сферы
    float dirX = x + 0.1f * cos(yaw * M_PI / 180.0);  // Увеличил до 0.5 для заметности
    float dirZ = z + 0.1f * sin(yaw * M_PI / 180.0);

    // Обновляем сферу
    QScatterDataArray *sphereData = new QScatterDataArray();
    sphereData->append(QVector3D(dirX, y, dirZ));
    directionSphere->dataProxy()->resetArray(sphereData);
}

void Widget3D::updateMarkerPosition(float x, float y, float z)
{
    QScatterDataArray *markerData = new QScatterDataArray();
    markerData->append(QVector3D(x, y, z));
    pointDS->dataProxy()->resetArray(markerData);
}

Widget3D::~Widget3D()
{
    if (graph) {
        delete graph;
        graph = nullptr;
    }
}
