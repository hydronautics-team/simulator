#ifndef WIDGET3D_H
#define WIDGET3D_H

#include <QWidget>
#include <QVBoxLayout>
#include <Q3DScatter>
#include <QScatter3DSeries>
#include <QScatterDataArray>

#include "ui_widget3d.h"

class Widget3D : public QWidget, private Ui::Widget3D
{
    Q_OBJECT
public:
    explicit Widget3D(QWidget *parent = nullptr);
    ~Widget3D();
    void updateROVPosition(float x, float y, float z, float yaw);
    void updateMarkerPosition(float x, float y, float z);
    Q3DScatter *graph;
    QScatter3DSeries *pointROV;
    QScatter3DSeries *pointDS;
    QScatter3DSeries *directionLine;
};
#endif
