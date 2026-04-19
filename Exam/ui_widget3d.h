/********************************************************************************
** Form generated from reading UI file 'widget3d.ui'
**
** Created by: Qt User Interface Compiler version 6.4.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WIDGET3D_H
#define UI_WIDGET3D_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Widget3D
{
public:

    void setupUi(QWidget *Widget3D)
    {
        if (Widget3D->objectName().isEmpty())
            Widget3D->setObjectName("Widget3D");
        Widget3D->resize(400, 300);

        retranslateUi(Widget3D);

        QMetaObject::connectSlotsByName(Widget3D);
    } // setupUi

    void retranslateUi(QWidget *Widget3D)
    {
        Widget3D->setWindowTitle(QCoreApplication::translate("Widget3D", "Form", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Widget3D: public Ui_Widget3D {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WIDGET3D_H
