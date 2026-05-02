QT += core gui widgets multimedia multimediawidgets datavisualization network
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
CONFIG += c++17
SOURCES += main.cpp widget.cpp widget3d.cpp
HEADERS += widget.h widget3d.h
FORMS += widget.ui widget3d.ui
RESOURCES += BG.qrc
