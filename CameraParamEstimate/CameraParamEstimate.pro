#-------------------------------------------------
#
# Project created by QtCreator 2019-07-15T13:31:52
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets opengl

TARGET = CameraParamEstimate
TEMPLATE = app
DEFINES += QT_DEPRECATED_WARNINGS _USE_MATH_DEFINES
DESTDIR = $$PWD/../bin

CONFIG += c++14
INCLUDEPATH+=   ../include/ \


LIBS+=  -L$$PWD/../lib/ \
        -lfreeglut \
        -lopencv_world410 \
        -lopencv_ximgproc410

SOURCES += \
        main.cpp \
        mainwindow.cpp \
    imgRenderLabel.cpp \
    Widgets/RenderWidget.cpp \
    Scene/Scene.cpp \
    Scene/EventHandler/Manipulator.cpp \
    Scene/EventHandler/TrackBallManipulator.cpp \
    Scene/Camera.cpp \
    Scene/Scene.cpp \
    Widgets/RenderWidget.cpp \
    main.cpp \
    mainwindow.cpp \
    ModelScene/ModelScene.cpp \
    Algorithm/CameraSolver.cpp \
    Algorithm/FindBound.cpp \
    Algorithm/DistField.cpp \
    Algorithm/KDTreeDistField.cpp \
    Algorithm/MultiTrackKDTreeDistField.cpp

HEADERS += \
        mainwindow.h \
    imgRenderLabel.h \
    Widgets/RenderWidget.h \
    Scene/Scene.h \
    Math/HomoMatrix.h \
    Math/MathDefines.h \
    Math/Quaternion.h \
    Math/Vec.h \
    Scene/EventHandler/EventHandler.h \
    Scene/EventHandler/Manipulator.h \
    Scene/EventHandler/TrackBallManipulator.h \
    Scene/Camera.h \
    Scene/colorscheme.h \
    Scene/gcl_global.h \
    Scene/gclnamespace.h \
    Scene/Scene.h \
    Widgets/RenderWidget.h \
    mainwindow.h \
    ModelScene/ModelScene.h \
    Algorithm/CameraSolver.h \
    Algorithm/FindBound.h \
    Algorithm/DistField.h \
    Algorithm/kd-tree/kdtree.h \
    Algorithm/kd-tree/quicksort.h \
    Algorithm/KDTreeDistField.h \
    Algorithm/MultiTrackKDTreeDistField.h

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
