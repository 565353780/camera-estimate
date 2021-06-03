#include "RenderWidget.h"
#include "Scene/Scene.h"
#include "../ModelScene/ModelScene.h"
//#include "../Main/Model3D/Model3D.h"
//#include "../Main/Model3D/BoundingBox.h"
//#include "../Main/Operations/OperationManager.h"
#include <QtOpenGL>
#include <QMouseEvent>
#include <QProcess>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QPixmap>
#include <QBitmap>
#include <QIcon>
#include <QDebug>
#include <QPainterPath>
#include <QVector>
#include <QOpenGLContext>
#include <iostream>
namespace GCL {
RenderWidget::RenderWidget(Scene* scene, QWidget *parent)
    :scene_(dynamic_cast<ModelScene*>(scene)), QOpenGLWidget(),info_text_("Hello World!")
{
//    setObjectName("RenderWidget");
//    QGLFormat format;
//    format.setSampleBuffers(true);
//    format.setSamples(4);
//    this->setMouseTracking(true);
//    m_context_ = new QOpenGLContext(this);
//    m_context_->makeCurrent(this);
    makeCurrent();
//    initializeGL();

    QOpenGLContext* m_context=this->context();
    qDebug()<<"context valid:"<<m_context;
}

RenderWidget::~RenderWidget()
{

}

void RenderWidget::initializeGL()
{
    QOpenGLContext* m_context=this->context();
    qDebug()<<"context valid:"<<m_context->isValid();
    scene_->setContext(m_context);
//    scene_->init();

}

void RenderWidget::resizeGL(int w, int h)
{
    qDebug()<<"resize:"<<w<<h;
    scene_->resize(w,h);
    update();
}
//3d paint
void RenderWidget::paintGL()
{
    scene_->render();
}

void RenderWidget::mousePressEvent(QMouseEvent *e)
{
    this->setFocus();
    emit mousePressed();
    /// Screened Corrdinate System in OpenGL  start from left-bottom, is diffrent with  Qt's,
    /// Turn it into OpenGL screen corrdiantes;
    scene_->mousePress(e->x(),height() - e->y(),e->button(),e->modifiers());
    update();
}

void RenderWidget::mouseDoubleClickEvent(QMouseEvent *e)
{
    scene_->mouseDoubleClick(e->x(),height() - e->y(),e->button(),e->modifiers());
    update();
}



void RenderWidget::mouseReleaseEvent(QMouseEvent *e)
{
    scene_->mouseRelease(e->x(),height() - e->y(),e->button(),e->modifiers());
    update();
}

void RenderWidget::mouseMoveEvent(QMouseEvent *e)
{
    //qDebug()<<"mouse moveing";
    scene_->mouseMove(e->x(),height() - e->y());
//    show_text_->move(e->pos().x(), e->pos().y());
//    if(index_++ % 3 == 0)
//    {

//        index_ = 0;
//    }
    update();
}

void RenderWidget::wheelEvent(QWheelEvent *e)
{
    scene_->wheel((double)e->delta());
    update();
}

void RenderWidget::timerEvent(QTimerEvent *)
{
    scene_->timerRun();
    update();
}

void RenderWidget::keyReleaseEvent(QKeyEvent *e)
{
    scene_->keyUp(e->key(),e->modifiers());
    update();
}


void RenderWidget::slot_update()
{
    this->update();
}

void RenderWidget::slot_changeText(const QString &text)
{
    info_text_ = text;
}

void RenderWidget::loadTargetImage(QImage &image){
    scene_->loadTargetImage(image);
}

}
