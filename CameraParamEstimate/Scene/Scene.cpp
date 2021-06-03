
#include "Scene.h"
#include "Camera.h"
#include "EventHandler/TrackBallManipulator.h"
#ifdef WIN32
#include <Windows.h>
#endif
#include <QOpenGLFunctions>
#include "qcolor.h"

namespace GCL {

Scene::Scene():camera_(new Camera()),manipulator_(new TrackBallManipulator(camera_.get())),
    has_light_(true),interval_(300),back_color_mode_(0)
{

}

Scene::~Scene()
{

}

void Scene::init()
{
    /// 背景色
    glClearColor(.9f,.9f,.9f,1.f);

    /// 打开深度测试： 如果关闭的话，物体就没有遮挡了;
    glEnable(GL_DEPTH_TEST);

    /// 关闭剔除面功能: 如果打开， 一些位于背面（基于当前相机视角）的面就不会渲染了；
    glDisable(GL_CULL_FACE);

    /// 打开颜色融合与设置融合函数：这样颜色的Alpha通道才会有作用
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);

    this->setLighting(has_light_);
}

void Scene::resize(int w, int h)
{
    width_ = w;
    height_ = h;
    ///* 相机视角需要改变
    camera_->set_viewport(w,h);

    if(manipulator_->isEnable())
    {
        manipulator_->resize(w,h);
    }
}

void Scene::render()
{
    glShadeModel(GL_SMOOTH);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    if(back_color_mode_ == 1)
    {
       glBegin(GL_QUADS);
       QColor color(0,200,200);
       glColor3ub(color.red(),color.green(),color.blue());

       glVertex2d(-width_,-height_);
       glVertex2d(width_,-height_);

       QColor qcolor(51,51,51);
       glColor3ub(qcolor.red(),qcolor.green(),qcolor.blue());

       glVertex2d(width_,height_);
       glVertex2d(-width_,height_);
       glEnd();
       glEnable(GL_DEPTH_TEST);
       glEnable(GL_LIGHTING);
       back_color_mode_ = 0;
    }
    else
    {
        glBegin(GL_QUADS);
        QColor color(241,255,243);
        glColor3ub(color.red(),color.green(),color.blue());

        glVertex2d(-width_,-height_);
        glVertex2d(width_,-height_);

        QColor qcolor(241,255,243);
        glColor3ub(qcolor.red(),qcolor.green(),qcolor.blue());

        glVertex2d(width_,height_);
        glVertex2d(-width_,height_);
        glEnd();
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHTING);
    }
    /// 光照
    if(has_light_)
    {
        this->renderLights();
    }
    /// 相机
    camera_->enter();

    /// 具体的渲染
    this->renderObjects();
    camera_->leave();
}

void Scene::timerRun()
{

}

Camera *Scene::getCamera()
{
    return camera_.get();
}

void Scene::enableManipulator(bool t)
{
    manipulator_->setEnable(t);
}

void Scene::setManipulator(const std::shared_ptr<Manipulator> &manipulator)
{
    manipulator_ = manipulator;
}

void Scene::setLighting(bool t)
{
    has_light_ = t;
    if(has_light_)
    {
        ///* 打开光照
        glEnable(GL_LIGHTING);
        ///* 打开颜色材质
        glEnable(GL_COLOR_MATERIAL);
        ///* 打开双面光照
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);

        GLfloat light_diffuse0[]  = { 205/255.0, 205/255.0 , 205/255.0, 0.2 };  // 主光白光
        GLfloat light_specular0[] = { 0.0f, 0.0f, 0.0f, 0.0f };
        GLfloat light_ambient0[]  = { 104/255.0, 104/255.0, 104/155.0, 0.9 };
        GLfloat light_emission0[] = { 0.0f, 0.0f, 0.0f, 0.0f };

        GLfloat light_diffuse1[] ={187/255.0,187/255.0,187/255.0,0.3};
        GLfloat light_specular1[] = { 0.0f, 0.0f, 0.0f, 0.0f };
        GLfloat light_ambient1[]  = { 0.0, 0.0, 0.0, 0 };
        GLfloat light_emission1[] = { 0.0f, 0.0f, 0.0f, 0.0f };

//        GLfloat light_position0[] = {7,  -5, -8, 0.0f };
//        GLfloat light_position1[] = {5,-6,-7,0.0};
//        glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
//        glLightfv(GL_LIGHT0, GL_POSITION, light_position1);
        // 光源位置在 camera::update_viewport_and_matrix里面定，使得能相对世界固定

        glEnable(GL_LIGHT0);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse0);
        glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular0);
        glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient0);
        glLightfv(GL_LIGHT0, GL_EMISSION, light_emission0);

        glEnable(GL_LIGHT1);
        glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 55);
        glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse1);
        glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular1);
        glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient1);
        glLightfv(GL_LIGHT1, GL_EMISSION, light_emission1);

    }
    else
    {
        ///* 关闭光照
        glDisable(GL_LIGHTING);
        ///* 关闭颜色材质
        glDisable(GL_COLOR_MATERIAL);
    }
}

void Scene::renderLights()
{




    glEnable(GL_LIGHT_MODEL_TWO_SIDE);
}
int Scene::interval() const
{
    return interval_;
}

void Scene::setInterval(int interval)
{
    interval_ = interval;
}

//void Scene::OnUpdate()
//{
//    emit signal_update();
//}




void Scene::mouseMove(int posX, int posY)
{
    if(manipulator_->isEnable())
    {
        manipulator_->mouseMove(posX,posY);
    }
}

void Scene::mousePress(int posX, int posY, unsigned int button, unsigned int modifier)
{

    if(manipulator_->isEnable())
    {
        manipulator_->mousePress(posX,posY,button,modifier);
    }
}

void Scene::mouseRelease(int posX, int posY, unsigned int button, unsigned int modifier)
{
    if(manipulator_->isEnable())
    {
        manipulator_->mouseRelease(posX,posY,button,modifier);
    }
}

void Scene::mouseDoubleClick(int posX, int posY, unsigned int button, unsigned int modifier)
{
    if(manipulator_->isEnable())
    {
        manipulator_->mouseDoubleClick(posX,posY,button,modifier);
    }
}
void Scene::keyUp(const unsigned int &key, unsigned int modifier)
{
//    camera_->get_direction().print();
    if(manipulator_->isEnable())
    {
        manipulator_->keyUp(key,modifier);
    }
}

void Scene::keyDown(const unsigned int &key, unsigned int modifier)
{
    if(manipulator_->isEnable())
    {
        manipulator_->keyDown(key,modifier);
    }
}

void Scene::wheel(const float &delta)
{
    if(manipulator_->isEnable())
    {
        manipulator_->wheel(delta);
    }
}







}
