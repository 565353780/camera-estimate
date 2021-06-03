#include "ModelScene.h"
#include <QOpenGLTexture>
#include <QOpenGLContext>
#include <QDebug>
#include <GL/glut.h>

#include "../Scene/Camera.h"
#include "../Scene/EventHandler/TrackBallManipulator.h"
//#include <cmath>
namespace GCL {
    ModelScene::ModelScene()
    {
//        gl= new QOpenGLFunctions();
    }

    void ModelScene::slotSetFocalLength(double focal_length){

    }
    void ModelScene::slotRotateRight(){

        ((TrackBallManipulator*)getManipulator())->rotate(2,-1);
    }
    void ModelScene::slotRotateLeft(){
        ((TrackBallManipulator*)getManipulator())->rotate(-4,-1);
    }
    void ModelScene::slotRotateUp(){
        ((TrackBallManipulator*)getManipulator())->rotate(-1,0);

    }
    void ModelScene::slotRotateDown(){
        ((TrackBallManipulator*)getManipulator())->rotate(-1,-2);
    }
    void ModelScene::slotRotateClockWise()
    {
        ((TrackBallManipulator*)getManipulator())->rotateAlongDir(-0.02);
    }
    void ModelScene::slotRotateCounterClockWise(){
        ((TrackBallManipulator*)getManipulator())->rotateAlongDir(0.02);
    }
    void ModelScene::setContext(QOpenGLContext *context){
        m_context=context;
    }

    void ModelScene::setColor(GLfloat *color)
    {
        glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color);
        glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,color);
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,color);
        glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,color);
    }

    void ModelScene::setPointSet(QVector<QVector<Vec3>> &point_set)
    {
        point_set_=point_set;
        int num=point_set_.size();
        for(int i=0; i<num; i++)
        {
            for(int j = 0; j < point_set_[i].size(); ++j)
            {
                point_set_[i][j][2]=0;
            }
        }
    }
    void ModelScene::renderObjects(){

        glEnable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        //renderTracks();

        //renderCurling();
        if(point_set_.size()>0)
        {
            renderSampleSet(point_set_);
        }

        if(target_image_!=NULL){
            renderTargetImage();
        }

    }
    void ModelScene::renderTracks(){
        glDisable(GL_COLOR_MATERIAL);
        GLfloat half_blue[]={0.2,0.7,0.9,0.5};
        GLfloat half_white[]={1,1,1,0.5};
        setColor(half_blue);


        glBegin(GL_QUADS);
        glVertex3d(track_length_/2,track_radius_,0);
        glVertex3d(track_length_/2,-track_radius_,0);
        glVertex3d(-track_length_/2,-track_radius_,0);
        glVertex3d(-track_length_/2,track_radius_,0);
        glEnd();
        for(int i=0; i<track_num_; i++){
            renderSingleTrack(track_radius_+track_width_*i, track_length_);
        }
    }
    void ModelScene::renderSingleTrack(double radius, double straight_length)
    {
        glLineWidth(4);
        glEnable(GL_LIGHTING);
        glDisable(GL_COLOR_MATERIAL);
        GLfloat half_red[]={0.9,0.2,0.2,1};
        glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,half_red);
        glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,half_red);
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,half_red);
        glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,half_red);
        glBegin(GL_LINE_LOOP);
        //left circle
        for(int i=0; i<19; i++){
            glVertex3d(radius*cos(i/18.0*M_PI+M_PI/2.0)-straight_length/2.0,radius*sin(i/18.0*M_PI+M_PI/2.0),0.1);
        }
        glVertex3d(straight_length/2.0, -radius,0.1);
        for(int i=0; i<19; i++){
            glVertex3d(radius*cos(i/18.0*M_PI-M_PI/2.0)+straight_length/2.0,radius*sin(i/18.0*M_PI-M_PI/2.0),0.1);
        }
        glVertex3d(-straight_length/2.0, radius,0.1);
        glEnd();
    }

    void ModelScene::loadTargetImage(QImage &temp_image){
        QOpenGLTexture* to_delete=NULL;
        if(target_image_!=NULL){
            to_delete=target_image_;
        }
        target_image_=new QOpenGLTexture(temp_image);
        target_image_->setMinificationFilter(QOpenGLTexture::Linear);
        target_image_->setMagnificationFilter(QOpenGLTexture::Linear);
        target_image_->setWrapMode(QOpenGLTexture::Repeat);
        qDebug()<<"texture param set";
        glFlush();
        if(to_delete!=NULL)
            delete to_delete;
        qDebug()<<"finish delete";
    }

    void ModelScene::renderTargetImage(){
        GLfloat half_white[]={1,1,1,0.5};
        QOpenGLFunctions gl(m_context);
        gl.glActiveTexture(GL_TEXTURE0);
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        gluLookAt(0,0,1,0,0,0,0,1,0);

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(-get_width()/2.0,get_width()/2.0,-get_height()/2.0,get_height()/2.0,0,10);


        glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,half_white);
        glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,half_white);
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,half_white);
        glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,half_white);
//            glColor4d(1,1,1,0.2);

        glEnable(GL_TEXTURE_2D);

        target_image_->bind();

        glBegin(GL_QUADS);
        glVertex2d(-get_width()/2.0,-get_height()/2.0);glTexCoord2d(0,0);
        glVertex2d(-get_width()/2.0,get_height()/2.0);glTexCoord2d(1,0);
        glVertex2d(get_width()/2.0,get_height()/2.0);glTexCoord2d(1,1);
        glVertex2d(get_width()/2.0,-get_height()/2.0);glTexCoord2d(0,1);
        glEnd();
        target_image_->release();

        glBindTexture(GL_TEXTURE_2D,0);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
    }

    void ModelScene::renderSampleSet(QVector<QVector<Vec3>> &point_set)
    {
        glDisable(GL_COLOR_MATERIAL);
        GLfloat black[]={0,0,0,1};
        setColor(black);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        gluLookAt(0,0,1,0,0,0,0,-1,0);

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(-get_width()/2.0,get_width()/2.0,-get_height()/2.0,get_height()/2.0,0,100);

        for(int ii = 0; ii < point_set.size(); ++ii)
        {
            glBegin(GL_LINE_LOOP);
            int num=point_set[ii].size();
            for(int i=0; i<num; i++){
                glVertex3dv(point_set[ii][i].data());
            }
            glEnd();
        }

        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
    }

    void ModelScene::renderCircle(double x, double y, double r, int num)
    {
        glBegin(GL_LINE_LOOP);
        for(int i=0; i<num; i++){
            glVertex3d(x+r*cos(i*M_PI*2/num),y+r*sin(i*M_PI*2/num),0);
        }
        glEnd();
    }

    void ModelScene::renderQuad(double x, double y, double width, double height)
    {
        GLfloat white[]={0.9,0.9,0.9,0.9};
        GLfloat black[]={0,0,0,1};
        setColor(white);
        glBegin(GL_QUADS);
        glVertex3d(x-width/2.0,y-height/2.0,-0.1);
        glVertex3d(x-width/2.0,y+height/2.0,-0.1);
        glVertex3d(x+width/2.0,y+height/2.0,-0.1);
        glVertex3d(x+width/2.0,y-height/2.0,-0.1);
        glEnd();
        setColor(black);
        glBegin(GL_LINE_LOOP);
        glVertex3d(x-width/2.0,y-height/2.0,0);
        glVertex3d(x-width/2.0,y+height/2.0,0);
        glVertex3d(x+width/2.0,y+height/2.0,0);
        glVertex3d(x+width/2.0,y-height/2.0,0);
        glEnd();
    }

    void ModelScene::renderCurling()
    {
        double y=10.975+6.4, half_radius=0.61, num=36, min_radius=0.15, width=4.75, hog_height=44.5/2-10.975,total_height=44.5;
        renderQuad(0,0,width, total_height);
        renderQuad(0,10.975+hog_height/2,width, hog_height);
        renderQuad(0,-(10.975+hog_height/2),width, hog_height);
        int sign=-1;
        for(int i=0; i<2; i++){
            renderCircle(0,y*sign,min_radius*(1),num);
            for(int j=1; j<4; j++){
                renderCircle(0,y*sign,half_radius*(j),num);
            }
            sign*=-1;
        }
    }
    HomoMatrix4  ModelScene::getCameraRotation(){
        GLfloat rotation[16];
        glGetFloatv(GL_MODELVIEW_MATRIX,rotation);
        return HomoMatrix4(rotation);
    }
    Vec3 ModelScene::getCameraCenter(){
        GLfloat rotation[16];
        glGetFloatv(GL_MODELVIEW_MATRIX,rotation);
        HomoMatrix4 model_matrix(rotation);
        Vec3 camera_center;
        camera_center[0]=model_matrix.getValue(3,0);
        camera_center[1]=model_matrix.getValue(3,1);
        camera_center[2]=model_matrix.getValue(3,2);
        return camera_center;
    }
}

