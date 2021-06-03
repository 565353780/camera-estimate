#include "Camera.h"
#ifdef WIN32
#include <Windows.h>
#endif
#include <QtOpenGL>
#include <GL/glut.h>
#include <QDebug>
namespace GCL{

Camera::Camera()
    :pos_(0,0,200),center_(0,0,0),up_(0,1,0)
{
    default_pos_ = Vec3(0, -200,200);
    isOrth_=false;
    slotSetFocalLength(2000);
    focal_angle_=45.0;
}


void Camera::enter()
{
    set_viewport(width_,height_);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glPushMatrix();
    gluLookAt(pos_[0],pos_[1],pos_[2], center_[0],center_[1],center_[2],up_[0],up_[1],up_[2]);
    glMultMatrixd(rotate_matrix_.data());
    glTranslated(offset_[0],offset_[1],offset_[2]);
    this->update_viewport_and_matrix();
    //    get_direction().print();
}

void Camera::leave()
{
    glPopMatrix();
}


Vec3 Camera::projectToScreen(const Vec3 &point) const
{
    Vec3 ans;
    gluProject(point[0],point[1],point[2],model_matrix_.data(), project_matrix_.data(), viewport_.data(),&ans[0],&ans[1],&ans[2]);
    return ans;
}

Vec3 Camera::unprojectToGround(const Vec3 &point) const
{
    Vec3 ans;
    gluUnProject(point[0],point[1],point[2],model_matrix_.data(), project_matrix_.data(), viewport_.data(),&ans[0],&ans[1],&ans[2]);
//    qDebug()<<"first result:"<<ans[0]<<ans[1]<<ans[2];
    Vec3 project_point_;
    if(fabs(ans[2])>ZERO_TOLERANCE){
        Vec3 actual_position=get_actual_position();
//        qDebug()<<"actual camera position:"<<actual_position[0]<<actual_position[1]<<actual_position[2];

        Vec3 dir=ans-actual_position;
        ans= actual_position-actual_position[2]/(dir[2])*dir;

        gluProject(ans[0],ans[1],ans[2],model_matrix_.data(),project_matrix_.data(),viewport_.data(),&project_point_[0],&project_point_[1],&project_point_[2]);
//        qDebug()<<"projected to:"<< project_point_[0]<<project_point_[1]<<project_point_[2];
    }
    return ans;
}

Vec3 Camera::unprojectToWorld(const Vec3 &point) const
{
    Vec3 ans;
    gluUnProject(point[0],point[1],point[2],model_matrix_.data(), project_matrix_.data(), viewport_.data(),&ans[0],&ans[1],&ans[2]);
    return ans;
}
void Camera::set_viewport(int w, int h)
{

    glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    Scalar dist = (pos_ - center_).length();
    Scalar distScale=dist/1370;
    if(!isOrth_)
    {
//        gluPerspective(focal_angle_,double(w)/double(h),0.01,100000);
        glFrustum(-w/2.0*0.1/focal_length_,w/2.0*0.1/focal_length_,-h/2.0*0.1/focal_length_, h/2.0*0.1/focal_length_, 0.1, 100000);
    }
    else
    {
        int length=w<h?w/8+dist/3:h/8+dist/3;
        int d= w<h?w:h;
        int d1=w<h?h-w:w-h;
        //glViewport(d1/2,0,d,d);
        glOrtho(-w*distScale,w*distScale,-h*distScale,h*distScale,-1000,1000);
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    width_ = w;
    height_ = h;
}
void Camera::set_orth(bool t)
{
    if(t)
    {
        isOrth_=true;
    }
    else
    {
        isOrth_=false;
    }

}

double truncate(double xmin, double xmax, double x){
    if(x>xmax)
        return xmax;
    if(x<xmin)
        return xmin;
    return x;
}

void Camera::setRotationFromQuat(Scalar x, Scalar y, Scalar z, Scalar w){
    Quat rotate_quat(truncate(-1,1,x),truncate(-1,1,y),truncate(-1,1,z),truncate(-1,1,w));
    rotate_matrix_=rotate_quat.convertToMatrix();
    emit signalCameraUpdated();
}

void Camera::setRotateFromQuatVec(Vec4 xyzw){
    xyzw.Normalize();
    setRotationFromQuat(xyzw[0],xyzw[1],xyzw[2],xyzw[3]);
}

Vec4 Camera::getQuatVecFromRotate() const
{
    Quat rotate_quat;
    rotate_quat.convertFromMatrix(rotation_matrix());
    return Vec4(rotate_quat.x(),rotate_quat.y(),rotate_quat.z(),rotate_quat.w());
}

Vec3 Camera::getProjectToScreenOnUniformRange(Vec3 &point)
{
    Vec<4,double> in;
    in[0] =point[0];
    in[1] =point[1];
    in[2] = point[2];
    in[3] = 1.0;
    in=(model_matrix_*Vec<4,double>(point[0],point[1],point[2],1.0));
    in=project_matrix_*in;
    in[0] /= in[3];
    in[1] /= in[3];
    in[2] /= in[3];
    return Vec3(in[0],in[1],in[2]);
}
void Camera::get_project_ray(int posx, int posy, Vec3 &ray_start, Vec3 &ray_direction) const
{
    GLdouble x,y,z;
    gluUnProject((GLdouble)posx,(GLdouble)posy,0.2,model_matrix_.data(), project_matrix_.data(), viewport_.data(),&x,&y,&z);
    ray_start = Vec3(x,y,z);
    gluUnProject((GLdouble)posx,(GLdouble)posy,1.0,model_matrix_.data(), project_matrix_.data(), viewport_.data(),&x,&y,&z);
    ray_direction = Vec3(x,y,z) - ray_start;
}

void Camera::get_cube_center_point(int posx, int posy, Vec3 &ray_start, Vec3 &ray_direction) const
{
    GLdouble x,y,z;
    gluUnProject((GLdouble)posx,(GLdouble)posy,0.9,model_matrix_.data(), project_matrix_.data(), viewport_.data(),&x,&y,&z);
    ray_start = Vec3(x,y,z);
}

Vec3 Camera::get_actual_position() const
{
    Quat quat;
    quat.convertFromMatrix(rotate_matrix_);

    HomoMatrix4 inverseRotateMatrix = quat.conjugate().convertToMatrix();
    return rotate_matrix_.transpose()*pos_ - offset_;
}

Vec3 Camera::getCameraPosition(int posx, int posy) const
{
    GLdouble x,y,z;
    gluUnProject((GLdouble)posx,(GLdouble)posy,0.0,model_matrix_.data(), project_matrix_.data(), viewport_.data(),&x,&y,&z);
    Vec3 ray_start = Vec3(x,y,z);
    //    gluUnProject((GLdouble)posx,(GLdouble)posy,1.0,model_matrix_.data(), project_matrix_.data(), viewport_.data(),&x,&y,&z);
    //    Vec3 ray_direction = Vec3(x,y,z) - ray_start;
    return ray_start;
}
Vec3 Camera::get_actual_center() const
{
    Quat quat;
    quat.convertFromMatrix(rotate_matrix_);

    return rotate_matrix_.transpose()*center_ - offset_;
}
Vec3 Camera::get_actual_direction() const
{
    Quat quat;
    quat.convertFromMatrix(rotate_matrix_);

    HomoMatrix4 inverseRotateMatrix = quat.conjugate().convertToMatrix();
    //if center=0,0,0
    return (inverseRotateMatrix*(center_-pos_)).normalize();
}

Vec3 Camera::get_actual_up() const
{
    Quat quat;
    //    model_matrix_.print();
    quat.convertFromMatrix(model_matrix_);
    HomoMatrix4 inverseModelMatrix = quat.conjugate().convertToMatrix();
    Vec3 direction=center_-pos_;
    Vec3 temp_up=up_-(up_*direction)*direction;
    return  (inverseModelMatrix * temp_up).normalize();
}

Scalar Camera::distance() const
{
    return (pos_ - center_).length();
}
void Camera::setDistance(Scalar dist){
    Scalar dist0=distance();
    pos_=center_+dist/dist0*(pos_-center_);
    emit signalCameraUpdated();
}

Vec<9,double> Camera::getParams(){
    Vec<9,double> params_;
    Vec4 rotate_quat=this->getQuatVecFromRotate();
    for(int i=0; i<4; i++){
        params_[i]=rotate_quat[i];
    }
    double d= this->distance();
    params_[4]=d;
    Vec3 offset=this->getOffset();
    for(int i=0; i<3; i++){
        params_[i+5]=offset[i];
    }
    params_[8]=this->focalLength();
    return params_;
}

void Camera::setParams(Vec<9, double> params_){

    Vec4 rotate_quat;
    for(int i=0; i<4; i++){
        rotate_quat[i]=params_[i];
    }
    this->setRotateFromQuatVec(rotate_quat);
    this->setDistance(params_[4]);
    Vec3 offset;
    for(int i=0; i<3; i++){
        offset[i]=params_[i+5];
    }
    set_offset(offset);
    slotSetFocalLength(params_[8]);

}

void Camera::slotSetFocalLength(double len){
    focal_length_=len;
    focal_angle_=atan(height_/focal_length_/2)/M_PI*180;
    emit signalCameraUpdated();
//    qDebug()<<"focal angle:"<<focal_angle_;
}

void Camera::update_viewport_and_matrix()
{
    GLfloat light_position0[] = {7,  -5, 8, 0.0f };
    GLfloat light_position1[] = {5,-6,7,0.0};
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);
    viewport_ = Vec4i(viewport);
    double projMatrix[16];
    glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
    project_matrix_ = HomoMatrix4(projMatrix);
    double modelMatrix[16];
    glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
    model_matrix_ = HomoMatrix4(modelMatrix);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position1);

}

}
