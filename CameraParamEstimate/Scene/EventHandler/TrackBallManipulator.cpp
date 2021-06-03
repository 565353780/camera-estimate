#include "TrackBallManipulator.h"
#include "../Camera.h"
#include <math.h>
#include <QDebug>
namespace GCL {

/**
 * @brief      在构造函数中设置初始旋转速度和放缩速度
 *
 */
TrackBallManipulator::TrackBallManipulator(Camera *camera)
    :Manipulator(camera)
{
    left_press_ = false;
    rotate_speed_ = 0.003;  //旋转速度，影响移动对
    zoom_speed_ = 0.05;     //放缩速度，影响滚轮对放缩的效果
}

/**
 * @brief      视角旋转函数，记录鼠标在屏幕上移动的两个方向上的距离，转化为旋转向量，并最终求出旋转后的视角位置
 * @param[in]  moveDirecition  旋转向量
 * @param[in]  angle  旋转向量在空间中的距离，知道视角位置的情况下可代表旋转的角度
 * @param[in]  axis  视角和单位旋转向量的外积
 */
void TrackBallManipulator::rotate(int posX, int posY)
{
    double PI= 3.14159;
    Vec3 upDirection,sideDirection;
    //x分量只有数值用上了，y分量用了方向。
    this->computeUpAndSideDirection(upDirection,sideDirection);
    Vec3 eye = Vec3(0,0,1);
    Vec3 moveDirection_y = upDirection ;

    Scalar angle_x = (posX-last_posX_)/2.0*PI*4; //moveDirection_x.length();
     Scalar angle_y = moveDirection_y.length()*2* (posY - last_posY_);

    moveDirection_y.Normalize();



    Vec3 axis_x = Vec3(0,0,1);//eye^moveDirection_x;
    axis_x[0]=axis_x[1]=0;
    Vec3 axis_y = eye^moveDirection_y;
    axis_y[2]=0;
    axis_x.Normalize();
    axis_y.Normalize();

    Quat quat_x(axis_x, rotate_speed_ *angle_x);
    HomoMatrix4 result=getCamera()->rotation_matrix();
    if(!((upDirection^sideDirection)[2] <-0.995 && (posY - last_posY_)<0) && !((upDirection^sideDirection)[2] >0.995 && (posY - last_posY_)>0))
    {
        Quat quat_y(axis_y, rotate_speed_ * angle_y);
        result *= quat_y.convertToMatrix();
    }


    result *= quat_x.convertToMatrix();

    getCamera()->set_camera_rotateMatrix(result);

}

/**
 * @brief      平移视角函数，当鼠标前后指向的两点所在的光线都在视野内时，对视角进行平移
 *
 */
void TrackBallManipulator::pan(int posX, int posY)
{
    Vec3 hitOld;
    int hit0 = hitCenterPlane(last_posX_,last_posY_, hitOld);
    Vec3 hitNew;
    int hit1 = hitCenterPlane(posX,posY,hitNew);
    if(hit0 && hit1)
    {
        getCamera()->move(hitNew - hitOld);
    }

}

void TrackBallManipulator::rotateAlongDir(double angle){
    Vec3 direction=getCamera()->get_actual_direction();
    Quat quat_rotate(direction,angle);
    HomoMatrix4 result=getCamera()->rotation_matrix();
     result*= quat_rotate.convertToMatrix();
     getCamera()->set_camera_rotateMatrix(result);
}



/**
 * @brief      计算当前视角下的x,y方向代表的模型坐标下的方向
 * 
 *
 */
void TrackBallManipulator::computeUpAndSideDirection(Vec3 &upDirection, Vec3 &sideDirection)
{
    Vec4i viewport = this->getCamera()->get_viewport();
    Vec3  direction = this->getCamera()->get_actual_direction();
    Vec3  center = this->getCamera()->center();
    Vec3 rayPoint,rayDir, intersect0, intersect1, intersect2;


    viewport[0] = viewport[0] + viewport[2] / 2;
    viewport[1] = viewport[1] + viewport[3] / 2;
   // viewport[0] = last_posX_;
  //  viewport[0] = last_posY_;

    this->getCamera()->get_project_ray(viewport[0],viewport[1],rayPoint,rayDir);
    Vec3::getIntersectionRayToPlane(rayPoint,rayDir,center,direction,intersect0);

    this->getCamera()->get_project_ray(viewport[0] + 1,viewport[1],rayPoint,rayDir);
    Vec3::getIntersectionRayToPlane(rayPoint,rayDir,center,direction,intersect1);


    this->getCamera()->get_project_ray(viewport[0],viewport[1] + 1,rayPoint,rayDir);
    Vec3::getIntersectionRayToPlane(rayPoint,rayDir,center,direction,intersect2);

    upDirection = intersect2 - intersect0;
    upDirection.Normalize();

    sideDirection = intersect1 - intersect0;
    sideDirection.Normalize();


}
/**
 * @brief      返回旋转速度
 *
 * @return     旋转速度
 */
Scalar TrackBallManipulator::speed() const
{
    return rotate_speed_;
}

/**
 * @brief      重设旋转速度
 *
 */
void TrackBallManipulator::setSpeed(const Scalar &speed)
{
    rotate_speed_ = speed;
}

/**
 * @brief      鼠标移动函数
 *
 *
 * @return     { description_of_the_return_value }
 */
int TrackBallManipulator::mouseMove(int posX, int posY)
{
    if(getModifier() == NoModifier )
    {
        if(left_press_ == false)
        {
            if(getButton() == RightButton)
            {
                rotate(posX,posY);
            }
            else if(getButton() == MidButton)
            {
                pan(posX,posY);
            }
            last_posX_ = posX;
            last_posY_ = posY;
        }
    }
    return EventHandler::mouseMove(posX,posY);
}

/**
 * @brief      鼠标按下事件
 *
 */
int TrackBallManipulator::mousePress(int posX, int posY, unsigned int button, unsigned int modifier)
{
    if(modifier == NoModifier)
    {
        last_posX_ = posX;
        last_posY_ = posY;
        if(button == LeftButton)
            left_press_ = true;
            //originPosX=posX;
            //originPosY=posY;

    }

    return EventHandler::mousePress(posX,posY,button,modifier);
}

int TrackBallManipulator::mouseDoubleClick(int posX, int posY, unsigned int button, unsigned int modifier)
{
    if(modifier == NoModifier)
    {
        last_posX_ = posX;
        last_posY_ = posY;
    }
    return EventHandler::mousePress(posX,posY,button,modifier);
}

/**
 * @brief       鼠标松开事件
 *
 *
 */
int TrackBallManipulator::mouseRelease(int posX, int posY, unsigned int button, unsigned int modifier)
{
    if(getModifier() == NoModifier)
    {
        last_posX_ = -1;
        last_posY_ = -1;
    }
    if(getButton()==RightButton)
    {
        // suit workplaneangle to  -7.5~-2->-5  -2~2->0  2~7.5->5
    }
    if(button == LeftButton)
        left_press_ = false;
    return EventHandler::mouseRelease(posX,posY,button,modifier);
}

/**
 * @brief      滚轮事件，控制放缩，将视角移近或远
 *
 * @param[in]  delta  滚轮滚动的距离
 *
 */
int TrackBallManipulator::wheel(const float &delta)
{
    Scalar scale =  (1 + ((delta > 0)?1:-1)*zoom_speed_ );
    Scalar result = getCamera()->distance()*scale;
    if( result>10 && result < 1000)
    getCamera()->setDistance(result);

    return 0;
}

/**
 * @brief      判断光线是否在视窗内
 *
 *
 */
int TrackBallManipulator::hitCenterPlane(int posX, int posY, Vec3 &intersect)
{

    Vec3 center = getCamera()->center();
    Vec3 np = getCamera()->get_actual_direction();
    Vec3 rayPoint,rayDir;
    getCamera()->get_project_ray(posX,posY,rayPoint,rayDir);//由当前点窗口坐标和视角坐标求得光线
    //rayPoint.print();
    rayDir.Normalize();
    return Vec3::getIntersectionRayToPlane(rayPoint,rayDir,center,np,intersect);//求光线和平面的交点
}


}
