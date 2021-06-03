#ifndef GCL_SCENEGRAPH_CAMERA_H
#define GCL_SCENEGRAPH_CAMERA_H
#include "../Math/MathDefines.h"
#include <QObject>
namespace GCL{
/**
 * @brief 相机类
 * -# 提供初始化、改变相机的位置、方向、视窗等函数
 * -# 提供视窗坐标与模型坐标的转化
 * @author Zhao Sun(sunzhao@mail.ustc.edu.cn)
 */
class  Camera : public QObject
{
    Q_OBJECT
public:

    /**
     * @brief      构造函数
     * 
     * 设置了：
     * -# 初始位置
     * -# 静止位置
     * -# 中心位置
     * -# 法向up
     * 
     */
    Camera();

    /**
     * @brief      进入相机视角
     */
    void enter();

    /**
     * @brief      离开相机视角
     */
    void leave();

    const Vec3& position() const {return pos_;}
    Vec3& position() {return pos_;}
    Vec3& default_pos(){return default_pos_;}
    const Vec3& center() const {return center_;}
    Vec3& center() {return center_;}
    const Vec3& up() const {return up_;}
    Vec3& up() {return up_;}
    double focalLength(){return focal_length_;}

    double focalAngle(){return focal_angle_;}

    void set_position(const Vec3& pos) {pos_ = pos;emit signalPosChange(pos[0],pos[1],pos[2]);}
    //void set_default_pos(const Vec3& pos){default_pos_ = pos;}
    void set_orth(bool t);
    void set_center(const Vec3 &center) {center_ = center;}
    void set_offset(const Vec3 &offset) {offset_ = offset;}
    void set_up(const Vec3 &up) {up_ = up;}
    void set_position(Scalar x, Scalar y, Scalar z) {pos_[0] = x; pos_[1] = y; pos_[2] = z; emit signalPosChange(x,y,z);}
    void set_center(Scalar x, Scalar y, Scalar z) {center_[0] = x; center_[1] = y; center_[2] = z;}
    void sut_up(Scalar x, Scalar y, Scalar z ) {up_[0] = x; up_[1] = y; up_[2] = z; }
    void set_camera_rotateMatrix( HomoMatrix4& rotation_matrix){rotate_matrix_=rotation_matrix; emit signalCameraUpdated();}
    const HomoMatrix4& rotation_matrix() const {return rotate_matrix_;}
    HomoMatrix4& rotation_matrix() {return rotate_matrix_;}

    void setRotationFromQuat(Scalar x, Scalar y, Scalar z, Scalar w );
    void setRotateFromQuatVec(Vec4);
    Vec4 getQuatVecFromRotate() const;
    /**
     * @brief      投影映射函数
     *
     * @param[in]  point  输入世界坐标
     *
     * @return     返回视窗坐标
     */
    Vec3 projectToScreen(const Vec3 &point) const;
    /**
     * @brief      投影逆映射函数
     *
     * @param[in]  point  输入视窗坐标
     *
     * @return     返回世界坐标
     */
    Vec3 unprojectToWorld(const Vec3& point) const;

    Vec3 unprojectToGround(const Vec3& point) const;

    const HomoMatrix4& get_project_matrix() const { return project_matrix_;}
    const HomoMatrix4& get_model_matrix() const { return model_matrix_; }
    
    /**
     * @brief      获得视窗
     *
     * @return     返回Vec4i viewport_
     * .
     */
    const Vec4i& get_viewport() const {return viewport_;}
    void set_viewport(int w, int h);

    /**
     * @brief      输入视窗上的坐标（x，y）获得模型三维坐标
     *
     * @param[in]  posx           坐标x
     * @param[in]  posy           坐标y
     * @param      ray_start      用以返回 起始点
     * @param      ray_direction  用以返回 方向（未归一化）
     */
    void get_project_ray(int posx, int posy, Vec3& ray_start, Vec3& ray_direction) const;
    
    /**
     * @brief      输入视窗上的坐标（x，y）获得立方体中心坐标
     * @param[in]  posx           坐标x
     * @param[in]  posy           坐标y
     * @param      ray_start      用以返回中心的坐标
     * @param      ray_direction  没用
     */
    void get_cube_center_point(int posx, int posy, Vec3& ray_start, Vec3& ray_direction) const;

    Vec3 get_actual_position() const;
    Vec3 getCameraPosition(int posx, int posy) const;
    Vec3 get_actual_direction() const;
    Vec3 get_actual_center()const;
    Vec3 get_actual_up() const;
    void move(const Vec3& v) {offset_ += v; emit signalCameraUpdated();}
    Vec3 getOffset() const {return offset_;}
    void set_default_offset() {offset_ = Vec3();emit signalCameraUpdated();}

    /**
     * @brief 将mesh网格点投影到视景体的近平面，可见范围为[-1,1]之间
     * @param point
     * @return
     */
    Vec3 getProjectToScreenOnUniformRange(Vec3 &point);
    /**
     * @brief      得到当前坐标与场景中心的距离
     *
     * @return     距离
     */
    Scalar distance() const;

    void setDistance(Scalar dist) ;

    Vec<9,double> getParams();

    void setParams(Vec<9,double> params_);
public slots:
    void slotSetFocalLength(double len);
//    void slotRotateRight();
//    void slotRotateLeft();
//    void slotRotateUp();
//    void slotRotateDown();
signals:
    void signalPosChange(double x, double y, double z);
    void signalCameraUpdated();
public:
    void update_viewport_and_matrix();
private:

    Vec3 pos_;
    Vec3 default_pos_;
    Vec3 center_;
    Vec3 up_;

    Vec4i viewport_;
    HomoMatrix4 project_matrix_;
    HomoMatrix4 model_matrix_;

    HomoMatrix4 rotate_matrix_;
    Vec3 offset_;
    int width_;
    int height_;
    bool isOrth_;
    double focal_length_;
    double focal_angle_;

};
}

#endif // CAMERA_H
