#ifndef LANEFITTING_H
#define LANEFITTING_H
#include <QVector>
#include <QTransform>
#include <QColor>
#include <eigen/Eigen/Core>
#define ZERO_TORELANCE 10e-7
//typedef Matrix<double,Dynamic,Dynamic> DMatrixXd;
//typedef Matrix<double,3,3> DMatrix3d;
namespace cv {
class Mat;
}
using namespace Eigen;
class TrackGround;

class MyCamera{
public:
    MyCamera();
private:
    Matrix3d    rotation_matrix_; //(U,T,N)
    Vector3d    center_;
    double      focal_length_=100;
    QVector<Vector3d> P_;
    QVector<QPointF>  Q_;
    QVector<Vector3d> Ft_Q_;
    QVector<QPointF> F_P_;
    QPolygonF   polygon_P_;
    QPolygonF   polygon_Ft_Q_;
    double      area_P_;
    double      area_Ft_Q_;

public:
    void        setRotationFromVectors(Vector3d N, Vector3d U, Vector3d T);
    void        setRotationFromEulerAngle(double thetaX,double thetaY, double thetaZ);
    void        setCenter(Vector3d center){center_=center;}
    void        setFocalLength(double length){focal_length_=length;}
    void        setDataset(QVector<Vector3d> &OriginPointList, QVector<QPointF> &projectedPointList);

public:
    bool        projectToScreen(Vector3d P, QPointF &ScreenPoint);
    bool        unprojectToGround(QPointF point, Vector3d &P);

    bool        computeProjection();
    bool        computeUnprojection();

    double      distToTrack(Vector3d &P, TrackGround* track);

    double      projectAvgError();
    double      unprojectAvgError();

    double      computeCenterFromRotation();
    double      moveCenterZtoFit();
    double      moveCenterXYtoFit();
    void        focalLengthFitting();
    void        moveCenterForProjection();
    void        printCameraInfo();

    Vector4d    solveCameraParam(Vector2d x_range, int x_divide, Vector2d y_range, int y_divide, Vector2d z_range, int z_divide);

};

class TrackGround{
public:
    TrackGround();
    TrackGround(double straightLaneLength, double circleLaneRadius, double lane_width, int lane_number);

private:
    //400m--straight lenght:84.39m,circle radius:36.5m
    //skate --straight:28.85m, radius: 8.00m
    double straight_lane_length_;
    double circle_lane_radius_;
    double lane_width_;
    int lane_number_=0;
    QPainterPath myPath_;
public:
    void getLengthAndRadius(double &length, double &radius);
    double distFromPoint(Vector3d &P);
    /**
     * @brief afterProjection   3D point:type,u,v on ground project to screen
     * @param u                 straight lane coord
     * @param v                 vertical to straight coord
     */
    QPointF afterProjection(double u, double v, MyCamera* camera);

    /**
     * @brief circlePointAfterProjection
     * @param type              0:left circle center, 1:right center
     * @param r                 radius from the center
     * @param theta             angle start from straight right counterclockwise, as polar system
     * @return
     */
    QPointF circlePointAfterProjection(int type, double r, double theta, MyCamera* camera);

    /**
     * @brief linePointAfterProjection
     * @param type              0:top, 1:bottom
     * @param w                 coord on straight line axis
     * @param index             index of lane
     * @param camera_rotation   3*3 camera rotation Matrix(U,T,N) last column
     * @param camera_center     camera center
     * @param focal_length      camera center distance to screen
     * @return                  position
     */
    QPointF linePointAfterProjection(int type, double w, int index, MyCamera *camera);

    void updatePath(MyCamera *camera);
    void paint(QPainter *painter);
};

class LaneFitting
{
public:
    LaneFitting();

private:
    Matrix<double, Dynamic,Dynamic,3> frame;
    TrackGround* track_ground_;
    MyCamera*   camera_;
    QVector<int> detected_lines_;
    Vector3d camera_center_;
    Vector3d camera_up_;
    Vector3d camera_direction_;
    double z_near_;
    double half_w_;
    double half_h_;
    QVector<QPointF> data_set_;

public:
    void setDataSet(QVector<QPointF> &OriginPointList, QVector<QPointF> &projectedPointList);
    double *solveCamera(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax, double focal_length=1000);
    VectorXd solveCameraParamByDeclining(Vector3d rotation, Vector3d center, double focal_length);
    void detectLine();
    void detectEllipse();
    void paint(QPainter *painter);
    double  UnProjectDist(QPointF P, double thetaX, double thetaY, double thetaZ, double CenterX, double CenterY, double CenterZ, double focal_length);
    double avgUnprojectError(VectorXd param_vector);
    VectorXd deltaAvgUnprojectError(VectorXd param_vector, double delta);

    VectorXd decliningMethod(VectorXd &init_param);
};

#endif // LANEFITTING_H
