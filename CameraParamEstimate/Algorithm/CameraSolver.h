#ifndef CAMERASOLVER_H
#define CAMERASOLVER_H

#include <QVector>
#include <QPointF>
#include <QObject>

#include "../Scene/Camera.h"
using namespace GCL;
class DistField;
class KDTreeDistField;
class MultiTrackKDTreeDistField;
class CameraSolver: public QObject
{
    Q_OBJECT
public:
    CameraSolver(Camera*camera);
private:
    Camera* solver_camera_;//camera control the scene
    Vec<9,double> params_;//rotate: x,y,z,w; distance: d; offset: x,y,z;  focal_length: f;
    QVector<Vec3> data_set_;// 2D point given by user
    QVector<Vec3> unprojected_data_set_;// data_set unproject to ground
    QVector<QVector<Vec3>> projected_sample_set_;// sample set projected to screen
    QVector<Vec3> sample_set_;// 3D point on track boundary
    QVector<QVector<Vec3>> multi_track_sample_set_;
    QVector<QVector<Vec3>> projected_multi_track_sample_set_;
    double track_radius_;
    double track_length_;
    DistField* sample_dist_field_=NULL;
    KDTreeDistField* kdtree_dist_field_=NULL;
    MultiTrackKDTreeDistField* multi_boundary_dist_field_=NULL;

    HomoMatrix<3,double> affine_matrix_;
    Vec<8,double> affine_params_;

    bool have_init=false;//--------
    double step_now=100;//--------
public:
    DistField* getDistField(){return sample_dist_field_;}

    QVector<QVector<Vec3>> &getProjectedSampleSet(){return projected_sample_set_;}
    /**
     * @brief setTrack      set track param
     * @param radius
     * @param length
     */
    void setTrack(double radius, double length){track_radius_=radius; track_length_=length;}

    Vec<9, double> getCameraParams();
    void setCameraParams(Vec<9, double> params);

    void set2DParam(Vec<8, double> params);
    void setDataSet(QVector<Vec3> &points);
    void setDataSet(QVector<QPointF> &points);

    void addRect(double x, double y, double w, double h, int num, QVector<Vec3> &output);

    /**
     * @brief generateSampleSet     generate sample set on track boundary
     * @param num                   num+1 point on y>0 and y<0 equalily distrubuted on x;
     */
    void generateSampleSet(int num);
    void generateUniformSampleSet(int num);// num is number on each section total number=4*num
    void generateCurlingSampleSet(int num);
    void generateDistanceField();

    void printSample();
    void printProjectedSample();

    void updateSampleDistField();
    void updateMultiTrackSampleDistField();

    void updateUnprojectedData();
    void updateProjectedSample();

    void update2DProjectedSample();
    void updateProjectedMultiTrackSample();

    void renderProjectedSamples();
    QVector<Vec3> &getUnprojectedData(){return unprojected_data_set_;}

    /**
     * @brief pointSetMatchError    compute distance  of input_set to target_set
     * @param target_set
     * @param input_set
     * @return                      sum(dist(v,input_set) | v in target_set)
     */
    double pointSetMatchError(QVector<Vec3> &target_set, QVector<Vec3> &input_set);

    double pointSetToDistField(QVector<Vec3> &target_set);

    double distPointToPointSet(Vec3 P, QVector<Vec3> &set2);//2D error on screen
    double distPointToTrack(Vec3 P, double radius, double length); // 3D error on ground
    Vec3 trackPoint(double radius, double length, double x, int signY);

    /**
     * @brief unprojectError    after unproject to ground, dist to track
     * @param P
     * @return
     */
    double unprojectError(Vec3 P);
    /**
     * @brief projectError      after project to screen, dist to data_set
     * @param P
     * @return
     */
    double projectError(Vec3 P);

    /**
     * @brief avgUnprojectError dist(sample_set,unprojected_data_set)
     * @return
     */
    double avgUnprojectError();

    /**
     * @brief avgProjectError   dist(data_set, projected_sample_set)
     * @return
     */
    double avgProjectError(int track_type=0);

    double avg2DProjectError(int bound_mode=0);

    /**
     * @brief deltaAvgUnprojectError    [df/dxi,...,]T, f=avgUnprojectError
     * @param delta
     * @return
     */
    Vec<9, double> deltaAvgUnprojectError(double delta);

    /**
     * @brief deltaAvgProjectError      [df/dxi,...,]T, f=avgProjectError
     * @param delta
     * @return
     */
    Vec<9, double> deltaAvgProjectError(double delta);

    Vec<8, double> delta2DAvgProjectError(double delta, int mode=0, int bound_mode=0);

    /**
     * @brief steepestDescentMethod     find gradient dir in each iteration, then find a step make f decrease
     * @param error_type                project error/ unproject
     *
     * currently use gradient dir.norm()<0.1 or max_iter=4000 to stop, and use samples generate by generateSampleSet(20);
     */
    void steepestDescentMethod(int error_type=0); //0:project error; 1:unproject error;

    void steepestDescentMethod2D();

    void init_param();
signals:
    void currentError(double);

};

#endif // CAMERASOLVER_H
