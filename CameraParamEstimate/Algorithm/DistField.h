#ifndef DISTFIELD_H
#define DISTFIELD_H

#include "../Math/MathDefines.h"
#include <QVector>
using namespace GCL;
class DistField
{
public:
    DistField(int w, int h, double max_dist=1e6, int active_radius=5);
private:
    int width_ ;
    int height_;
    double max_dist_;
    int active_radius_;
    QVector<double> dist_field_value_;
public:
    void clear();
    void resize(int w, int h);
    void setValue(int i, int j, double value);
    double getValue(int i, int j);
    void computeDistFromPolygon(QVector<Vec3> &point_list);
    void updateDistFromPolygon(QVector<Vec3> &point_list);
    void updateDistFromSeg(Vec3 p0, Vec3 p1, int min, int max, int interval);
    void updateDistALongSeg(Vec3 p0, Vec3 p1);
    void print();
};

#endif // DISTFIELD_H
