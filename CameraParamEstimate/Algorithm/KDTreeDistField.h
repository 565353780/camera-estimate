#ifndef KDTREEDISTFIELD_H
#define KDTREEDISTFIELD_H

#include <QVector>

#include "../Math/MathDefines.h"
#include "Algorithm/kd-tree/kdtree.h"
using namespace GCL;

class KDTreeDistField
{
public:
    KDTreeDistField(QVector<Vec3> &point_list);
    ~KDTreeDistField();
    void setPointSet(QVector<Vec3> &point_list);
    void setDirSet();
private:
    KDTree::KDTree<3>* kdtree_dist_field_;
    std::vector<Vec3> point_set_;
    std::vector<Vec3> dir_set_;
public:
    double getDistance(Vec3 p);
};

#endif // KDTREEDISTFIELD_H
