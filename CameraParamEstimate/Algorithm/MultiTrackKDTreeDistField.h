#ifndef MULTITRACKKDTREEDISTFIELD_H
#define MULTITRACKKDTREEDISTFIELD_H

#include <QVector>
#include "Math/MathDefines.h"

using namespace GCL;
class KDTreeDistField;

class MultiTrackKDTreeDistField
{
public:
    MultiTrackKDTreeDistField();
    MultiTrackKDTreeDistField(QVector<QVector<Vec3>> &track_point_list);
    ~MultiTrackKDTreeDistField();

private:
    QVector<KDTreeDistField*> track_field_list_;

public:
    void setPointSetList(QVector<QVector<Vec3>> &track_point_list);
    double getDistance(Vec3 p);
};

#endif // MULTITRACKKDTREEDISTFIELD_H
