#include "MultiTrackKDTreeDistField.h"

#include "KDTreeDistField.h"

MultiTrackKDTreeDistField::MultiTrackKDTreeDistField()
{
}

MultiTrackKDTreeDistField::MultiTrackKDTreeDistField(QVector<QVector<Vec3> > &track_point_list)
{
    setPointSetList(track_point_list);
}

MultiTrackKDTreeDistField::~MultiTrackKDTreeDistField()
{
    int num=track_field_list_.size();
    for(int i=0; i<num; i++){
        delete track_field_list_[i];
    }
    track_field_list_.clear();
}

void MultiTrackKDTreeDistField::setPointSetList(QVector<QVector<Vec3> > &track_point_list)
{
    int num=track_field_list_.size();
    for(int i=0; i<num; i++){
        delete track_field_list_[i];
    }
    track_field_list_.clear();
    int track_num=track_point_list.size();
    for(int i=0; i<track_num; i++){
        KDTreeDistField* track_field= new KDTreeDistField(track_point_list[i]);
        track_field_list_.push_back(track_field);
    }

}

double MultiTrackKDTreeDistField::getDistance(Vec3 p)
{
    if(track_field_list_.size()<1) return NAN;
    double result=track_field_list_[0]->getDistance(p);
    if(track_field_list_.size()<2) return result;
    int num=track_field_list_.size();
    for(int i=1; i<num; i++){

        double tmp_dist=track_field_list_[i]->getDistance(p);
        if(tmp_dist<result)
            result=tmp_dist;
    }
    return result;
}
