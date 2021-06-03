#include "DistField.h"

#include <QDebug>

DistField::DistField(int w, int h, double max_dist, int active_radius):width_(w),height_(h),max_dist_(max_dist),active_radius_(active_radius)
{

}

void DistField::clear(){
    dist_field_value_.fill(max_dist_);
}

void DistField::resize(int w, int h){
    dist_field_value_.resize(w*h);
    dist_field_value_.fill(max_dist_);
    qDebug()<<"w,h"<<width_<<height_;
}

void DistField::setValue(int i, int j, double value){
    if(i>=0 && i<width_ && j>=0 || j<height_)
        dist_field_value_[i+j*width_]=value;
}

double DistField::getValue(int i, int j){
    if(i<0 || i>=width_ || j<0 || j>=height_)
        return max_dist_;
    return dist_field_value_[i+j*width_];
}

void DistField::computeDistFromPolygon(QVector<Vec3> &point_list){
    clear();
    updateDistFromPolygon(point_list);
}

double sqrDistToSegment(double x, double y, double x0, double y0, double x1, double y1){
//    qDebug()<<"enter compute x,y:"<<x<<y;
    Vec2 p(x,y);
    Vec2 p0(x0,y0);
    Vec2 p1(x1,y1);
    Vec2 diff=(p-p0), dir=(p1-p0).normalize();
    double l=diff*dir, l01=(p1-p0).norm();
    double result=1e8;
    if(l<0){
        result= (p0-p).sqrnorm();
    }
    else if(l>l01){
        result= (p1-p).sqrnorm();
    }
    else{
        Vec2 ph=diff-l*dir;
        result= ph.sqrnorm();
    }
//    qDebug()<<"has result"<<result;
    return result;

}

double sqrDistToSegment(Vec3 p, Vec3 p0, Vec3 p1){
    Vec3 diff=(p-p0), dir=(p1-p0).normalize();
    double l=diff*dir, l01=(p1-p0).norm();
    double result=1e8;
    if(l<0){
        result= (p0-p).sqrnorm();
    }
    else if(l>l01){
        result= (p1-p).sqrnorm();
    }
    else{
        Vec3 ph=diff-l*dir;
        result= ph.sqrnorm();
    }
//    qDebug()<<"has result"<<result;
    return result;
}

void DistField::updateDistFromPolygon(QVector<Vec3> &point_list){
    int num=point_list.size();
    for(int i=0; i<num; i++){
        updateDistALongSeg(point_list[i],point_list[(i+1)%num]);
        updateDistFromSeg(point_list[i],point_list[(i+1)%num],0,active_radius_,1);
        updateDistFromSeg(point_list[i],point_list[(i+1)%num],active_radius_,2*active_radius_,2);
        updateDistFromSeg(point_list[i],point_list[(i+1)%num],2*active_radius_,8*active_radius_,5);
    }

}

void DistField::updateDistFromSeg(Vec3 p0, Vec3 p1, int min, int max, int interval){
    int x0=p0[0], y0=p0[1];
//        qDebug()<<"computing x,y:"<<x0<<y0;
    for(int x=x0-(max); x<=(x0+(max)); x+=interval){
        if(abs(x-x0)<min) continue;
        for(int y=y0-(max); y<(y0+(max));y+=interval){
            if(abs(y-y0)<min) continue;
            if(x<0 || x>=width_ || y<0 || y>=height_) continue;
            double dist=sqrDistToSegment(Vec3(x,y,0),p0,p1);
            if(dist<getValue(x,y)){
                setValue(x,y,dist);
            }
        }
    }
}

void DistField::updateDistALongSeg(Vec3 p0, Vec3 p1){
    int w=10;
    int x0=p0[0], y0=p0[1], x1=p1[0], y1=p1[1];
    int xmin=x0<x1?x0:x1, xmax=x0>x1?x0:x1, ymin=y0<y1?y0:y1, ymax=y0>y1?y0:y1;
//        qDebug()<<"computing x,y:"<<x0<<y0;
    for(int x=xmin-w; x<=xmax+w; x++){
        for(int y=ymin-w; y<ymax+w;y++){
            if(x<0 || x>=width_ || y<0 || y>=height_) continue;
            double dist=sqrDistToSegment(Vec3(x,y,0),p0,p1);
            if(dist<getValue(x,y)){
                setValue(x,y,dist);
            }
        }
    }
}

void DistField::print(){
    for(int j=0; j<height_; j++){
        QStringList stringlist;
        for(int i=0; i<width_; i++){
            stringlist<<QString::number(dist_field_value_[i+j*width_]);
        }
        qDebug()<<stringlist;
    }
}
