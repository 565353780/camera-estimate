#include "KDTreeDistField.h"
#include "kd-tree/kdtree.h"

#include <QDebug>

KDTreeDistField::KDTreeDistField(QVector<Vec3> &point_list)
{
    point_set_=point_list.toStdVector();
    int num=point_set_.size();
    for(int i=0; i<num; i++){
        point_set_[i][2]=i;
    }
    setDirSet();
    kdtree_dist_field_=new KDTree::KDTree<3>(point_set_,Vec3(0,0,0),Vec3(10000,10000,0),2);
}

KDTreeDistField::~KDTreeDistField()
{
    delete kdtree_dist_field_;
}
void KDTreeDistField::setDirSet(){
    int num=point_set_.size();
    dir_set_.resize(num);
    for(int i=1; i<num; i++){
        dir_set_[i-1]=point_set_[i]-point_set_[i-1];
        dir_set_[i-1][2]=0;
        dir_set_[i-1].Normalize();

    }
    dir_set_[num-1]=point_set_[0]-point_set_[num-1];
    dir_set_[num-1][2]=0;
    dir_set_[num-1].Normalize();
}

void KDTreeDistField::setPointSet(QVector<Vec3> &point_list){
    point_set_=point_list.toStdVector();
    int num=point_set_.size();
    for(int i=0; i<num; i++){
        point_set_[i][2]=i;
    }
    setDirSet();
    delete kdtree_dist_field_;
    kdtree_dist_field_=new KDTree::KDTree<3>(point_set_,Vec3(0,0,0),Vec3(10000,10000,0),2);
}

double sqrPointDistToSegment(Vec3 p, Vec3 p0, Vec3 p1){
    p[2]=0; p0[2]=0; p1[2]=0;
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

double sqrMinDistToTriangle(Vec3 p, Vec3 p0, Vec3 dir1, Vec3 dir2){
    Vec3 diff=p-p0;
    double result=1e8;
    bool flag=true;
    if(diff*dir1>0)
    {
        Vec3 ph1=diff-(diff*dir1)*dir1;
        result=fmin(ph1.sqrnorm(),result);
        flag=false;
    }
    if (diff*dir2>0) {
        Vec3 ph2=diff-(diff*dir2)*dir2;
        result=fmin(ph2.sqrnorm(),result);
        flag=false;
    }
    if(flag){
        result=diff.sqrnorm();
    }
    return result;
}

double KDTreeDistField::getDistance(Vec3 p){
    std::vector<double> distance_set;
    std::vector<KDTree::KDTree<3>::Node*> node_set;
    //kdtree_dist_field_->findKNearest(kdtree_dist_field_->root_, 1,p,distance_set,node_set);
    int index=node_set[0]->data_[2];
//    qDebug()<<"find k-nearest success:"<<index;
    int num= point_set_.size();
    Vec3 p0=point_set_[0];
    double result=sqrPointDistToSegment(p,point_set_[0],point_set_[1]);

    for(int i=1; i<num; i++)
    {
        result=fmin(result,sqrPointDistToSegment(p,point_set_[i],point_set_[(i+1)%num]));
    }

//    if(result>400) return 1e8;
    return result;

}
