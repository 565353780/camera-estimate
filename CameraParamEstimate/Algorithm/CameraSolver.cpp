#include "CameraSolver.h"
#include <QDebug>
#include <iostream>
#include <QMultiHash>


#include "Algorithm/DistField.h"
#include "Algorithm/KDTreeDistField.h"
#include "Algorithm/MultiTrackKDTreeDistField.h"

CameraSolver::CameraSolver(GCL::Camera *camera):solver_camera_(camera)
{
    params_=Vec<9,double> ();
    affine_params_=Vec<8,double> ();
    affine_params_[0]=16.84;
    affine_params_[3]=5.46;
    //affine_params_[4]=196;
    //affine_params_[5]=332;

    affine_matrix_=HomoMatrix<3,double> ();
    set2DParam(affine_params_);
    affine_matrix_.Unit();
    this->getCameraParams();
}


Vec<9, double> CameraSolver::getCameraParams(){
    Vec4 rotate_quat=solver_camera_->getQuatVecFromRotate();
    for(int i=0; i<4; i++){
        params_[i]=rotate_quat[i];
    }
    double d= solver_camera_->distance();
    params_[4]=d/100;
    Vec3 offset=solver_camera_->getOffset();
    for(int i=0; i<3; i++){
        params_[i+5]=offset[i]/100;
    }
    params_[8]=solver_camera_->focalLength()/2000;
    return params_;
}
void CameraSolver::setCameraParams(Vec<9, double> params){
    Vec4 rotate_quat;
    for(int i=0; i<4; i++){
        rotate_quat[i]=params[i];
    }
    solver_camera_->setRotateFromQuatVec(rotate_quat);
    solver_camera_->setDistance(params[4]*100);
    Vec3 offset;
    for(int i=0; i<3; i++){
        offset[i]=params[i+5]*100;
    }
    solver_camera_->set_offset(offset);
    solver_camera_->slotSetFocalLength(params[8]*2000);
    solver_camera_->enter();
}

void CameraSolver::set2DParam(Vec<8, double> params)
{
    for(int i=0; i<2; i++){
        for(int j=0; j<2; j++){
//            qDebug()<<"setting param"<<i<<j<<params[2*i+j];
            affine_matrix_.setValue(i,j,params[2*i+j]);
        }
    }
    for(int i=0; i<2; i++){
        affine_matrix_.setValue(i,2,params[4+i]);
        affine_matrix_.setValue(2, i, params[6 + i]);
    }
//    qDebug()<<"finish set param";
}


void CameraSolver::setDataSet(QVector<Vec3> &points){
    data_set_=points;
}
void CameraSolver::setDataSet(QVector<QPointF> &points){
    int num=points.size();
    data_set_.clear();
    for(int i=0; i<num; i++){
        data_set_.push_back(Vec3(points[i].x(),points[i].y(),0));
        qDebug()<<"dataset i:"<<points[i].x()<<points[i].y();
    }
}

void addCircle(double x, double y, double r, int num, QVector<Vec3> &output){
    for(int i=0; i<num; i++){
        output.push_back(Vec3(x+r*cos(i*M_PI*2/num),y+y*sin(i*M_PI*2/num),0));
    }
}

void CameraSolver::addRect(double x, double y, double w, double h, int num, QVector<Vec3> &output){
    //output.reserve(output.size()+4*num);
    for(int i=0; i<num; i++){
        output.push_back(Vec3(x-w/2+i*w/num,y-h/2,0));
    }
    for(int i=0; i<num; i++){
        output.push_back(Vec3(x+w/2,y-h/2+i*h/num,0));
    }
    for(int i=0; i<num; i++){
        output.push_back(Vec3(x+w/2-i*w/num,y+h/2,0));
    }
    for(int i=0; i<num; i++){
        output.push_back(Vec3(x-w/2,y+h/2-i*h/num,0));
    }
}

void CameraSolver::generateSampleSet(int num){
    double xmin=-track_radius_-track_length_/2;
    double xstep=(track_length_+2*track_radius_)/num;
    double x;
    int signY=-1;
    sample_set_.clear();
    addRect(0,0,4.75,44.5,4,sample_set_);
//    qDebug()<<"generating:";
//    int count=0;
//    for(int i=num; i>-1; i--){
//        x=xmin+i*xstep;
//        Vec3 current_point=this->trackPoint(track_radius_,track_length_,x,signY);
//        sample_set_.push_back(current_point);
//        qDebug()<<"sample point:"<<count<<","<<current_point[0]<<current_point[1];
//        count++;
//    }
//    signY*=-1;
//    for(int i=0; i<num+1; i++){
//        x=xmin+i*xstep;
//        Vec3 current_point=this->trackPoint(track_radius_,track_length_,x,signY);
//        sample_set_.push_back(current_point);
//        qDebug()<<"sample point:"<<count<<","<<current_point[0]<<current_point[1];
//        count++;
//    }

}

void CameraSolver::generateUniformSampleSet(int num)// num is number on each section total number=4*num
{
    sample_set_.clear();
    for(int i=0; i<num; i++){
        sample_set_.push_back(Vec3(-track_length_/2+i*track_length_/num,track_radius_,0));
    }
    for(int i=0; i<num; i++){
        sample_set_.push_back(Vec3(track_length_/2+track_radius_*cos(M_PI/2-i*M_PI/num),track_radius_*sin(M_PI/2-i*M_PI/num)));
    }
    for(int i=0; i<num; i++){
        sample_set_.push_back(Vec3(track_length_/2-i*track_length_/num,-track_radius_,0));
    }
    for(int i=0; i<num; i++){
        sample_set_.push_back(Vec3(-track_length_/2+track_radius_*cos(-M_PI/2-i*M_PI/num),track_radius_*sin(-M_PI/2-i*M_PI/num)));
    }
}



void CameraSolver::generateCurlingSampleSet(int num)
{
    sample_set_.clear();
    double y=36+21, half_radius=3;
    int sign=-1;/*
    for(int i=0; i<2; i++){
        for(int j=0; j<2; j++){
            QVector<Vec3> tmp_point;
            addCircle(0,y*sign,half_radius*(j+1),num,tmp_point);
            multi_track_sample_set_.push_back(tmp_point);
        }
        sign*=-1;
    }*/
//    QVector<Vec3> main_track;
//    addRect(0,0,4.75,44.5,4,main_track);
//    printProjectedSample();
//    multi_track_sample_set_.push_back(main_track);
    addRect(0,0,4.75,44.5,num,sample_set_);
}

void CameraSolver::generateDistanceField(){
    Vec4i viewport= solver_camera_->get_viewport();
    int w=viewport[2],h=viewport[3];
    if(sample_dist_field_==NULL){
        sample_dist_field_=new DistField(w,h);
    }
    sample_dist_field_->resize(w,h);


}

void CameraSolver::printSample()
{
    qDebug()<<"sample set:";
    int num =sample_set_.size();
    for(int i=0; i<num; i++){
        qDebug()<<i<<":"<<sample_set_[i][0]<<sample_set_[i][1];
    }

}

void CameraSolver::printProjectedSample()
{
    qDebug()<<"projected sample set:";
    int num =projected_sample_set_[0].size();
    for(int i=0; i<num; i++){
        qDebug()<<i<<":"<<projected_sample_set_[0][i][0]<<projected_sample_set_[0][i][1];
    }
}

void CameraSolver::updateSampleDistField(){
//    sample_dist_field_->computeDistFromPolygon(projected_sample_set_);
    if(kdtree_dist_field_==NULL)
    {
        kdtree_dist_field_=new KDTreeDistField(projected_sample_set_[0]);
    }
    else{
        kdtree_dist_field_->setPointSet(projected_sample_set_[0]);
    }
}

void CameraSolver::updateMultiTrackSampleDistField()
{
    if(multi_boundary_dist_field_==NULL){
        multi_boundary_dist_field_=new MultiTrackKDTreeDistField();
    }
    multi_boundary_dist_field_->setPointSetList(multi_track_sample_set_);
}

void CameraSolver::updateUnprojectedData()
{
//    setCameraParams(params_);
    int num=data_set_.size();
    if(unprojected_data_set_.size()!=num)
        unprojected_data_set_.resize(num);
    for(int i=0; i<num; i++){
        unprojected_data_set_[i]=solver_camera_->unprojectToGround(data_set_[i]);
    }

}

void CameraSolver::updateProjectedSample(){

    int left_num = 3;
    int right_num = 2;

    int num=sample_set_.size();

    projected_sample_set_.resize(left_num + right_num + 1);
    for(int i = 0; i < projected_sample_set_.size(); ++i)
    {
        projected_sample_set_[i].resize(num);
    }

    double half_width = fabs(sample_set_[0][0]);

    for(int i = 1; i <= left_num; ++i)
    {
        QVector<Vec3> new_point_set = sample_set_;

        for(int j = 0; j < num; ++j)
        {
            new_point_set[j][0] += 2*i*half_width;
        }

        for(int ii=0; ii<num; ii++){
            Vec3 point=solver_camera_->projectToScreen(new_point_set[ii]);

            point[1]=solver_camera_->get_viewport()[3]-point[1];
            point[2]=1;
            projected_sample_set_[i][ii]=point;
        }
    }
    for(int i = 1; i <= right_num; ++i)
    {
        QVector<Vec3> new_point_set = sample_set_;

        for(int j = 0; j < num; ++j)
        {
            new_point_set[j][0] -= 2*i*half_width;
        }

        for(int ii=0; ii<num; ii++){
            Vec3 point=solver_camera_->projectToScreen(new_point_set[ii]);

            point[1]=solver_camera_->get_viewport()[3]-point[1];
            point[2]=1;
            projected_sample_set_[i+left_num][ii]=point;
        }
    }

    for(int i=0; i<num; i++){
        Vec3 point=solver_camera_->projectToScreen(sample_set_[i]);

        point[1]=solver_camera_->get_viewport()[3]-point[1];
        point[2]=1;
        projected_sample_set_[0][i]=point;
//        qDebug()<<"i:"<<i<<",to point:"<<point[0]<<point[1];
//        renderProjectedSamples();
    }
}

void CameraSolver::update2DProjectedSample()
{
    //左右暂时反过来设
    int left_num = 2;
    int right_num = 3;

    int num=sample_set_.size();

    projected_sample_set_.resize(left_num + right_num + 1);
    for(int i = 0; i < projected_sample_set_.size(); ++i)
    {
        projected_sample_set_[i].resize(num);
    }

    double half_width = fabs(sample_set_[0][0]);

    for(int i = 1; i <= left_num; ++i)
    {
        QVector<Vec3> new_point_set = sample_set_;

        for(int j = 0; j < num; ++j)
        {
            new_point_set[j][0] += 2*i*half_width;
        }

        for(int ii=0; ii<num; ii++){
            Vec3 origin_point=new_point_set[ii];
            origin_point[2]=1;
            Vec3 point=affine_matrix_*(origin_point);
            if(point[2]!=0)
                point=point/point[2];
            projected_sample_set_[i][ii]=point;
        }
    }
    for(int i = 1; i <= right_num; ++i)
    {
        QVector<Vec3> new_point_set = sample_set_;

        for(int j = 0; j < num; ++j)
        {
            new_point_set[j][0] -= 2*i*half_width;
        }

        for(int ii=0; ii<num; ii++){
            Vec3 origin_point=new_point_set[ii];
            origin_point[2]=1;
            Vec3 point=affine_matrix_*(origin_point);
            if(point[2]!=0)
                point=point/point[2];
            projected_sample_set_[i+left_num][ii]=point;
        }
    }

    for(int i=0; i<num; i++){
        Vec3 origin_point=sample_set_[i];
        origin_point[2]=1;
        Vec3 point=affine_matrix_*(origin_point);
        if(point[2]!=0)
            point=point/point[2];
        projected_sample_set_[0][i]=point;
//        qDebug()<<"i:"<<i<<",to point:"<<point[0]<<point[1];
//        renderProjectedSamples();
    }
}

void CameraSolver::updateProjectedMultiTrackSample()
{

    int num=multi_track_sample_set_.size();
    if(projected_multi_track_sample_set_.size()!=num)
        projected_multi_track_sample_set_.resize(num);

    for(int i=0; i< num; i++){
        int num0=multi_track_sample_set_[i].size();
        if(projected_multi_track_sample_set_[i].size()!=num0)
            projected_multi_track_sample_set_[i].resize(num0);
        for(int j=0; j<num0; j++){
            Vec3 point= solver_camera_->projectToScreen(multi_track_sample_set_[i][j]);
            point[1]=solver_camera_->get_viewport()[3]-point[1];
            point[2]=1;
            projected_multi_track_sample_set_[i][j]=point;
        }
    }
}

void CameraSolver::renderProjectedSamples(){

}



double CameraSolver::pointSetMatchError(QVector<Vec3> &target_set,QVector<Vec3> &input_set){

    double result=0;
    int m=input_set.size(), n=target_set.size();
    if(m<=0 || n<=0 )return 0;
    QHash<int,double> dist_graph;
    //make sure n>m

    double min_dist=(input_set[0]-target_set[0]).sqrnorm();
    int min_i=0,min_j=0;
    for(int i=0; i<m; i++){
        for(int j=0; j<n; j++){
            double dist=(input_set[i]-target_set[j]).sqrnorm();
            dist_graph.insert(i*10000+m+j,dist);
            if(dist<min_dist){
                min_i=i;
                min_j=j;
                min_dist=dist;
            }
        }
    }
    int curr_i=min_i, curr_j=min_j;
    QMultiHash<int,int> point_match_hash;
    point_match_hash.insertMulti(curr_i,curr_j);

    QHash<int, int> target_belong_hash;


    for(int j=0; j<n; j++)
    {
        double min_dist_j=dist_graph[0+m+j];
        for(int i=0; i<m; i++){
            if(dist_graph[i*10000+m+j]<min_dist_j){
                min_dist_j=dist_graph[i*10000+m+j];
                target_belong_hash[j]=i;
            }
        }
    }
//    qDebug()<<"min j,i:"<<min_j<<min_i;


    for(int j=0; j<n; j++){
        double dist=dist_graph[10000*target_belong_hash[j]+m+j];
//        qDebug()<<"matching j,i:"<<j<<target_belong_hash[j];
//        qDebug()<<"target point j:"<<target_set[j][0]<<target_set[j][1]<<",dist to input i:"<<input_set[target_belong_hash[j]][0]<<input_set[target_belong_hash[j]][1]<<",dist:"<<dist;
        result+=dist;
    }
    return result;
}

double CameraSolver::pointSetToDistField(QVector<Vec3> &target_set){
    double result=0;
    int num=target_set.size();
    for(int i=0; i<num; i++){
        int x=target_set[i][0], y=target_set[i][1];
        double value=kdtree_dist_field_->getDistance(target_set[i]);//sample_dist_field_->getValue(x,y);//
//        qDebug()<<"get test value:"<<test_value;
        if(value>10e4)
        {
//            qDebug()<<"out of range point:"<<x<<y;
//            qDebug()<<"nearby points:"<<sample_dist_field_->getValue(x-1,y-1)<<sample_dist_field_->getValue(x-1,y)<<sample_dist_field_->getValue(x-1,y+1)<<sample_dist_field_->getValue(x,y+1)<<sample_dist_field_->getValue(x+1,y+1)<<sample_dist_field_->getValue(x+1,y)<<sample_dist_field_->getValue(x+1,y-1);
        }

        if(value > 2500)
        {
            result+= 10 * value;
        }
        else
        {
            result+=value;
        }
    }
    return result;
}

double CameraSolver::distPointToPointSet(Vec3 P, QVector<Vec3> &set2)//2D error on screen
{
    P[2]=0;
    int num=set2.size();
    double result=(P-set2[0]).norm();
    for(int i=1; i<num; i++){
        double dist=(P-set2[i]).norm();
        if(dist<result){
            result=dist;
        }
    }
    return result;
}
double CameraSolver::distPointToTrack(Vec3 P,double radius, double length) // 3D error on ground
{
    P[2]=0;
    double x=P[0], y=P[1];
    int signX=x>0?1:-1, signY=y>0?1:-1;
    double result=fmin(fabs(y-signY*radius),fabs(sqrt(pow(x-signX*length/2,2)+pow(y,2))-radius));
    return result;
}
Vec3 CameraSolver::trackPoint(double radius, double length,  double x, int signY){
    double x_abs=fabs(x);
    int signX=x>0?1:-1;
    if( x_abs>length/2 && x_abs<=radius+length/2){
        return Vec3(x,signY*sqrt(pow(radius,2)-pow(x-signX*length/2,2)),0);
    }
    else if(x_abs<=length/2){
        return Vec3(x,signY*radius,0);
    }
    else{
        return Vec3(-1,-1,-1);
    }
}

double CameraSolver::unprojectError(Vec3 P){
    Vec3 unproject_point=solver_camera_->unprojectToGround(P);
    return distPointToTrack(unproject_point,track_radius_,track_length_);

}
double CameraSolver::projectError(Vec3 P){
    Vec3 projected_point=solver_camera_->projectToScreen(P);
    return distPointToPointSet(projected_point,data_set_);
}

double CameraSolver::avgUnprojectError()
{
    updateUnprojectedData();
    double result=pointSetMatchError(sample_set_,unprojected_data_set_);
    return result/sample_set_.size();
}

double CameraSolver::avgProjectError(int track_type)
{
    if(track_type==0){
        updateProjectedSample();
        updateSampleDistField();
        double result=pointSetToDistField(data_set_);//pointSetMatchError(data_set_,projected_sample_set_);
        return result/data_set_.size();

    }
    else if (track_type==1) {
        updateProjectedMultiTrackSample();
        updateSampleDistField();
        double result=pointSetToDistField(data_set_);
        return result/data_set_.size();
    }
}

double CameraSolver::avg2DProjectError(int bound_mode)
{
    update2DProjectedSample();
    updateSampleDistField();
    double result=pointSetToDistField(data_set_) / data_set_.size();//pointSetMatchError(data_set_,projected_sample_set_);

    if(bound_mode == 0)
    {
        for(int i = 0; i < projected_sample_set_[0].size(); ++i)
        {
            double min_dist = 10000;

            for(int j = 0; j < data_set_.size(); ++j)
            {
                double current_dist = (projected_sample_set_[0][i] - data_set_[j]).norm();

                if(current_dist < min_dist)
                {
                    min_dist = current_dist;
                }
            }

            if(min_dist > 50)
            {
                result += 20 * min_dist;
            }
            else
            {
                result += 2 * min_dist;
            }
        }
    }
    else if(bound_mode == 1)
    {
        for(int i = 0; i < projected_sample_set_[0].size(); ++i)
        {
            double min_dist = 100000000;

            for(int j = 0; j < data_set_.size(); ++j)
            {
                double current_dist = (projected_sample_set_[0][i] - data_set_[j]).sqrnorm();

                if(current_dist < min_dist)
                {
                    min_dist = current_dist;
                }
            }

            if(min_dist > 2500)
            {
                result += 10 * min_dist;
            }
            else
            {
                result += min_dist;
            }
        }
    }

    return result;
}

Vec<9,double> CameraSolver::deltaAvgUnprojectError(double delta)
{
    Vec<9,double> result;
    for(int i=0; i<9; i++){
        double f=avgUnprojectError();
        Vec<9,double> delta_vector;
        delta_vector[i]=delta;
        setCameraParams(params_+delta_vector);
        double f_delta=avgUnprojectError();
        result[i]=(f_delta-f)/delta;
//        qDebug()<<"f,fdelta:"<<f<<f_delta;
        setCameraParams(params_);
    }
    return result;
}

Vec<9,double> CameraSolver::deltaAvgProjectError(double delta)
{
    Vec<9,double> result;
    for(int i=0; i<9; i++){
//        if(i==8) continue;
        setCameraParams(params_);
        double f=avgProjectError();

        Vec<9,double> delta_vector;
        delta_vector[i]=delta;
        setCameraParams(params_+delta_vector);
        double f_delta=avgProjectError();

        result[i]=(f_delta-f)/delta;
    }
    setCameraParams(params_);
    return result;
}

Vec<8, double> CameraSolver::delta2DAvgProjectError(double delta, int mode, int bound_mode)
{
    //mode: 0:all 1:平移 2:旋转 3:X放缩 4:Y放缩 5:畸变1 6:畸变2 7:除扭曲
    if(mode == 0)
    {
        Vec<8,double> result;
        for(int i=0; i<8; i++){

            set2DParam(affine_params_);
            double f=avg2DProjectError(bound_mode);

            Vec<8,double> delta_vector;
            delta_vector[i]=delta;
            set2DParam(affine_params_+delta_vector);
            double f_delta=avg2DProjectError(bound_mode);

            result[i]=(f_delta-f)/delta;
        }

        set2DParam(affine_params_);
        return result;
    }
    if(mode == 1)
    {
        Vec<8,double> result;
        for(int i=4; i<6; i++){

            set2DParam(affine_params_);
            double f=avg2DProjectError(bound_mode);

            Vec<8,double> delta_vector;
            delta_vector[i]=delta;
            set2DParam(affine_params_+delta_vector);
            double f_delta=avg2DProjectError(bound_mode);

            result[i]=(f_delta-f)/delta;
        }

        set2DParam(affine_params_);
        return result;
    }
    if(mode == 2)
    {
        Vec<8,double> result;

        double theta = 1000*delta;

        set2DParam(affine_params_);
        double f=avg2DProjectError(bound_mode);

        Vec<8,double> new_params_;
        for(int i = 0; i < new_params_.size(); ++i)
        {
            new_params_[i] = affine_params_[i];
        }

        new_params_[0] = affine_params_[0]*cos(theta) + affine_params_[1]*sin(theta);
        new_params_[1] = affine_params_[1]*cos(theta) - affine_params_[0]*sin(theta);
        new_params_[2] = affine_params_[2]*cos(theta) - affine_params_[3]*sin(theta);
        new_params_[3] = affine_params_[3]*cos(theta) - affine_params_[2]*sin(theta);

        set2DParam(new_params_);
        double f_delta=avg2DProjectError(bound_mode);

        if(f_delta < f)
        {
            for(int i = 0; i < 4; ++i)
            {
                result[i] = new_params_[i] - affine_params_[i];
            }
        }
        else
        {
            new_params_[0] = affine_params_[0]*cos(-theta) + affine_params_[1]*sin(-theta);
            new_params_[1] = affine_params_[1]*cos(-theta) - affine_params_[0]*sin(-theta);
            new_params_[2] = affine_params_[2]*cos(-theta) - affine_params_[3]*sin(-theta);
            new_params_[3] = affine_params_[3]*cos(-theta) - affine_params_[2]*sin(-theta);

            for(int i = 0; i < 4; ++i)
            {
                result[i] = new_params_[i] - affine_params_[i];
            }
        }

        set2DParam(affine_params_);
        return result;
    }
    if(mode == 3)
    {
        Vec<8,double> result;

        set2DParam(affine_params_);
        double f=avg2DProjectError(bound_mode);

        Vec<8,double> new_params_;
        for(int i = 0; i < new_params_.size(); ++i)
        {
            new_params_[i] = affine_params_[i];
        }

        new_params_[0] *= (1.0 + delta);
        new_params_[1] *= (1.0 + delta);

        set2DParam(new_params_);
        double f_delta=avg2DProjectError(bound_mode);

        if(f_delta < f)
        {
            for(int i = 0; i < 2; ++i)
            {
                result[i] = new_params_[i] - affine_params_[i];
            }
        }
        else
        {
            for(int i = 0; i < 2; ++i)
            {
                result[i] = affine_params_[i] - new_params_[i];
            }
        }

        set2DParam(affine_params_);
        return result;
    }
    if(mode == 4)
    {
        Vec<8,double> result;

        set2DParam(affine_params_);
        double f=avg2DProjectError(bound_mode);

        Vec<8,double> new_params_;
        for(int i = 0; i < new_params_.size(); ++i)
        {
            new_params_[i] = affine_params_[i];
        }

        new_params_[2] *= (1.0 + delta);
        new_params_[3] *= (1.0 + delta);

        set2DParam(new_params_);
        double f_delta=avg2DProjectError(bound_mode);

        if(f_delta < f)
        {
            for(int i = 2; i < 4; ++i)
            {
                result[i] = new_params_[i] - affine_params_[i];
            }
        }
        else
        {
            for(int i = 2; i < 4; ++i)
            {
                result[i] = affine_params_[i] - new_params_[i];
            }
        }

        set2DParam(affine_params_);
        return result;
    }
    if(mode == 5)
    {
        Vec<8,double> result;

        for(int ii = 6; ii < 8; ++ii)
        {
            set2DParam(affine_params_);
            double f=avg2DProjectError(bound_mode);

            Vec<8,double> new_params_;
            for(int i = 0; i < new_params_.size(); ++i)
            {
                new_params_[i] = affine_params_[i];
            }
            new_params_[ii] += delta;

            set2DParam(new_params_);
            double f_delta=avg2DProjectError(bound_mode);

            result[ii]=(f_delta-f)/delta;
        }

        set2DParam(affine_params_);
        return result;
    }
    if(mode == 6)
    {
        Vec<8,double> result;

        for(int ii = 7; ii < 8; ++ii)
        {
            set2DParam(affine_params_);
            double f=avg2DProjectError(bound_mode);

            Vec<8,double> new_params_;
            for(int i = 0; i < new_params_.size(); ++i)
            {
                new_params_[i] = affine_params_[i];
            }
            new_params_[ii] += delta;

            set2DParam(new_params_);
            double f_delta=avg2DProjectError(bound_mode);

            result[ii]=(f_delta-f)/delta;
        }

        set2DParam(affine_params_);
        return result;
    }
    if(mode == 7)
    {
        Vec<8,double> result;
        for(int i=0; i<6; i++){

            set2DParam(affine_params_);
            double f=avg2DProjectError(bound_mode);

            Vec<8,double> delta_vector;
            delta_vector[i]=delta;
            set2DParam(affine_params_+delta_vector);
            double f_delta=avg2DProjectError(bound_mode);

            result[i]=(f_delta-f)/delta;
        }

        set2DParam(affine_params_);
        return result;
    }
}

void CameraSolver::steepestDescentMethod(int error_type){
//    generateSampleSet(20);
//    generateDistanceField();
    generateUniformSampleSet(20);
    updateProjectedSample();
    printProjectedSample();
    std::cout<<"initial param:";
    for(int i=0; i<9; i++){
        std::cout<<params_[i]<<",";
    }
    std::cout<<std::endl;
    Vec<9,double> default_param=params_;
    double error;
    if(error_type==0){
        error=avgProjectError();
    }
    else if(error_type==1){
        error=avgUnprojectError();
    }

    int max_iter=2000;
    int iter=0;
    double step=0.08;
    double rate=1.3;
    double len=100;
    Vec<9,double> dir;
    std::cout<<"error:"<<error<<std::endl;

    while (error>10 && len>1e-1 && iter<max_iter){
        step=0.08;

        if(error_type==0){
            dir=-deltaAvgProjectError(1e-5);
        }
        else if(error_type==1){
            dir=-deltaAvgUnprojectError(1e-5);
        }
        len=dir.norm();
//        std::cout<<"gradient length:"<<len<<std::endl;
        dir=(1/len)*dir;

        if(len<ZERO_TOLERANCE)
        {
            qDebug()<<"has 0 delta";
            std::cout<<"finished with:";
            for(int i=0; i<9; i++){
                std::cout<<params_[i]<<",";
            }
            std::cout<<std::endl;
            setCameraParams(params_);
            return;
        }
        while(step>1e-5){
            iter++;
            setCameraParams(params_+step*dir);
            double current_error;
            if(error_type==0){
                current_error=avgProjectError();
            }
            else if(error_type==1){
                current_error=avgUnprojectError();
            }
            if( current_error<error){
                std::cout<<"dir:";
                for(int i=0; i<9; i++){
                    std::cout<<dir[i]<<",";
                }
                std::cout<<std::endl;
                qDebug()<<"error decreased:"<<error<<",step:"<<step;
                params_=params_+step*dir;
                error=current_error;
                break;
            }
            else{
                step/=rate;
            }
        }

        if(iter%100>95){
            emit currentError(sqrt(error));
        }
    }
    std::cout<<"finished with:";
    for(int i=0; i<9; i++){
        std::cout<<params_[i]<<",";
    }
    std::cout<<std::endl;
    std::cout<<"initial param:";
    for(int i=0; i<9; i++){
        std::cout<<default_param[i]<<",";
    }
    std::cout<<std::endl;
    setCameraParams(params_);
    emit currentError(sqrt(error));
    printProjectedSample();
}

void CameraSolver::steepestDescentMethod2D()
{
    generateCurlingSampleSet(1);
    printSample();

    if(!have_init)
    {
        have_init = true;

        if(data_set_.size() > 0)
        {
            int average_x = 0, average_y = 0;

            for(int i = 0; i < data_set_.size(); ++i)
            {
                average_x += data_set_[i][0];
                average_y += data_set_[i][1];
            }
            average_x /= data_set_.size();
            average_y /= data_set_.size();

            affine_params_[4] = average_x;
            affine_params_[5] = average_y;
        }

        if(data_set_.size() > 1)
        {
            int max_dist_point_idx_1 = 0;
            int max_dist_point_idx_2 = 1;

            double max_dist = 0;

            for(int i = 0; i < data_set_.size() - 1; ++i)
            {
                for(int j = i + 1; j < data_set_.size(); ++j)
                {
                    double current_dist = (data_set_[i] - data_set_[j]).norm();

                    if(current_dist > max_dist)
                    {
                        max_dist = current_dist;

                        max_dist_point_idx_1 = i;
                        max_dist_point_idx_2 = j;
                    }
                }
            }

            Vec3 point;

            if(data_set_[max_dist_point_idx_1][0] - data_set_[max_dist_point_idx_2][0] < 0)
            {
                point = (data_set_[max_dist_point_idx_1] - data_set_[max_dist_point_idx_2]) / 2;
            }
            else
            {
                point = (data_set_[max_dist_point_idx_2] - data_set_[max_dist_point_idx_1]) / 2;
            }

            Vec3 rect_init_pose;

            rect_init_pose[0] = 0;
            rect_init_pose[1] = 1;
            rect_init_pose[2] = 0;

            double point_norm = point.norm();

            double theta = acos(rect_init_pose*point / point_norm);
            std::cout << theta*180.0/3.14159265359 << " , " << point_norm << std::endl;

            affine_params_[0] = 1.0/22.5 * point_norm * cos(theta);
            affine_params_[1] = -1.0/22.5 * point_norm * sin(theta);
            affine_params_[2] = 5.46 * sin(theta);
            affine_params_[3] = 5.46 * cos(theta);
        }

        affine_params_[6] = 0;
        affine_params_[7] = 0;
    }

    set2DParam(affine_params_);
    update2DProjectedSample();
    printProjectedSample();

    std::cout<<"initial param:";
    for(int i=0; i<8; i++){
        std::cout<<affine_params_[i]<<",";
    }
    std::cout<<std::endl;
    Vec<8,double> default_param=affine_params_;
    double error;
    error=avg2DProjectError();

    int max_iter=20000;
    int iter=0;
    double step=0.08;
    double rate=0.8;
    double len=100;
    Vec<8,double> dir;
    std::cout<<"error:"<<error<<std::endl;

    //return;

    int run_time = 0;
    int max_run_time = 20000;

    for(int cycle = 0; cycle < 2; ++cycle)
    {
        iter=0;
        step=0.08;
        len=100;
        run_time = 0;

        while (error>10 && len>1e-1 && iter<max_iter)
        {
            bool result_changed = false;//--------

            step=1000/(cycle+1);
            dir=-delta2DAvgProjectError(1e-5, 7);
            for(int i = 5; i < 55; i+=5)
            {
                dir -= delta2DAvgProjectError(i*1e-5, 7);
            }

            len=dir.norm();
    //        std::cout<<"gradient length:"<<len<<std::endl;
            dir=(1.0/len)*dir;

            if(len<ZERO_TOLERANCE)
            {
                qDebug()<<"has 0 delta";
                std::cout<<"finished with:";
                for(int i=0; i<8; i++){
                    std::cout<<affine_params_[i]<<",";
                }
                std::cout<<std::endl;
                set2DParam(affine_params_);
                return;
            }

            double best_step = step;
            double min_error = error;

            while(step>1e-5)
            {
                iter++;
                set2DParam(affine_params_+step*dir);
                double current_error;
                current_error=avg2DProjectError();

                if(current_error < min_error)
                {
                    min_error = current_error;
                    best_step = step;

                    if(min_error / error < 0.9)
                    {
                        break;
                    }
                }

                step*=rate;
            }

            if( min_error<error){

                error = min_error;

                if(run_time < max_run_time)//--------
                {
                    ++run_time;
                }
                else//--------
                {
                    result_changed = true;
                }

                std::cout<<"normal dir:";
                for(int i=0; i<8; i++){
                    std::cout<<dir[i]<<",";
                }
                std::cout<<std::endl;
                qDebug()<<"error decreased:"<<error<<",step:"<<step;
                affine_params_=affine_params_+best_step*dir;
            }

            if(result_changed)//--------
            {
                break;
            }

            if(iter%100>95){
                emit currentError(sqrt(error));
            }
        }
        continue;

        iter=0;
        step=0.08;
        len=100;
        run_time = 0;

        while (error>10 && len>1e-1 && iter<max_iter)
        {
            bool result_changed = false;//--------

            step=1000/(cycle+1);
            dir=-delta2DAvgProjectError(1e-5, 5, cycle);
            for(int i = 5; i < 55; i+=5)
            {
                dir -= delta2DAvgProjectError(i*1e-5, 5, cycle);
            }

            len=dir.norm();
    //        std::cout<<"gradient length:"<<len<<std::endl;
            dir=(1.0/len)*dir;

            if(len<ZERO_TOLERANCE)
            {
                qDebug()<<"has 0 delta";
                std::cout<<"finished with:";
                for(int i=0; i<8; i++){
                    std::cout<<affine_params_[i]<<",";
                }
                std::cout<<std::endl;
                set2DParam(affine_params_);
                return;
            }

            double best_step = step;
            double min_error = error;

            while(step>1e-20)
            {
                iter++;
                set2DParam(affine_params_+step*dir);
                double current_error;
                current_error=avg2DProjectError();

                if(current_error < min_error)
                {
                    min_error = current_error;
                    best_step = step;

                    if(min_error / error < 0.9)
                    {
                        break;
                    }
                }

                step*=rate;
            }

            if( min_error<error){

                error = min_error;

                if(run_time < max_run_time)//--------
                {
                    ++run_time;
                }
                else//--------
                {
                    result_changed = true;
                }

                std::cout<<"dir:";
                for(int i=0; i<8; i++){
                    std::cout<<dir[i]<<",";
                }
                std::cout<<std::endl;
                qDebug()<<"error decreased:"<<error<<",step:"<<step;
                affine_params_=affine_params_+best_step*dir;
            }

            if(result_changed)//--------
            {
                break;
            }

            if(iter%100>95){
                emit currentError(sqrt(error));
            }
        }
    }

    std::cout<<"finished with:";
    for(int i=0; i<8; i++){
        std::cout<<affine_params_[i]<<",";
    }
    std::cout<<std::endl;
    std::cout<<"initial param:";
    for(int i=0; i<8; i++){
        std::cout<<default_param[i]<<",";
    }
    std::cout<<std::endl;
    set2DParam(affine_params_);

    emit currentError(sqrt(error));
    printProjectedSample();
}

void CameraSolver::init_param()
{
    have_init = false;
    step_now = 100;
}
