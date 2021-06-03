#include "imgRenderLabel.h"
#include <QPainter>
#include <QDebug>
#include <QImage>
#include <QPixmap>
#include <QPolygonF>
#include <QPainterPath>
#include <QMouseEvent>

ImgRenderLabel::ImgRenderLabel(QWidget *parent) : QWidget(parent)
{
    qDebug()<<"begin create fitting";

    projected_point_list_.clear();
}

void ImgRenderLabel::slotLoadImage(QString filename){
    image_.load(filename);
    qDebug()<<image_.size();
    temp_pixmap_=QPixmap::fromImage(image_);
    projected_point_list_.clear();
//    this->setPixmap(temp_pixmap_);
//    update();
}

void ImgRenderLabel::slotSetImage(QImage *img){
    image_=img->scaled(QSize(600,400),Qt::KeepAspectRatio);
    qDebug()<<image_.size();
    projected_point_list_.clear();
    this->setFixedSize(image_.size());
//    this->setPixmap(QPixmap::fromImage(*img));
    update();
}
void ImgRenderLabel::slotAddPoint(QPointF point){
    QPointF offset=QPointF(0,0);
//    if(!image_.isNull())
//        offset=QPointF(image_.width()/2,image_.height()/2);
    projected_point_list_.push_back(point-offset);
    update();
}

void ImgRenderLabel::slotPopPoint()
{
    projected_point_list_.pop_back();
    update();
}

void ImgRenderLabel::paintEvent(QPaintEvent *e){
    QPainter painter(this);
    if(!image_.isNull()){
        painter.drawImage(QPointF(0,0),image_);
//        painter.translate(image_.width()/2,image_.height()/2);
        painter.drawRect(rect_);
        painter.setPen(QPen(Qt::red,3));

        painter.drawPoints(QPolygonF(projected_point_list_));
    }

//    painter.translate(-image_.width()/2,-image_.height()/2);



}

void ImgRenderLabel::mousePressEvent(QMouseEvent *e)
{
    if(operate_mode_==2){
        rect_.setTopLeft(e->pos());
        rect_.setBottomRight(e->pos());
        in_drag_=true;
    }
}

void ImgRenderLabel::mouseMoveEvent(QMouseEvent *e)
{
    if(in_drag_){
        rect_.setBottomRight(e->pos());
        update();
    }
}


void ImgRenderLabel::slotSetData(QVector<QPointF> &origin_point_list){

}

void ImgRenderLabel::slotSetAddRectangleMode()
{
    qDebug()<<"has switch to rect mode";
    operate_mode_=2;
}

void ImgRenderLabel::slotSetAddPointMode()
{
    operate_mode_=1;
}


QSize ImgRenderLabel::slotImageSize(){
    //qDebug()<<"image_size:"<<image_.size();
    return image_.size();
}

void ImgRenderLabel::mouseReleaseEvent(QMouseEvent *e){
    if(operate_mode_==1){
        slotAddPoint(QPointF(e->pos()));
        emit signalAddPoint(e->pos().x(),e->pos().y());
    }
    else if(operate_mode_==2){
        in_drag_=false;
    }
}
