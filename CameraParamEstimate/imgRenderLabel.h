#ifndef IMGRRENDERLABEL_H
#define IMGRRENDERLABEL_H

#include <QWidget>
#include <QLabel>
#include <QPixmap>
#include <QImage>
#include <QVector>
#include <QPointF>


class ImgRenderLabel : public QWidget
{
    Q_OBJECT
public:
    explicit ImgRenderLabel(QWidget *parent = nullptr);

    QRectF getTargetRectF(){return rect_;}
private:
    QImage image_=QImage{};
    QPixmap temp_pixmap_;
    QVector<QPointF> projected_point_list_;

    int operate_mode_=1;// 0: none, 1:add point 2:add rectF
    bool in_drag_=false;
    QRectF rect_=QRectF(0,0,0,0);
signals:
    void signalAddPoint(int x, int y);
public slots:
    void slotLoadImage(QString filename);
    void slotSetImage(QImage* img);
    void slotAddPoint(QPointF point);
    void slotPopPoint();
    void slotSetData(QVector<QPointF> &origin_point_list);
    void slotSetAddRectangleMode();
    void slotSetAddPointMode();
    QSize slotImageSize();
protected:
    void paintEvent(QPaintEvent* e);
    void mousePressEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent* e);
    void mouseReleaseEvent(QMouseEvent*);
public:
    QVector<QPointF> &projectedPointList(){return projected_point_list_;}
};

#endif // IMGRRENDERLABEL_H
