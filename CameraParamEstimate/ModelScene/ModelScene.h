#ifndef MODELSCENE_H
#define MODELSCENE_H

#include <QObject>
#include <QOpenGLFunctions>
#include <QVector>
#include "../Math/MathDefines.h"
#include "Scene/Scene.h"
class QOpenGLTexture;
class QOpenGLContext;
class QImage;
namespace GCL {

    class ModelScene: public Scene, public QObject
    {
        Q_OBJECT
    public:
        ModelScene();
    private:
        QOpenGLTexture* target_image_=NULL;
        QOpenGLContext *m_context;
        GLuint    texture_;
        double track_radius_;
        double track_length_;
        double track_width_;
        int track_num_;

        QVector<QVector<Vec3>> point_set_;
    public slots:
        void slotSetTrack(double radius, double length, double width, int num){track_radius_=radius; track_length_=length; track_width_=width; track_num_=num;}
        void slotSetFocalLength(double focal_length);
        void slotRotateLeft();
        void slotRotateRight();
        void slotRotateDown();
        void slotRotateUp();
        void slotRotateClockWise();
        void slotRotateCounterClockWise();
    signals:
        void signalCameraCenterChange(double x,double y, double z);
    public:
        void setContext(QOpenGLContext *context);
        void setColor(GLfloat* color);
        void setPointSet(QVector<QVector<Vec3>> &point_set);

        void renderObjects();
        void renderSingleTrack(double radius, double straight_length);
        void renderTracks();
        void renderTargetImage();

        void renderSampleSet(QVector<QVector<Vec3>> &point_set);

        void renderCircle(double x, double y, double r, int num=36);
        void renderQuad(double x, double y, double width, double height);
        void renderCurling();
    public:
        void loadTargetImage(QImage &temp_image);
        HomoMatrix4 getCameraRotation();
        Vec3    getCameraCenter();
    };

}


#endif // MODELSCENE_H
