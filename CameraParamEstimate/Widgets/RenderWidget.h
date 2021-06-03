#ifndef MAGICCUBE_RENDERWIDGET_H
#define MAGICCUBE_RENDERWIDGET_H

#include <QOpenGLWidget>
#include <memory>
#include "../Math/MathDefines.h"

class QLineEdit;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QOpenGLContext;

namespace GCL {
class Scene;
class ModelScene;
/**
 * @brief      场景渲染类
 * 
 * 基于给定Scene场景类添加了细节设置、增加信息、update（）功能，Scene内部保持原样
 * 
 * 
 */
class RenderWidget : public QOpenGLWidget
{
    Q_OBJECT
public:

    /**
     * @brief      构造函数
     *
     * @param      scene   场景（内含）
     * @param      parent  父Widget类
     */
    explicit RenderWidget(GCL::Scene *scene,  QWidget *parent = 0);
    ~RenderWidget();
   //QProcess* get_process(){return slicing_process_;}

protected:

    void initializeGL();

    void resizeGL(int w, int h);

    /**
     * @brief      渲染
     * 
     * -# 场景渲染
     * -# 设置边界
     * -# 设置信息
     */
    void paintGL();

    
    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);

    void wheelEvent(QWheelEvent *e);

    void timerEvent(QTimerEvent *);

    //void dragEnterEvent(QDragEnterEvent*);

public:
    void mouseMoveEvent(QMouseEvent *e);
    void mouseDoubleClickEvent(QMouseEvent* e);
signals:
    void mousePressed();

public slots:

    /**
     * @brief      update
     */
    void slot_update();
    void slot_changeText(const QString& text);
    void keyReleaseEvent(QKeyEvent* e);



    void loadTargetImage(QImage &image);
private:
    void setupWidget();

    GCL::ModelScene *scene_;
    QString info_text_;

    QOpenGLContext* m_context_;
};
}
#endif // RENDERWIDGET_H
