#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QLayout>
#include <QVector>
#include <QTableWidget>

#include "Math/MathDefines.h"
#include "imgRenderLabel.h"

#include "Algorithm/FindBound.h"
#include <direct.h>
#include <io.h>

namespace GCL {
    class RenderWidget;
    class ModelScene;
    class Camera;
}
class CameraSolver;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
    /**
     * @brief initMenu      init open image and load\save data menu
     */
    void initMenu();

    /**
     * @brief initRenderWidgets init 2D/3D render widgets/Scenes
     */
    void initRenderWidgets();
    /**
     * @brief addRotateShortCut  add shortcut ADSW for rotate left/right/down/up QE for Clockwise/counterClockwise
     */
    void addRotateShortCut();
    /**
     * @brief initDownWidget    widget on the bottom show points marked on 2D scene and params of the 3D
     * scene
     */
    void initDownWidget();
    /**
     * @brief initParamWidget
     * 1.simplified params,Rx,Ry,Rz,Rw: quanion  represent Rotation, d:in opengl distance between camera pos&center
     * Tx,Ty,Tz offset applied on both center and pos, f:used to control focal length,in GCL::Camera set_viewport
     * 2.camera matrix  --modelview matrix; intrinsisc matrix--projection matrix
     * 3. f shown again
     * 4.track params and solve button
     *
     */
    void initParamWidget();

    void drawDistField();

    void setDataset(QVector<QPointF> &point_set);

    void updateDataWidget();
public slots:
//    void slotOpenVideo();

    void slotOpenImage();

    void slotOpenVideo();

    /**
     * @brief slotAddPoint  receive addpoint signal,record coord, generate spin box to recoord data_set_'s unrpoject coord;
     * @param x
     * @param y
     */
    void slotAddPoint(int x, int y);


    void slotPopPoint();

    /**
     * @brief slotSolveCameraBySteepestSescent
     */
    void slotSolveCameraBySteepestDescent();

    void getFiles( std::string path, std::vector<std::string>& files );

    void slotLoadImageAndData(QString filename, bool load_point=true, bool load_camera=true, bool update_data_widget=true);

    void slotLoadPointDataFromFile(QString filename);

    void slotLoadCameraParamsFromFile(QString filename);

    void slotSavePointDataToFile(QString filename);

    void slotSaveCameraParamsToFile(QString filename);

    void slotLoadVideo(QString filename);

    void slotSolveVideoFrame(QString filename);

    void slotSavePointData();

    void slotSaveCameraParams();

    /**
     * @brief slotSetCameraParams       set camera params from current values in param_table widget
     */
    void slotSetCameraParams();

    void slotLoadCameraParams(GCL::Vec<9,double> param);

    void slotUpateCameraParams();

    void slotSetTrackParam();

    void slotFindBound();

    void slotLoadNextFrameAndRun();

    void slotProcessFrames();

    void slotProcessFramesAndSave();

private:
    QVBoxLayout*        main_layout_;

    ImgRenderLabel*     img_render_label_;
    GCL::ModelScene*    model_scene_;
    GCL::Camera*        camera_;
    CameraSolver*       camera_solver_;
    GCL::RenderWidget*  render_widget_;

    QWidget*            bottom_control_widget_;
    QHBoxLayout*        bottom_control_layout_;
    QVBoxLayout*        point_layout_;
    QWidget*            point_widget_;

    QTableWidget*   params_table_;
    QTableWidget*   camera_matrix_table_;
    QTableWidget*   intrinsics_matrix_table_;
    QTableWidget*   focal_length_table_;

    QDoubleSpinBox* error_box;
    QTableWidget*   track_params_table_;

    QVector<QDoubleSpinBox*> u_spinbox_list_;
    QVector<QDoubleSpinBox*> v_spinbox_list_;
    QVector<QLabel*>    data_set_label_;
    QVector<QPointF>    data_set_; //screen feature points

    QImage          render_image_=QImage{};
    QString         image_filename_;
    QString         video_name_;
    bool            video_mode;
    FindBound*      findbound_;
    QFileInfoList   frame_list_;
    int             current_frame_index_;
    int             max_frame_index_;
    bool            save_camera_param_=false;

    QSize           target_image_size;

protected:

signals:
    void signalFinishProcessFrame();

};

#endif // MAINWINDOW_H
