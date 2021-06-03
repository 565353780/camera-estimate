#include "mainwindow.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <QDebug>
#include <QFileDialog>
#include <QMenu>
#include <QMenuBar>
#include <QMouseEvent>
#include <QToolButton>
#include <QPainter>
#include <QHeaderView>
#include <QPushButton>
#include <QTextStream>
#include <QRegExp>
#include <QtOpenGL>
#include <QOpenGLContext>
#include <QTime>
#include <QTimer>

#include "Widgets/RenderWidget.h"
#include "ModelScene/ModelScene.h"
#include "Scene/Camera.h"
#include "Algorithm/CameraSolver.h"
#include "Algorithm/DistField.h"

#include <QScrollArea>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    main_layout_=new QVBoxLayout;
    main_layout_->setContentsMargins(0,0,0,0);
    initMenu();
    initRenderWidgets();
    camera_solver_=new CameraSolver(model_scene_->getCamera());

    addRotateShortCut();
    initDownWidget();



    qDebug()<<"finish init UI";
    QWidget* central_widget=new QWidget(this);
    central_widget->setLayout(main_layout_);
    this->setCentralWidget(central_widget);
    this->resize(800,600);

    data_set_.clear();

    video_mode = false;


    findbound_= new FindBound();
}

MainWindow::~MainWindow()
{

}

void MainWindow::initMenu(){
    QMenuBar* menu_bar= new QMenuBar(this);
    QMenu* file_menu=new QMenu(tr("file"),this);
//    QAction* openAction= new QAction(tr("Open"),this);
//    connect(openAction,&QAction::triggered,this,&MainWindow::slotOpenVideo);
//    menu->addAction(openAction);

    QAction* loadImageAction= new QAction(tr("image"),this);
    connect(loadImageAction,&QAction::triggered,this,&MainWindow::slotOpenImage);


    QAction* loadVideoAction= new QAction(tr("video"),this);
    connect(loadVideoAction,&QAction::triggered,this,&MainWindow::slotOpenVideo);

    QMenu* data_menu=new QMenu(tr("data"));
    QAction* save_point_set=new QAction(tr("Save Points"),this);
    connect(save_point_set,&QAction::triggered,this,&MainWindow::slotSavePointData);
    save_point_set->setShortcut(QKeySequence("Ctrl+S"));
    QAction* save_camera_param=new QAction(tr("Save Camera Params"),this);
    connect(save_camera_param,&QAction::triggered,this,&MainWindow::slotSaveCameraParams);

    QMenu* operation_menu= new QMenu(tr("operation"));
    QAction* find_bound_action=new QAction(tr("Find Boundary"),this);
    connect(find_bound_action,&QAction::triggered,this,&MainWindow::slotFindBound);
    find_bound_action->setShortcut(Qt::Key_B);

    QAction* pop_action=new QAction(tr("pop point"),this);
    connect(pop_action,&QAction::triggered,this,&MainWindow::slotPopPoint);
    pop_action->setShortcut(QKeySequence("Ctrl+Z"));

    QAction* process_frames= new QAction(tr("Solve frames"),this);
    connect(process_frames,&QAction::triggered, this, &MainWindow::slotProcessFrames);
    connect(this,&MainWindow::signalFinishProcessFrame,this,&MainWindow::slotLoadNextFrameAndRun);

    QAction* process_frames_and_save= new QAction(tr("Solve and Save"),this);
    connect(process_frames_and_save,&QAction::triggered,this,&MainWindow::slotProcessFramesAndSave);


    file_menu->addAction(loadImageAction);
    file_menu->addAction(loadVideoAction);

    operation_menu->addAction(find_bound_action);
    operation_menu->addAction(pop_action);
    operation_menu->addAction(process_frames);
    operation_menu->addAction(process_frames_and_save);

    data_menu->addAction(save_point_set);
    data_menu->addAction(save_camera_param);

    menu_bar->addMenu(file_menu);
    menu_bar->addMenu(data_menu);
    menu_bar->addMenu(operation_menu);

    this->setMenuBar(menu_bar);
}

void MainWindow::initRenderWidgets(){
    img_render_label_= new ImgRenderLabel(this);
    img_render_label_->setFixedSize(600,400);
    connect(img_render_label_,&ImgRenderLabel::signalAddPoint,this,&MainWindow::slotAddPoint);

    model_scene_= new GCL::ModelScene();
    camera_=model_scene_->getCamera();
    render_widget_=new GCL::RenderWidget(model_scene_,this);
    render_widget_->setFixedSize(600,400);

    QHBoxLayout *hlayout=new QHBoxLayout;
    hlayout->addWidget(img_render_label_);
    hlayout->addWidget(render_widget_);

    main_layout_->addLayout(hlayout);

    model_scene_->slotSetTrack(8,28.85,3,4);
    QVector<QVector<Vec3>> test_point_set;
    test_point_set.resize(1);
    for(int i=0; i<4; i++)
        test_point_set[0].push_back(Vec3((i/2+1)*200,(i%2+1)*200));
    model_scene_->setPointSet(test_point_set);
    render_widget_->update();

    QAction* enter_add_point_mode= new QAction(this);
    connect(enter_add_point_mode,&QAction::triggered, img_render_label_,&ImgRenderLabel::slotSetAddPointMode);
    enter_add_point_mode->setShortcut(Qt::Key_P);

    QAction* enter_add_rect_mode= new QAction(this);
    connect(enter_add_rect_mode,&QAction::triggered, img_render_label_,&ImgRenderLabel::slotSetAddRectangleMode);
    enter_add_rect_mode->setShortcut(Qt::Key_R);

    this->addAction(enter_add_point_mode);
    this->addAction(enter_add_rect_mode);
}
void MainWindow::initDownWidget(){
    bottom_control_widget_=new QWidget(this);
    bottom_control_layout_=new QHBoxLayout;

    QScrollArea* scroll_area=new QScrollArea(this);

    point_widget_= new QWidget(this);
    point_layout_=new QVBoxLayout;

    bottom_control_layout_->addWidget(scroll_area);
    scroll_area->setWidget(point_widget_);

    initParamWidget();

    point_widget_->setLayout(point_layout_);
    bottom_control_widget_->setLayout(bottom_control_layout_);
    main_layout_->addWidget(bottom_control_widget_);

    scroll_area->setStyleSheet("background:white;");
    point_widget_->setStyleSheet("font-size:14px;");
}



void MainWindow::initParamWidget(){
    QVBoxLayout* vlayout=new QVBoxLayout;

    params_table_=new QTableWidget(this);
    params_table_->setRowCount(1);
    params_table_->setColumnCount(9);
    QStringList col_title;
    col_title<<"Rx"<<"Ry"<<"Rz"<<"Rw"<<"d"<<"Tx"<<"Ty"<<"Tz"<<"f";
    params_table_->setHorizontalHeaderLabels(col_title);

    for(int i=0; i<9 ; i++){
        QDoubleSpinBox * double_box=new QDoubleSpinBox(this);
        double_box->setRange(-10000,10000);
        double_box->setButtonSymbols(QAbstractSpinBox::NoButtons);
        params_table_->setCellWidget(0,i,double_box);
        params_table_->setColumnWidth(i,50);
    }
    params_table_->setFixedHeight(60);

    QPushButton* apply_button=new QPushButton(tr("Apply"),this);
    connect(apply_button,&QPushButton::clicked,this,&MainWindow::slotSetCameraParams);

    camera_matrix_table_=new QTableWidget(this);
    camera_matrix_table_->setRowCount(4);
    camera_matrix_table_->setColumnCount(4);
    QStringList title;
    title<<"0"<<"1"<<"2"<<"3";
    camera_matrix_table_->setHorizontalHeaderLabels(title);
    camera_matrix_table_->setVerticalHeaderLabels(title);
    for(int i=0; i<4; i++){
        camera_matrix_table_->setColumnWidth(i,50);
//        camera_matrix_table_->setRowHeight(i,50);
        for(int j=0; j<4; j++){
            QDoubleSpinBox * double_box=new QDoubleSpinBox(this);
            double_box->setRange(-10000,10000);
            double_box->setButtonSymbols(QAbstractSpinBox::NoButtons);
            camera_matrix_table_->setCellWidget(i,j,double_box);
        }
    }
    camera_matrix_table_->setFixedHeight(160);

    intrinsics_matrix_table_=new QTableWidget(this);
    intrinsics_matrix_table_->setColumnCount(4);
    intrinsics_matrix_table_->setRowCount(4);
    intrinsics_matrix_table_->setHorizontalHeaderLabels(title);
    intrinsics_matrix_table_->setVerticalHeaderLabels(title);
    for(int i=0; i<4; i++){
        intrinsics_matrix_table_->setColumnWidth(i,50);
//        intrinsics_matrix_table_->setRowHeight(i,30);
        for(int j=0; j<4; j++){
            QDoubleSpinBox * double_box=new QDoubleSpinBox(this);
            double_box->setRange(-10000,10000);
            double_box->setButtonSymbols(QAbstractSpinBox::NoButtons);
            intrinsics_matrix_table_->setCellWidget(i,j,double_box);
        }
    }
    intrinsics_matrix_table_->setFixedHeight(160);

    focal_length_table_=new QTableWidget(this);
    focal_length_table_->setRowCount(1);
    focal_length_table_->setColumnCount(1);
    focal_length_table_->setColumnWidth(0,50);
    focal_length_table_->setFixedHeight(60);
    QDoubleSpinBox * double_box=new QDoubleSpinBox(this);
    double_box->setRange(-10000,10000);
    double_box->setButtonSymbols(QAbstractSpinBox::NoButtons);
    focal_length_table_->setCellWidget(0,0,double_box);

    track_params_table_=new QTableWidget(this);
    track_params_table_->setRowCount(1);
    track_params_table_->setColumnCount(4);
    QStringList track_title;
    track_title<<"radius"<<"length"<<"width"<<"num";
    track_params_table_->setHorizontalHeaderLabels(track_title);
    double default_value[4]= {8,28.85,3,4};
    for(int i=0; i<4 ; i++){
        QDoubleSpinBox * double_box=new QDoubleSpinBox(this);
        double_box->setRange(-10000,10000);
        double_box->setButtonSymbols(QAbstractSpinBox::NoButtons);
        double_box->setValue(default_value[i]);
        track_params_table_->setCellWidget(0,i,double_box);
        track_params_table_->setColumnWidth(i,50);
        connect(double_box,SIGNAL(valueChanged(double)),this,SLOT(slotSetTrackParam()));
    }
    track_params_table_->setFixedHeight(60);

    error_box=new QDoubleSpinBox(this);
    error_box->setRange(-10000,10000);
    error_box->setButtonSymbols(QAbstractSpinBox::NoButtons);
    connect(camera_solver_,SIGNAL(currentError(double)),error_box,SLOT(setValue(double)));


    QPushButton* solve_by_decline_button= new QPushButton(tr("Steepest Descent"),this);
    connect(solve_by_decline_button,SIGNAL(clicked(bool)),this,SLOT(slotSolveCameraBySteepestDescent()));


    QHBoxLayout* param_hlayout=new QHBoxLayout;
    QHBoxLayout* matrix_hlayout=new QHBoxLayout;
    QHBoxLayout* track_hlayout_= new QHBoxLayout;

    vlayout->addLayout(param_hlayout);
    vlayout->addLayout(matrix_hlayout);
    vlayout->addWidget(focal_length_table_);
    vlayout->addLayout(track_hlayout_);

    param_hlayout->addWidget(params_table_);
    param_hlayout->addWidget(apply_button);

    matrix_hlayout->addWidget(camera_matrix_table_);
    matrix_hlayout->addWidget(intrinsics_matrix_table_);

    track_hlayout_->addWidget(track_params_table_);
    track_hlayout_->addWidget(error_box);
    track_hlayout_->addWidget(solve_by_decline_button);

    bottom_control_layout_->addLayout(vlayout);

    connect(camera_,&Camera::signalCameraUpdated,this,&MainWindow::slotUpateCameraParams);
}

void MainWindow::drawDistField(){
    QLabel*result_label=new QLabel(this);
    int w=img_render_label_->width(), h=img_render_label_->height();
    QImage image(img_render_label_->size(),QImage::Format_RGBA8888);
    image.fill(Qt::white);
    DistField* field=camera_solver_->getDistField();
    for(int i=0; i<w; i++){
        for(int j=0; j<h; j++){
            double value=pow((field->getValue(i,j)/1e6),0.1);
            QColor c=QColor::fromHsvF(value,value,value);
            image.setPixelColor(i,j,c);
        }
    }
    result_label->setPixmap(QPixmap::fromImage(image));
    result_label->setGeometry(0,h,w,h);
    result_label->show();
}

void MainWindow::setDataset(QVector<QPointF> &point_set)
{
    data_set_=point_set;
}

void MainWindow::updateDataWidget()
{
//    data_set_.clear();

    QList<QHBoxLayout*> all_children=point_layout_->findChildren<QHBoxLayout*>();
    for(int i=0; i<all_children.size(); i++){

        all_children[i]->deleteLater();
    }
    u_spinbox_list_.clear();
    v_spinbox_list_.clear();
    int num=data_set_.size();
    for(int i=0; i<num; i++){
        float x=data_set_[i].x(), y=data_set_[i].y();
        QLabel* pos_label= new QLabel(this);
        pos_label->setText("X,Y:"+QString::number(x)+","+QString::number(y));
        QDoubleSpinBox* u_spinbox= new QDoubleSpinBox(this);
        u_spinbox->setRange(-10000,10000);
        QDoubleSpinBox* v_spinbox= new QDoubleSpinBox(this);
        v_spinbox->setRange(-10000,10000);
        data_set_label_.push_back(pos_label);
        u_spinbox_list_.push_back(u_spinbox);
        v_spinbox_list_.push_back(v_spinbox);
        QHBoxLayout* hlayout=new QHBoxLayout;
        hlayout->addWidget(pos_label);
        hlayout->addWidget(u_spinbox);
        hlayout->addWidget(v_spinbox);
        point_layout_->addLayout(hlayout);
    }
    point_widget_->adjustSize();
    point_widget_->update();
    point_widget_->show();
}

void MainWindow::addRotateShortCut(){
    QAction* pAction= new QAction(tr("right"),this);
    pAction->setShortcut(Qt::Key_D);
    connect(pAction,&QAction::triggered,model_scene_,&GCL::ModelScene::slotRotateRight);
    this->addAction(pAction);
    pAction= new QAction(tr("left"),this);
    pAction->setShortcut(Qt::Key_A);
    connect(pAction,&QAction::triggered,model_scene_,&GCL::ModelScene::slotRotateLeft);
    this->addAction(pAction);
    pAction= new QAction(tr("up"),this);
    pAction->setShortcut(Qt::Key_W);
    connect(pAction,&QAction::triggered,model_scene_,&GCL::ModelScene::slotRotateUp);
    this->addAction(pAction);
    pAction= new QAction(tr("down"),this);
    pAction->setShortcut(Qt::Key_S);
    connect(pAction,&QAction::triggered,model_scene_,&GCL::ModelScene::slotRotateDown);
    this->addAction(pAction);
    pAction= new QAction(tr("ClockWise"),this);
    pAction->setShortcut(Qt::Key_Q);
    connect(pAction,&QAction::triggered,model_scene_,&GCL::ModelScene::slotRotateClockWise);
    this->addAction(pAction);
    pAction= new QAction(tr("CounterClockwise"),this);
    pAction->setShortcut(Qt::Key_E);
    connect(pAction,&QAction::triggered,model_scene_,&GCL::ModelScene::slotRotateCounterClockWise);
    this->addAction(pAction);

}

//void MainWindow::slotOpenVideo(){
//    QString filename=QFileDialog::getOpenFileName(this);
//    cv::VideoCapture capture(filename.toStdString().c_str());
//    if(!capture.isOpened()){
//        qDebug()<<"cannot open:"<<filename;
//        return;
//    }
//    double rate =capture.get(cv::CAP_PROP_FPS);
//    bool stop(false);
//    cv::Mat frame;
////    cv::namedWindow("frames");
//    int delay =1000/rate;

//    while(!stop){
//        if(!capture.read(frame)){
//            break;
//        }
//        cv::imshow("frame",frame);
//        if(cv::waitKey(delay)>=0){
//            stop=true;
//        }
//    }
//}

void MainWindow::slotOpenImage(){

    video_mode = false;
    QString filename=QFileDialog::getOpenFileName(this);
    if(filename.isEmpty()) return;
    image_filename_=filename;
    slotLoadImageAndData(filename);

    QFileInfo info(filename);
    QDir dir=info.absoluteDir();
    QStringList filter;
    filter<<"*.jpg";
    QFileInfoList tmp_list=dir.entryInfoList(filter,QDir::NoFilter,QDir::Time);
    int num=tmp_list.size();
    for(int i=num-1; i>-1; i--){
        frame_list_.push_back(tmp_list[i]);
        qDebug()<<tmp_list[i].fileName();
    }
    current_frame_index_=0;

    camera_solver_->init_param();//--------

//    findbound_ = new FindBound(filename);
}

//void MainWindow::slotOpenImage()

void MainWindow::slotOpenVideo()
{
    QString filename=QFileDialog::getOpenFileName(this);
    if(filename.isEmpty()) return;
    video_name_=filename;
    slotLoadVideo(filename);
}


void MainWindow::slotAddPoint(int x, int y){
    QPointF point_f=QPointF(x,y);
    data_set_.push_back(point_f);
    qDebug()<<"add point:";
//    QLabel* pos_label= new QLabel(this);
//    pos_label->setText("X,Y:"+QString::number(x)+","+QString::number(y));
//    QDoubleSpinBox* u_spinbox= new QDoubleSpinBox(this);
//    u_spinbox->setRange(-10000,10000);
//    QDoubleSpinBox* v_spinbox= new QDoubleSpinBox(this);
//    v_spinbox->setRange(-10000,10000);
//    data_set_label_.push_back(pos_label);
//    u_spinbox_list_.push_back(u_spinbox);
//    v_spinbox_list_.push_back(v_spinbox);
//    QHBoxLayout* hlayout=new QHBoxLayout;
//    hlayout->addWidget(pos_label);
//    hlayout->addWidget(u_spinbox);
//    hlayout->addWidget(v_spinbox);
//    point_layout_->addLayout(hlayout);
//    point_widget_->adjustSize();
//    point_widget_->update();

}

void MainWindow::slotPopPoint()
{
    img_render_label_->slotPopPoint();
    data_set_.pop_back();
}

void MainWindow::slotLoadCameraParams(GCL::Vec<9, double> param){
    camera_->setParams(param);
    render_widget_->update();
}

void MainWindow::slotSetCameraParams(){
    Vec<9,double> params;
    for(int i=0; i<9; i++){
        params[i]=((QDoubleSpinBox*)params_table_->cellWidget(0,i))->value();
    }
    camera_->setParams(params);
    render_widget_->update();
}


void MainWindow::slotUpateCameraParams(){
    Vec<9,double> params=camera_->getParams();
    for(int i=0; i<9; i++){
        ((QDoubleSpinBox*)params_table_->cellWidget(0,i))->setValue(params[i]);
    }
    HomoMatrix4 modelview_matrix=camera_->get_model_matrix();
    HomoMatrix4 projection_matrix=camera_->get_project_matrix();
    for(int i=0; i<4; i++){
        for(int j=0; j<4; j++){
            ((QDoubleSpinBox*)camera_matrix_table_->cellWidget(i,j))->setValue(modelview_matrix.getValue(i,j));
            ((QDoubleSpinBox*)intrinsics_matrix_table_->cellWidget(i,j))->setValue(projection_matrix.getValue(i,j));
        }
    }
    ((QDoubleSpinBox*)focal_length_table_->cellWidget(0,0))->setValue(camera_->focalLength());
}

void MainWindow::slotSetTrackParam(){
    double param[4];
    for(int i=0; i<4; i++){
        param[i]=((QDoubleSpinBox*)track_params_table_->cellWidget(0,i))->value();
    }
    model_scene_->slotSetTrack(param[0],param[1],param[2],param[3]);
    camera_solver_->setTrack(param[0],param[1]);
    render_widget_->update();
}

void MainWindow::slotFindBound()
{
    QTime time;time.start();

    QRectF target_rect=img_render_label_->getTargetRectF();
    target_rect.normalized();
    double  xmin=target_rect.left()/img_render_label_->rect().width(),
            xmax=target_rect.right()/img_render_label_->rect().width(),
            ymin=target_rect.top()/img_render_label_->rect().height(),
            ymax=target_rect.bottom()/img_render_label_->rect().height();
    findbound_->startFindBound();
    qDebug()<<"find bound time:"<<time.elapsed()<<"ms";
    std::cout << "--------Finish finding bound..." << std::endl;

    data_set_.clear();

    int point_set_mode = 4;

    if(point_set_mode == 1)
    {
        std::vector<std::vector<std::vector<int>>> ReliableBoundPointSet = findbound_->ReturnReliableBoundPoints();

        for(int i = 0; i < ReliableBoundPointSet.size(); ++i)
        {
            for(int j = 1; j < ReliableBoundPointSet[i].size(); ++j)
            {
                slotAddPoint(int(ReliableBoundPointSet[i][j][1] * img_render_label_->slotImageSize().width() / render_image_.width()), int(ReliableBoundPointSet[i][j][0] * img_render_label_->slotImageSize().height() / render_image_.height()));
                img_render_label_->slotAddPoint(QPointF(int(ReliableBoundPointSet[i][j][1] * img_render_label_->slotImageSize().width() / render_image_.width()), int(ReliableBoundPointSet[i][j][0] * img_render_label_->slotImageSize().height() / render_image_.height())));
            }
        }
    }
    else if(point_set_mode == 2)
    {
        std::vector<std::vector<int>> FinalBoundPointSet = findbound_->ReturnFinalBoundPoints();

        for(int i = 0; i < FinalBoundPointSet.size(); ++i)
        {
            slotAddPoint(int(FinalBoundPointSet[i][1] * img_render_label_->slotImageSize().width() / render_image_.width()), int(FinalBoundPointSet[i][0] * img_render_label_->slotImageSize().height() / render_image_.height()));
            img_render_label_->slotAddPoint(QPointF(int(FinalBoundPointSet[i][1] * img_render_label_->slotImageSize().width() / render_image_.width()), int(FinalBoundPointSet[i][0] * img_render_label_->slotImageSize().height() / render_image_.height())));
        }
    }
    else if(point_set_mode == 3)
    {
        std::vector<std::vector<int>> FinalBoundPointSetWithSobel = findbound_->ReturnFinalBoundPointsWithSobel();
        qDebug()<<"bound point num:"<<FinalBoundPointSetWithSobel.size();
        for(int i = 0; i < FinalBoundPointSetWithSobel.size(); ++i)
        {
            slotAddPoint(int(FinalBoundPointSetWithSobel[i][1] * img_render_label_->slotImageSize().width() / render_image_.width()), int(FinalBoundPointSetWithSobel[i][0] * img_render_label_->slotImageSize().height() / render_image_.height()));
            img_render_label_->slotAddPoint(QPointF(int(FinalBoundPointSetWithSobel[i][1] * img_render_label_->slotImageSize().width() / render_image_.width()), int(FinalBoundPointSetWithSobel[i][0] * img_render_label_->slotImageSize().height() / render_image_.height())));
        }
    }
    else if(point_set_mode == 4)
    {
        std::vector<std::vector<int>> MergedFinalBoundPointSetWithSobel = findbound_->ReturnMergedFinalBoundPointsWithSobel();
        qDebug()<<"bound point num:"<<MergedFinalBoundPointSetWithSobel.size();
        for(int i = 0; i < MergedFinalBoundPointSetWithSobel.size(); ++i)
        {
            slotAddPoint(int(MergedFinalBoundPointSetWithSobel[i][1] * img_render_label_->slotImageSize().width() / render_image_.width()), int(MergedFinalBoundPointSetWithSobel[i][0] * img_render_label_->slotImageSize().height() / render_image_.height()));
            img_render_label_->slotAddPoint(QPointF(int(MergedFinalBoundPointSetWithSobel[i][1] * img_render_label_->slotImageSize().width() / render_image_.width()), int(MergedFinalBoundPointSetWithSobel[i][0] * img_render_label_->slotImageSize().height() / render_image_.height())));
        }
    }

    delete(findbound_);

}

void MainWindow::slotLoadNextFrameAndRun()
{
    current_frame_index_++;
    if(current_frame_index_<max_frame_index_){
        slotLoadImageAndData(frame_list_[current_frame_index_].absoluteFilePath(),true,false,false);
        //QTimer::singleShot(500,this,&MainWindow::slotSolveCameraBySteepestDescent);
        slotSolveCameraBySteepestDescent();
        if(save_camera_param_)
            slotSaveCameraParamsToFile(frame_list_[current_frame_index_].absoluteFilePath());
        render_widget_->update();
        QTimer::singleShot(1000,this,[=]{emit signalFinishProcessFrame();});

    }
}

void MainWindow::slotProcessFrames()
{
    current_frame_index_=-1;
    max_frame_index_=9;
    save_camera_param_=false;
    slotLoadNextFrameAndRun();
}

void MainWindow::slotProcessFramesAndSave()
{
    current_frame_index_=-1;
    max_frame_index_=9;
    save_camera_param_=true;
    slotLoadNextFrameAndRun();
}

void MainWindow::slotSolveCameraBySteepestDescent(){

    QTime time;time.start();
    //400m--straight lenght:84.39m,circle radius:36.5m
    //skate --straight:28.85m, radius: 8.00m
    camera_solver_->setTrack(8.0,28.85);

    QVector<QPointF> data_set_trans;

    for(int i = 0; i < data_set_.size(); ++i)
    {
        QPointF point_trans(img_render_label_->slotImageSize().width()/2 - data_set_[i].x(), data_set_[i].y() - img_render_label_->slotImageSize().height()/2);

        data_set_trans.push_back(point_trans);
    }

    camera_solver_->setDataSet(data_set_trans);
    camera_solver_->getCameraParams();
    qDebug()<<"set param time:"<<time.elapsed()<<"ms";
    //camera_solver_->steepestDescentMethod(0);
    camera_solver_->steepestDescentMethod2D();
    qDebug()<<"steepest time:"<<time.elapsed()<<"ms";
    model_scene_->setPointSet(camera_solver_->getProjectedSampleSet());
    render_widget_->update();
    bottom_control_widget_->update();
    camera_solver_->updateUnprojectedData();


//    drawDistField();
}

void MainWindow::getFiles( std::string path, std::vector<std::string>& files )
{
    //文件句柄
    //long hFile = 0;  //win7
    intptr_t hFile = 0;   //win10
    //文件信息
    struct _finddata_t fileinfo;
    std::string p;
    if((hFile = _findfirst(p.assign(path).append("\\*").c_str(),&fileinfo)) !=  -1)
    // "\\*"是指读取文件夹下的所有类型的文件，若想读取特定类型的文件，以png为例，则用“\\*.png”
    {
        do
        {
            //如果是目录,迭代之
            //如果不是,加入列表
            if((fileinfo.attrib &  _A_SUBDIR))
            {
                if(strcmp(fileinfo.name,".") != 0  &&  strcmp(fileinfo.name,"..") != 0)
                    getFiles( p.assign(path).append("/").append(fileinfo.name), files );
            }
            else
            {
                files.push_back(path + "/" + fileinfo.name);
            }
        }while(_findnext(hFile, &fileinfo)  == 0);
        _findclose(hFile);
    }
}


void MainWindow::slotLoadImageAndData(QString filename, bool load_point, bool load_camera, bool update_data_widget){

    render_image_.load(filename);
    qDebug()<<"Input Image Size : "<<render_image_.width()<<render_image_.height();

    img_render_label_->slotSetImage(&render_image_);
    img_render_label_->update();
    render_widget_->setFixedSize(img_render_label_->slotImageSize());
    render_widget_->loadTargetImage(render_image_.copy());
    render_widget_->update();

    if(load_point)
        slotLoadPointDataFromFile(filename);
    if(load_camera)
        slotLoadCameraParamsFromFile(filename);
    if(update_data_widget)
        updateDataWidget();

}

void MainWindow::slotLoadPointDataFromFile(QString filename)
{
    QFileInfo info(filename);
    QString txt_filename=filename.remove(info.suffix()).append("txt");
    QFile data_file(txt_filename);
    bool result=data_file.open(QFile::ReadOnly);
    data_set_.clear();
    if(!result) return;

    QList<QHBoxLayout*> all_children=point_layout_->findChildren<QHBoxLayout*>();
    for(int i=0; i<all_children.size(); i++){

        all_children[i]->deleteLater();
    }
    u_spinbox_list_.clear();
    v_spinbox_list_.clear();


    QTextStream stream(&data_file);
    while(!stream.atEnd()){
        QString line=stream.readLine();

        QStringList param=line.split(QRegExp(",|;"));
//        slotAddPoint(param[0].toInt(),param[1].toInt());
        data_set_.push_back(QPointF(param[0].toFloat(),param[1].toFloat()));
        img_render_label_->slotAddPoint(QPointF(param[0].toInt(),param[1].toInt()));
//        u_spinbox_list_.back()->setValue(param[2].toFloat());
//        v_spinbox_list_.back()->setValue(param[3].toFloat());
    }

    img_render_label_->update();
    point_widget_->adjustSize();
    point_widget_->update();
}

void MainWindow::slotLoadCameraParamsFromFile(QString filename)
{
    QFileInfo info(filename);
    QString xml_filename=filename.remove(info.suffix()).append("xml");

    QFile xml_file(xml_filename);
    bool result=xml_file.open(QFile::ReadOnly);
    if(!result) return;
    QTextStream xml_stream(&xml_file);

    int count=0;
    while(!xml_stream.atEnd()&& count<3){
        QString line=xml_stream.readLine();
        if(line.contains("<!--self-Use-Params:")){
            QStringList param_string=line.split(QRegExp(",|:"));
            Vec<9,double> param;
            for(int i=0; i<9; i++){
                param[i]=param_string[i+1].toDouble();
            }
            slotLoadCameraParams(param);
        }

        count++;
    }
}

void MainWindow::slotSavePointDataToFile(QString filename)
{
    QFileInfo info(filename);
    QString txt_filename=filename.remove(info.suffix()).append("txt");
    QFile data_file(txt_filename);
    data_file.open(QFile::WriteOnly);
    QTextStream stream(&data_file);
    int num=data_set_.size();
    qDebug()<<"data_set_num:"<<num;
    for(int i=0; i<num; i++){
        stream<<QString::number(data_set_[i].x())<<","<<QString::number(data_set_[i].y())<<";"
             <<0<<","<<0<<";\n";
//             <<QString::number(u_spinbox_list_[i]->value())<<","<<QString::number(v_spinbox_list_[i]->value())<<";"<<"\n";
    }
    data_file.close();
}

void MainWindow::slotSaveCameraParamsToFile(QString filename)
{
    QFileInfo info(filename);
    QString xml_filename=filename.remove(info.suffix()).append("xml");
    QFile data_file(xml_filename);
    data_file.open(QFile::WriteOnly);
    QTextStream stream(&data_file);
    stream<<"<?xml version=\"1.0\"?>\n";
    stream<<"<!--self-Use-Params:";
    Vec<9,double> params=camera_->getParams();
    for(int i=0; i<9; i++){
        stream<<QString::number(params[i])<<",";
    }
    stream<<"-->\n";
    stream<<"<!-- Note: Camera matrix translation units are meters. -->\n"
            "<opencv_storage>\n"
            "<CameraMatrix type_id=\"opencv-matrix\">\n"
              "<rows>3</rows>\n"
              "<cols>4</cols>\n"
              "<dt>d</dt>\n"
              "<data>\n";

    camera_solver_->getCameraParams();

    HomoMatrix4 modelview_matrix=camera_->get_model_matrix();
    HomoMatrix4 projection_matrix=camera_->get_project_matrix();

    for(int i=0; i<3; i++){
        for(int j=0; j<4; j++){
            stream<<QString::number(modelview_matrix.getValue(i,j))<<" ";
        }
    }
    stream<<"</data>"
            "</CameraMatrix>"
            "<Intrinsics type_id=\"opencv-matrix\">"
            "<rows>3</rows>"
            "<cols>3</cols>"
            "<dt>d</dt>"
            "<data>";
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            stream<<QString::number(projection_matrix.getValue(i,j))<<" ";
        }
    }
    stream<<"</data></Intrinsics>"
            "<Distortion type_id=\"opencv-matrix\">"
              "<rows>8</rows>"
              "<cols>1</cols>"
              "<dt>d</dt>"
              "<data>0 0 0 0 0 0 0 0</data></Distortion>"
            "</opencv_storage>";

    data_file.close();
}

void MainWindow::slotLoadVideo(QString filename){


        video_name_ = filename;
        video_mode = true;

        cv::VideoCapture cap(filename.toStdString());

        int totalFrameNumber = cap.get(cv::CAP_PROP_FRAME_COUNT);
        std::cout << "----total frames---- : " << totalFrameNumber << std::endl;

        std::string folderPath = (filename.split(filename.split("/").back())[0] + "Frames_" + filename.split("/").back().split(".")[0]).toStdString();

        int cap_fps = 5;
        //for_test
        totalFrameNumber = 210;


        if (0 != access(folderPath.c_str(), 0))
        {
            mkdir(folderPath.c_str());

            cv::Mat frame, frame_in;
            int currentFrame = 0;

            while (true)
            {
                if (currentFrame >= totalFrameNumber)
                {
                    break;
                }

                cap.read(frame_in);
                double times = 600.0 / frame_in.rows;
                if(times > 400.0 / frame_in.cols)
                {
                    times = 400.0 / frame_in.cols;
                }
                cv::resize(frame_in, frame, cv::Size(0, 0), times, times, cv::INTER_LANCZOS4);

                std::stringstream str;
                str << filename.split("/").back().split(".")[0].toStdString() << "_" << currentFrame << ".jpg";
                if(currentFrame % cap_fps == 0)
                {
                    std::cout << "Writing frames : " << currentFrame << " / " << totalFrameNumber << std::endl;
                }

                if (currentFrame % cap_fps == 0)
                {
                    cv::imwrite(folderPath + "/" + str.str(), frame);
                }

                currentFrame++;
            }
        }
        else
        {
            std::vector<std::string> files;

            getFiles(folderPath, files);

            for(int i = 0; i < files.size(); ++i)
            {
                files[i] = QString(files[i].c_str()).split("/").back().toStdString();
            }

            cv::Mat frame, frame_in;
            int currentFrame = 0;

            while (true)
            {
                if (currentFrame >= totalFrameNumber)
                {
                    break;
                }

                cap.read(frame_in);
                double times = 600.0 / frame_in.rows;
                if(times > 400.0 / frame_in.cols)
                {
                    times = 400.0 / frame_in.cols;
                }
                cv::resize(frame_in, frame, cv::Size(0, 0), times, times, cv::INTER_LANCZOS4);

                std::stringstream str;
                str << filename.split("/").back().split(".")[0].toStdString() << "_" << currentFrame << ".jpg";
                if(currentFrame % cap_fps == 0)
                {
                    std::cout << "Writing frames : " << currentFrame << " / " << totalFrameNumber << std::endl;
                }

                if (currentFrame % cap_fps == 0)
                {
                    bool exist_this_frame = false;

                    for(int i = 0; i < files.size(); ++i)
                    {
                        if(files[i] == str.str())
                        {
                            exist_this_frame = true;

                            break;
                        }
                    }

                    if(!exist_this_frame)
                    {
                        cv::imwrite(folderPath + "/" + str.str(), frame);
                    }
                }

                currentFrame++;
            }
        }

        int currentFrame = 0;
        int solve_process = 0;

        std::vector<std::string> files;

        getFiles(folderPath, files);

        while (true)
        {
            if (currentFrame >= totalFrameNumber)
            {
                break;
            }

            if (currentFrame % cap_fps == 0)
            {
                int frame_idx = -1;

                for(int i = 0; i < files.size(); ++i)
                {
                    if(atoi(QString(files[i].c_str()).split("/").back().split(".")[0].split("_").back().toStdString().c_str()) == currentFrame)
                    {
                        frame_idx = i;

                        break;
                    }
                }
                ++solve_process;
                std::cout << "Solving frames : " << solve_process << " / " << files.size() << std::endl;

                slotSolveVideoFrame(QString(files[frame_idx].c_str()));
                cv::waitKey();
            }

            currentFrame++;
        }

}

void MainWindow::slotSolveVideoFrame(QString filename)
{

}

void MainWindow::slotSavePointData(){
    updateDataWidget();
    QVector<GCL::Vec3> unprojected_data=camera_solver_->getUnprojectedData();
    int num=unprojected_data.size();
    for(int i=0; i<num; i++){
        u_spinbox_list_[i]->setValue(unprojected_data[i][0]);
        v_spinbox_list_[i]->setValue(unprojected_data[i][1]);
    }
    slotSavePointDataToFile(image_filename_);
}


void MainWindow::slotSaveCameraParams(){
    QString filename=image_filename_;
    if(video_mode)
    {
        filename=video_name_;
    }
    slotSaveCameraParamsToFile(filename);

}
