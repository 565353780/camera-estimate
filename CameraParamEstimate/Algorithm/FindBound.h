#ifndef FINDBOUND_H
#define FINDBOUND_H

#include <QHash>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
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

#include <iostream>
#include <math.h>

#define PI 3.14159265359

#define mesh_dist 3

class FindBound
{

public:

    FindBound();
    FindBound(QString image_name);
    ~FindBound();

    void ReadImage(QString image_name);

    void startFindBound();

    void ShowImage();

    void ShowResultImage();

    std::vector<int> ReturnImageSize();

    std::vector<std::vector<std::vector<int>>> ReturnReliableBoundPoints();

    std::vector<std::vector<int>> ReturnFinalBoundPoints();

    std::vector<std::vector<int>> ReturnFinalBoundPointsWithSobel();

    std::vector<std::vector<int>> ReturnMergedFinalBoundPointsWithSobel();

private:

    void RemoveTrueBlack();

    void ChangeBrightnessAndContrast(float a, int b);

    void TransToAverageColor(int x, int y, int size=mesh_dist, double speed=0.3, double max_error=80);

    int rgb2hsv_max(int a,int b,int c);

    int rgb2hsv_min(int a,int b,int c);

    std::vector<int> rgb2hsv(int r,int g,int b);

    void UpdateHSVColor();

    void UpdateHSVColorSorted();

    double GetHSVDist(std::vector<int> hsv_1, std::vector<int> hsv_2);

    double GetHSVWeightDist(std::vector<int> hsv_1, std::vector<int> hsv_2);

    double CaiDu(double a,double b);

    double SeDiaoJiao(double a, double b);

    double delta_Eab(double L1, double a1, double b1, double L2, double a2, double b2);

    double delta_E00(double L1, double a1, double b1, double L2, double a2, double b2);

    void FindSameColorAreaFromImagePiexl(int tolerance=10, int diff_color_dist=40, int color_num=7);

    void FindSameColorAreaFromRGB(int tolerance=10, int diff_color_dist=40, int color_num=7);

    void FindSameColorPoints(int min_size=1, int delta_dist=mesh_dist);

    void MergeSameColorPoints();

    void FindSameColorBoundPoints(int min_size=2, int delta_dist=int(2*mesh_dist));

    void FindSameColorBoundPointsWithBoundColor(int min_size=2, int delta_dist=int(2*mesh_dist));

    bool IsDifferentColorOf(int bound_point_color_idx, int color_x, int color_y);

    void FindRealBoundPoints(int delta_pixel=int(4*mesh_dist));

    void GetReliableBoundPoints(int delta_dist=mesh_dist, int max_reliable=5);

    void ExpandReliableBoundPoints(int expand_dist=int(3*mesh_dist));

    int GetNeighboorNum(int x, int y, int max_dist=int(2.9*mesh_dist));

    void RemoveUselessPoints(int min_neighboor_point_num=2);

    void GetFinalReliableBoundPoints(int delta_dist=int(4*mesh_dist));

    void GetFinalExpandBoundPoints(int max_dist=int(1.5*mesh_dist));

    void GetFinalBoundPoints(int min_dist_to_image_bound=int(1.5*mesh_dist));

    void UpdateFinalBoundPointsWithSobel(int search_dist=3);

    void RemoveSigleFinalPoints(int search_dist=4, int min_group_num=2);

    void RemoveSigleMergedFinalPoints(int search_dist=29, int min_group_num=1);

    void RemoveSigleBoundMergedFinalPoints(int search_dist=19, int min_group_num=2, int max_bound_dist=30);

    void DrawSelectedColor(int length=20);

    void DrawColorAreaWithFixedColors(int draw_tolerance=30);

    void DrawColorAreaWithMostOfColorsInImage(int draw_tolerance=30);

    void RemoveHoles(int search_dist=20);

    void RemoveBlackPoint(int search_dist=5, double black_percent=0.2);

    void MergeHoles(int max_merge_dist=20);

    void MergeFinalPoints(int merge_dist=10, double min_point_dist=5);

    double Dot(std::vector<int> point_1, std::vector<int> point_2);

    double TriangleArea(std::vector<int> point_1, std::vector<int> point_2, std::vector<int> point_3);

    bool IsInTriangle(std::vector<int> point_1, std::vector<int> point_2, std::vector<int> point_3, std::vector<int> point_judge, double in_scale);

    void RemoveInsidePoints(double remove_scale=0.8, double min_triangle_area=2000);

    void DrawBoundPoints();

    void DrawReliableBoundPoints();

    void DrawFinalBoundPoints(int point_size=3);

    void on_Canny(int, void *);

    void on_Sobel(int, void *);

    void Scharr();

    void GetBoundWithCV();

private:
    int color_tolerance;
    int different_color_dist;
    int delta_pixel;
    int directions_num = 64;
    int outer_point_set_idx;

    bool USE_HSV = false;
    bool DEBUG_MODE = false;

    cv::Mat Image, Image_Copy;
    cv::Mat Image_to_draw;
    std::vector<std::vector<double>> directions;
    std::vector<std::vector<std::vector<int>>> HSV_Color;
    std::vector<std::vector<int>> HSV_ColorSorted;
    std::vector<std::vector<int>> ColorsSortedByArea;//[r, g, b, same_color_points_num]
    std::vector<std::vector<int>> BoundPointSet;
    std::vector<std::vector<std::vector<int>>> SameColorPointSet;//[[color, point1, point2, ...], ...]
    std::vector<std::vector<std::vector<int>>> SameColorBoundPointSet;
    std::vector<std::vector<std::vector<int>>> ReliableBoundPointSet;
    std::vector<std::vector<int>> FinalReliableBoundPointSet;
    std::vector<std::vector<int>> FinalExpandBoundPointSet;
    std::vector<std::vector<int>> FinalBoundPointSet;
    std::vector<std::vector<std::vector<int>>> ConnectedBoundPointSet;
    std::vector<std::vector<int>> FinalBoundPointSetWithSobel;
    std::vector<std::vector<int>> MergedFinalBoundPointSetWithSobel;

    std::vector<std::vector<bool>> HaveDrawnAtImage;
    std::vector<std::vector<bool>> HaveChoosedAtBoundPointSet;
    std::vector<std::vector<bool>> ReliableBoundPointUsed;

    QHash<int,int> boundary_points_;
    cv::Rect boundary_rect_;

    std::vector<int> target_color_;


    //原图，和目标图
    cv::Mat g_srcImage, g_srcGrayImage, g_dstImage, g_dstGrayImage;

    //Canny边缘检测相关的变量
    cv::Mat g_cannyDetectedEdges;
    int g_cannyLowThreshold; // TrackBar 位置参数

    // Sobel边缘检测相关变量
    cv::Mat g_sobelGradient_X, g_sobelGradient_Y;
    cv::Mat g_sobelAbsGradient_X, g_sobelAbsGradient_Y;
    int g_sobelKernelSize;  // TrackBar 位置参数

    //Scharr滤波器相关参数
    cv::Mat g_scharrGradient_x, g_scharrGradient_Y;
    cv::Mat g_scharrAbsGradient_x, g_scharrAbsGradient_Y;
};

#endif // FINDBOUND_H
