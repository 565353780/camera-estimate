#include "FindBound.h"
#include <QRect>

FindBound::FindBound()
{
    g_cannyLowThreshold = 1;
    g_sobelKernelSize = 1;

    directions.resize(directions_num);
    for(int i = 0; i < directions.size(); ++i)
    {
        directions[i].resize(2);
        directions[i][0] = cos(2.0 * i * PI / directions.size());
        directions[i][1] = sin(2.0 * i * PI / directions.size());
    }
}

FindBound::FindBound(QString image_name)
{
    g_cannyLowThreshold = 1;
    g_sobelKernelSize = 1;

    directions.resize(directions_num);
    for(int i = 0; i < directions.size(); ++i)
    {
        directions[i].resize(2);
        directions[i][0] = cos(2.0 * i * PI / directions.size());
        directions[i][1] = sin(2.0 * i * PI / directions.size());
    }

    ReadImage(image_name);
}

FindBound::~FindBound()
{

}

void FindBound::ReadImage(QString image_name)
{
    DEBUG_MODE = true;

    Image = cv::imread(image_name.toStdString());

    RemoveTrueBlack();

    Image.copyTo(Image_Copy);
    Image.copyTo(Image_to_draw);

    UpdateHSVColor();

    HaveDrawnAtImage.resize(Image.rows);
    for(int i = 0; i < Image.rows; ++i)
    {
        HaveDrawnAtImage[i].resize(Image.cols);
        for(int j = 0; j < Image.cols; ++j)
        {
            HaveDrawnAtImage[i][j] = false;
        }
    }

    //GetBoundPointSet();

    //ChangeGain(1.2, 1000);

    /*
    for(int i = 0; i < Image.rows; i+=3)
    {
        for(int j = 0; j < Image.cols; j+=3)
        {
            TransToAverageColor(i, j, 10, 0.3, 100);
        }
    }
    for(int i = 0; i < Image.rows; i+=3)
    {
        for(int j = 0; j < Image.cols; j+=3)
        {
            TransToAverageColor(i, j, 20, 0.3, 100);
        }
    }
    for(int i = 0; i < Image.rows; i+=3)
    {
        for(int j = 0; j < Image.cols; j+=3)
        {
            TransToAverageColor(i, j, 30, 0.3, 100);
        }
    }
    */

    //ShowImage();

    //cv::waitKey();

}

void FindBound::startFindBound()
{
    GetBoundWithCV();
    UpdateHSVColor();

    FindSameColorAreaFromImagePiexl();
    DrawColorAreaWithMostOfColorsInImage();
    RemoveHoles();
    //MergeHoles();
    RemoveBlackPoint();
    //DrawSelectedColor();
    FindSameColorBoundPointsWithBoundColor();
    FindRealBoundPoints();
    GetReliableBoundPoints();
    ExpandReliableBoundPoints();
    RemoveUselessPoints();
    DrawReliableBoundPoints();
    GetFinalReliableBoundPoints();
    GetFinalExpandBoundPoints();
    GetFinalBoundPoints();
    UpdateFinalBoundPointsWithSobel();
    RemoveSigleFinalPoints();
    RemoveSigleFinalPoints(39, 20);
    MergeFinalPoints();
    RemoveSigleMergedFinalPoints();
    RemoveInsidePoints();
    RemoveSigleBoundMergedFinalPoints();

    if(DEBUG_MODE)
    {
        ShowResultImage();
    }
}

void FindBound::ShowImage()
{
    cv::imshow("Source Image", Image_Copy);
}

void FindBound::ShowResultImage()
{
    cv::imshow("Color Clustering Result", Image);

    cv::imshow("Sobel Bound Search Result", g_dstImage);

    cv::imshow("Reliable Bound Points Result", Image_to_draw);
}

std::vector<int> FindBound::ReturnImageSize()
{
    std::vector<int> image_size;
    image_size.resize(2);
    image_size[0] = Image.rows;
    image_size[1] = Image.cols;

    return image_size;
}

std::vector<std::vector<std::vector<int>>> FindBound::ReturnReliableBoundPoints()
{
    return ReliableBoundPointSet;
}

std::vector<std::vector<int>> FindBound::ReturnFinalBoundPoints()
{
    return FinalBoundPointSet;
}

std::vector<std::vector<int>> FindBound::ReturnFinalBoundPointsWithSobel()
{
    return FinalBoundPointSetWithSobel;
}

std::vector<std::vector<int>> FindBound::ReturnMergedFinalBoundPointsWithSobel()
{
    return MergedFinalBoundPointSetWithSobel;
}

void FindBound::RemoveTrueBlack()
{
    for(int i = 0; i < Image.rows; ++i)
    {
        for(int j = 0; j < Image.cols; ++j)
        {
            if(Image.at<cv::Vec3b>(i, j)[0] + Image.at<cv::Vec3b>(i, j)[1] + Image.at<cv::Vec3b>(i, j)[2] == 0)
            {
                Image.at<cv::Vec3b>(i, j)[0] = 1;
            }
        }
    }
}

void FindBound::ChangeBrightnessAndContrast(float a, int b)
{
    if(b == 1000)
    {
        std::vector<double> p;
        p.resize(256);
        for(int i = 0; i < 256; ++i)
        {
            p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, a) * 255.0);
        }

        for (int r = 0; r < Image.rows; r++)
        {
            for (int c = 0; c < Image.cols; c++)
            {
                for (int i = 0; i < 3; i++)
                {
                    Image.at<cv::Vec3b>(r, c)[i] =
                        cv::saturate_cast<uchar>(p[Image.at<cv::Vec3b>(r, c)[i]]);
                }
            }
        }
    }
    else
    {
        for (int r = 0; r < Image.rows; r++)
        {
            for (int c = 0; c < Image.cols; c++)
            {
                for (int i = 0; i < 3; i++)
                {
                    Image.at<cv::Vec3b>(r, c)[i] =
                        cv::saturate_cast<uchar>(a * Image.at<cv::Vec3b>(r, c)[i] + b);
                }
            }
        }
    }
}

void FindBound::TransToAverageColor(int x, int y, int size, double speed, double max_error)
{
    double mean_x = 0;
    double mean_y = 0;
    double mean_z = 0;

    double sum = 0;

    int available_pixel_num = 0;

    for(int i = x - size; i <= x + size; ++i)
    {
        for(int j = y - size; j <= y + size; ++j)
        {
            if(0 <= i && i < Image.rows && 0 <= y && y < Image.cols)
            {
                mean_x += Image.at<cv::Vec3b>(i, j)[0];
                mean_y += Image.at<cv::Vec3b>(i, j)[1];
                mean_z += Image.at<cv::Vec3b>(i, j)[2];

                ++available_pixel_num;
            }
        }
    }

    if(available_pixel_num > 0)
    {
        mean_x /= available_pixel_num;
        mean_y /= available_pixel_num;
        mean_z /= available_pixel_num;

        for(int i = x - size; i <= x + size; ++i)
        {
            for(int j = y - size; j <= y + size; ++j)
            {
                if(0 <= i && i < Image.rows && 0 <= y && y < Image.cols)
                {
                    sum += (Image.at<cv::Vec3b>(i, j)[0] - mean_x) * (Image.at<cv::Vec3b>(i, j)[0] - mean_x);
                    sum += (Image.at<cv::Vec3b>(i, j)[1] - mean_y) * (Image.at<cv::Vec3b>(i, j)[1] - mean_y);
                    sum += (Image.at<cv::Vec3b>(i, j)[2] - mean_z) * (Image.at<cv::Vec3b>(i, j)[2] - mean_z);
                }
            }
        }

        sum = sqrt(sum);

        if(sum < max_error)
        {
            for(int i = x - size; i <= x + size; ++i)
            {
                for(int j = y - size; j <= y + size; ++j)
                {
                    if(0 <= i && i < Image.rows && 0 <= y && y < Image.cols)
                    {
                        Image.at<cv::Vec3b>(i, j)[0] += int(speed * (mean_x - Image.at<cv::Vec3b>(i, j)[0]));
                        Image.at<cv::Vec3b>(i, j)[1] += int(speed * (mean_y - Image.at<cv::Vec3b>(i, j)[1]));
                        Image.at<cv::Vec3b>(i, j)[2] += int(speed * (mean_z - Image.at<cv::Vec3b>(i, j)[2]));
                    }
                }
            }
        }
    }
}

int FindBound::rgb2hsv_max(int a,int b,int c)
{
    int max = a;
    if(b > max) max = b;
    if(c > max) max = c;
    return max;
}

int FindBound::rgb2hsv_min(int a,int b,int c)
{
    int min = a;
    if(b < min) min = b;
    if(c < min) min = c;
    return min;
}

std::vector<int> FindBound::rgb2hsv(int r,int g,int b)
{
    int imax,imin,diff;
    imax = rgb2hsv_max(r,g,b);
    imin = rgb2hsv_min(r,g,b);
    diff = imax - imin;
    int h, s, v;
    v = imax;
    if(imax == 0)
        s = 0;
    else
        s = diff;

    if(diff != 0)
    {
        if(r == imax)
        {
            h = 60 * (g - b) / diff;
        }
        else if(g == imax)
        {
            h = 60 * (b - r) / diff + 120;
        }
        else
        {
            h = 60 * (r - g) / diff + 240;
        }

        if(h < 0)
            h = h + 360;
    }
    else
        h = 0;

    std::vector<int> hsv;
    hsv.emplace_back(h);
    hsv.emplace_back(s);
    hsv.emplace_back(v);

    return hsv;
}

void FindBound::UpdateHSVColor()
{
    if(DEBUG_MODE)
    {
        std::cout << "    Start UpdateHSVColor..." << std::endl;
    }

    HSV_Color.resize(Image.rows);
    for(int i = 0; i < Image.rows; ++i)
    {
        HSV_Color[i].resize(Image.cols);
        for(int j = 0; j < Image.cols; ++j)
        {
            HSV_Color[i][j].resize(3);
        }
    }

    for(int x = 0; x < Image.rows; ++x)
    {
        for(int y = 0; y < Image.cols; ++y)
        {
            std::vector<int> hsv_color = rgb2hsv(Image.at<cv::Vec3b>(x, y)[0], Image.at<cv::Vec3b>(x, y)[1], Image.at<cv::Vec3b>(x, y)[2]);
            HSV_Color[x][y][0] = hsv_color[0];
            HSV_Color[x][y][1] = hsv_color[1];
            HSV_Color[x][y][2] = hsv_color[2];
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "    Finish UpdateHSVColor..." << std::endl;
    }
}

void FindBound::UpdateHSVColorSorted()
{
    HSV_ColorSorted.resize(ColorsSortedByArea.size());
    for(int i = 0; i < HSV_ColorSorted.size(); ++i)
    {
        HSV_ColorSorted[i].resize(3);

        std::vector<int> hsv_color_sorted = rgb2hsv(ColorsSortedByArea[i][0], ColorsSortedByArea[i][1], ColorsSortedByArea[i][2]);
        HSV_ColorSorted[i][0] = hsv_color_sorted[0];
        HSV_ColorSorted[i][1] = hsv_color_sorted[1];
        HSV_ColorSorted[i][2] = hsv_color_sorted[2];
    }
}

double FindBound::GetHSVDist(std::vector<int> hsv_1, std::vector<int> hsv_2)
{
    if(true)
    {
        double dist = 0;
        dist += (hsv_1[0] - hsv_2[0]) * (hsv_1[0] - hsv_2[0]);
        dist += (hsv_1[1] - hsv_2[1]) * (hsv_1[1] - hsv_2[1]);
        dist += (hsv_1[2] - hsv_2[2]) * (hsv_1[2] - hsv_2[2]);
        dist = sqrt(dist);

        return dist;
    }
    else
    {
        if(hsv_1[0] < hsv_2[0])
        {
            return hsv_2[0] - hsv_1[0];
        }
        else
        {
            return hsv_1[0] - hsv_2[0];
        }
    }
}

double FindBound::GetHSVWeightDist(std::vector<int> hsv_1, std::vector<int> hsv_2)
{
    double dist=0;
    dist += 32*abs(hsv_1[0] - hsv_2[0]) ;
    dist += abs(hsv_1[1] - hsv_2[1]);
    dist += 4*abs(hsv_1[2] - hsv_2[2]) ;

    return (dist/(32+1+4));
}

double FindBound::CaiDu(double a,double b)
{
    double Cab = 0;
    Cab = pow(a * a + b * b, 0.5);
    return Cab;
}

double FindBound::SeDiaoJiao(double a, double b)
{
    double h = 0;
    double hab = 0;

    h = (180 / PI) * atan(b / a);           //有正有负

    if (a>0&&b>0)
    {
        hab = h;
    }
    else if (a<0&&b>0)
    {
        hab = 180 + h;
    }
    else if (a<0&&b<0)
    {
        hab = 180 + h;
    }
    else     //a>0&&b<0
    {
        hab = 360 + h;
    }
    return hab;
}

double FindBound::delta_Eab(double L1, double a1, double b1, double L2, double a2, double b2)
{
    double Eab = 0;             //  △Eab
    double chafang_L = 0;             //  (L1-L2)*(L1-L2)
    double chafang_a = 0;             //   (a1-a2)*(a1-a2)
    double chafang_b = 0;             //   (b1-b2)*(b1-b2)

    chafang_L = ( L1 - L2 ) * ( L1 - L2 );      //差-方
    chafang_a = ( a1 - a2 ) * ( a1 - a2 );
    chafang_b = ( b1 - b2 ) * ( b1 - b2 );

    Eab = pow(chafang_L + chafang_a + chafang_b, 0.5);

    return Eab;
}

double FindBound::delta_E00(double L1, double a1, double b1, double L2, double a2, double b2)
{
    double E00 = 0;               //CIEDE2000色差E00
    double LL1, LL2, aa1, aa2, bb1, bb2; //声明L' a' b' （1,2）
    double delta_LL, delta_CC, delta_hh, delta_HH;        // 第二部的四个量
    double kL, kC, kH;
    double RT = 0;                //旋转函数RT
    double G = 0;                  //G表示CIELab 颜色空间a轴的调整因子,是彩度的函数.
    double mean_Cab = 0;    //两个样品彩度的算术平均值
    double SL, SC, SH, T;
    //------------------------------------------
    //参考实验条件见P88
    kL = 1;
    kC = 1;
    kH = 1;
    //------------------------------------------
    mean_Cab = (CaiDu(a1, b1) + CaiDu(a2, b2)) / 2;
    double mean_Cab_pow7 = pow(mean_Cab, 7);       //两彩度平均值的7次方
    G = 0.5*(1-pow(mean_Cab_pow7 / (mean_Cab_pow7 + pow(25, 7)), 0.5));

    LL1 = L1;
    aa1 = a1 * (1 + G);
    bb1 = b1;

    LL2 = L2;
    aa2 = a2 * (1 + G);
    bb2 = b2;

    double CC1, CC2;               //两样本的彩度值
    CC1 = CaiDu(aa1, bb1);
    CC2 = CaiDu(aa2, bb2);
    double hh1, hh2;                  //两样本的色调角
    hh1 = SeDiaoJiao(aa1, bb1);
    hh2 = SeDiaoJiao(aa2, bb2);

    delta_LL = LL1 - LL2;
    delta_CC = CC1 - CC2;
    delta_hh = SeDiaoJiao(aa1, bb1) - SeDiaoJiao(aa2, bb2);
    delta_HH = 2 * sin(PI*delta_hh / 360) * pow(CC1 * CC2, 0.5);

    //-------第三步--------------
    //计算公式中的加权函数SL,SC,SH,T
    double mean_LL = (LL1 + LL2) / 2;
    double mean_CC = (CC1 + CC2) / 2;
    double mean_hh = (hh1 + hh2) / 2;

    SL = 1 + 0.015 * pow(mean_LL - 50, 2) / pow(20 + pow(mean_LL - 50, 2), 0.5);
    SC = 1 + 0.045 * mean_CC;
    T = 1 - 0.17 * cos((mean_hh - 30) * PI / 180) + 0.24 * cos((2 * mean_hh) * PI / 180)
          + 0.32 * cos((3 * mean_hh + 6) * PI / 180) - 0.2 * cos((4 * mean_hh - 63) * PI / 180);
    SH = 1 + 0.015 * mean_CC * T;

    //------第四步--------
    //计算公式中的RT
    double mean_CC_pow7 = pow(mean_CC, 7);
    double RC = 2 * pow(mean_CC_pow7 / (mean_CC_pow7 + pow(25, 7)), 0.5);
    double delta_xita = 30 * exp(-pow((mean_hh - 275) / 25, 2));        //△θ 以°为单位
    RT = -sin((2 * delta_xita) * PI / 180) * RC;

    double L_item, C_item, H_item;
    L_item = delta_LL / (kL * SL);
    C_item = delta_CC / (kC * SC);
    H_item = delta_HH / (kH * SH);

    E00 = pow(L_item * L_item + C_item * C_item + H_item * H_item + RT * C_item * H_item, 0.5);

    return E00;
}

void FindBound::FindSameColorAreaFromImagePiexl(int tolerance, int diff_color_dist, int color_num)
{
    if(DEBUG_MODE)
    {
        std::cout << "    Start FindSameColorArea..." << std::endl;
    }

    ColorsSortedByArea.clear();

    color_tolerance = tolerance;
    different_color_dist = diff_color_dist;

    int seek_dist = 10;

    int r1 = 35;
    int g1 = 35;
    int b1 = 35;

    double color_dist_1;

    for(int xx = 0; xx < Image.rows; xx+=seek_dist)
    {
        if(DEBUG_MODE)
        {
            //std::cout << "xx : " << xx << " / " << Image.rows << std::endl;
        }


        for(int yy = 0; yy < Image.cols; yy+=seek_dist)
        {
            if(DEBUG_MODE)
            {
                //std::cout << "xx : " << xx << " / " << Image.rows << " || yy : " << yy << " / " << Image.cols << std::endl;
            }


            int r = Image.at<cv::Vec3b>(xx, yy)[0];
            int g = Image.at<cv::Vec3b>(xx, yy)[1];
            int b = Image.at<cv::Vec3b>(xx, yy)[2];

            color_dist_1 = sqrt((r - b1)*(r - b1) + (g - g1)*(g - g1) + (b - r1)*(b - r1));

            if(r + g + b == 0 || color_dist_1 < 190)
            {
                continue;
            }

            std::vector<int> rgb_hsv = rgb2hsv(r, g, b);
//                qDebug()<<"same color area hsv"<<rgb_hsv[0]<<rgb_hsv[1]<<rgb_hsv[2];
            int same_color_points_num = 0;

            for(int x = 0; x < Image.rows; x+=2)
            {
                for(int y = 0; y < Image.cols; y+=2)
                {
                    if(USE_HSV)
                    {
                        if(GetHSVDist(rgb_hsv, HSV_Color[x][y]) < color_tolerance && HSV_Color[x][y][2] > 128)
                        {
                            ++same_color_points_num;
                        }
                    }
                    else
                    {
                        double color_dist = 0;

                        color_dist += (Image.at<cv::Vec3b>(x, y)[0] - r) * (Image.at<cv::Vec3b>(x, y)[0] - r);
                        color_dist += (Image.at<cv::Vec3b>(x, y)[1] - g) * (Image.at<cv::Vec3b>(x, y)[1] - g);
                        color_dist += (Image.at<cv::Vec3b>(x, y)[2] - b) * (Image.at<cv::Vec3b>(x, y)[2] - b);

                        color_dist = sqrt(color_dist);

                        if(color_dist < color_tolerance && Image.at<cv::Vec3b>(x, y)[0] + Image.at<cv::Vec3b>(x, y)[1] + Image.at<cv::Vec3b>(x, y)[2] > 0)
                        {
                            ++same_color_points_num;
                        }
                    }
                }
            }

            int color_idx;
            for(color_idx = 0; color_idx < ColorsSortedByArea.size(); ++color_idx)
            {
                if(same_color_points_num > ColorsSortedByArea[color_idx][3])
                {
                    break;
                }
            }

            if(ColorsSortedByArea.size() == 0)
            {
                std::vector<int> insert_color;
                insert_color.resize(4);
                insert_color[0] = r;
                insert_color[1] = g;
                insert_color[2] = b;
                insert_color[3] = same_color_points_num;

                ColorsSortedByArea.emplace_back(insert_color);
                UpdateHSVColorSorted();
            }
            else if(color_idx < ColorsSortedByArea.size())
            {
                int color_check;

                for(color_check = 0; color_check < ColorsSortedByArea.size(); ++color_check)
                {
                    if(USE_HSV)
                    {
                        if(GetHSVDist(HSV_ColorSorted[color_check], rgb_hsv) < color_tolerance)
                        {
                            break;
                        }
                    }
                    else
                    {
                        double color_dist = 0;

                        color_dist += (ColorsSortedByArea[color_check][0] - r) * (ColorsSortedByArea[color_check][0] - r);
                        color_dist += (ColorsSortedByArea[color_check][1] - g) * (ColorsSortedByArea[color_check][1] - g);
                        color_dist += (ColorsSortedByArea[color_check][2] - b) * (ColorsSortedByArea[color_check][2] - b);

                        color_dist = sqrt(color_dist);

                        if(color_dist < different_color_dist)
                        {
                            break;
                        }
                    }
                }

                if(color_check > color_idx)
                {
                    if(color_check < ColorsSortedByArea.size())
                    {
                        for(int trans = color_check; trans > color_idx; --trans)
                        {
                            for(int idx = 0; idx < 4; ++idx)
                            {
                                ColorsSortedByArea[trans][idx] = ColorsSortedByArea[trans - 1][idx];
                            }
                        }
                        ColorsSortedByArea[color_idx][0] = r;
                        ColorsSortedByArea[color_idx][1] = g;
                        ColorsSortedByArea[color_idx][2] = b;
                        ColorsSortedByArea[color_idx][3] = same_color_points_num;
                        UpdateHSVColorSorted();
                    }
                    else
                    {
                        std::vector<int> insert_color;
                        insert_color.resize(4);
                        insert_color[0] = r;
                        insert_color[1] = g;
                        insert_color[2] = b;
                        insert_color[3] = same_color_points_num;

                        ColorsSortedByArea.insert(ColorsSortedByArea.begin() + color_idx, insert_color);
                        UpdateHSVColorSorted();
                    }
                }
            }
            if(ColorsSortedByArea.size() > color_num)
            {
                ColorsSortedByArea.pop_back();
                UpdateHSVColorSorted();
            }
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "    Finish FindSameColorArea..." << std::endl;
    }

}

void FindBound::FindSameColorAreaFromRGB(int tolerance, int diff_color_dist, int color_num)
{
    ColorsSortedByArea.clear();

    color_tolerance = tolerance;
    different_color_dist = diff_color_dist;

    int seek_dist = 10;

    for(int r = color_tolerance; r < 256 - color_tolerance; r+=color_tolerance)
    {
        if(DEBUG_MODE)
        {
            std::cout << "r : " << r << " / " << 256 << std::endl;
        }


        for(int g = color_tolerance; g < 256 - color_tolerance; g+=color_tolerance)
        {
            if(DEBUG_MODE)
            {
                //std::cout << "r : " << r << " / " << 256 << " || g : " << g << " / " << 256 << std::endl;
            }

            for(int b = color_tolerance; b < 256 - color_tolerance; b+=color_tolerance)
            {
                if(DEBUG_MODE)
                {
                    //std::cout << "r : " << r << " / " << 256 << " || g : " << g << " / " << 256 << " || b : " << b << " / " << 256 << std::endl;
                }

                std::vector<int> rgb_hsv = rgb2hsv(r, g, b);

                int same_color_points_num = 0;

                for(int x = 0; x < Image.rows; x+=3)
                {
                    for(int y = 0; y < Image.cols; y+=3)
                    {
                        if(USE_HSV)
                        {
                            if(GetHSVDist(rgb_hsv, HSV_Color[x][y]) < color_tolerance && HSV_Color[x][y][2] > 128)
                            {
                                ++same_color_points_num;
                            }
                        }
                        else
                        {
                            double color_dist = 0;

                            color_dist += (Image.at<cv::Vec3b>(x, y)[0] - r) * (Image.at<cv::Vec3b>(x, y)[0] - r);
                            color_dist += (Image.at<cv::Vec3b>(x, y)[1] - g) * (Image.at<cv::Vec3b>(x, y)[1] - g);
                            color_dist += (Image.at<cv::Vec3b>(x, y)[2] - b) * (Image.at<cv::Vec3b>(x, y)[2] - b);

                            color_dist = sqrt(color_dist);

                            if(color_dist < color_tolerance)
                            {
                                ++same_color_points_num;
                            }
                        }
                    }
                }

                int color_idx;
                for(color_idx = 0; color_idx < ColorsSortedByArea.size(); ++color_idx)
                {
                    if(same_color_points_num > ColorsSortedByArea[color_idx][3])
                    {
                        break;
                    }
                }

                if(ColorsSortedByArea.size() == 0)
                {
                    std::vector<int> insert_color;
                    insert_color.resize(4);
                    insert_color[0] = r;
                    insert_color[1] = g;
                    insert_color[2] = b;
                    insert_color[3] = same_color_points_num;

                    ColorsSortedByArea.emplace_back(insert_color);
                    UpdateHSVColorSorted();
                }
                else if(color_idx < ColorsSortedByArea.size())
                {
                    int color_check;

                    for(color_check = 0; color_check < ColorsSortedByArea.size(); ++color_check)
                    {
                        if(USE_HSV)
                        {
                            if(GetHSVDist(HSV_ColorSorted[color_check], rgb_hsv) < color_tolerance)
                            {
                                break;
                            }
                        }
                        else
                        {
                            double color_dist = 0;

                            color_dist += (ColorsSortedByArea[color_check][0] - r) * (ColorsSortedByArea[color_check][0] - r);
                            color_dist += (ColorsSortedByArea[color_check][1] - g) * (ColorsSortedByArea[color_check][1] - g);
                            color_dist += (ColorsSortedByArea[color_check][2] - b) * (ColorsSortedByArea[color_check][2] - b);

                            color_dist = sqrt(color_dist);

                            if(color_dist < different_color_dist)
                            {
                                break;
                            }
                        }
                    }

                    if(color_check > color_idx)
                    {
                        if(color_check < ColorsSortedByArea.size())
                        {
                            for(int trans = color_check; trans > color_idx; --trans)
                            {
                                for(int idx = 0; idx < 4; ++idx)
                                {
                                    ColorsSortedByArea[trans][idx] = ColorsSortedByArea[trans - 1][idx];
                                }
                            }
                            ColorsSortedByArea[color_idx][0] = r;
                            ColorsSortedByArea[color_idx][1] = g;
                            ColorsSortedByArea[color_idx][2] = b;
                            ColorsSortedByArea[color_idx][3] = same_color_points_num;
                            UpdateHSVColorSorted();
                        }
                        else
                        {
                            std::vector<int> insert_color;
                            insert_color.resize(4);
                            insert_color[0] = r;
                            insert_color[1] = g;
                            insert_color[2] = b;
                            insert_color[3] = same_color_points_num;

                            ColorsSortedByArea.insert(ColorsSortedByArea.begin() + color_idx, insert_color);
                            UpdateHSVColorSorted();
                        }
                    }
                }
                if(ColorsSortedByArea.size() > color_num)
                {
                    ColorsSortedByArea.pop_back();
                    UpdateHSVColorSorted();
                }
            }
        }
    }
}

void FindBound::FindSameColorPoints(int min_size, int delta_dist)
{
    SameColorPointSet.clear();

    delta_pixel = delta_dist;

    for(int x = 0; x < Image.rows; x+=delta_pixel)
    {
        for(int y = 0; y < Image.cols; y+=delta_pixel)
        {
            if((Image.at<cv::Vec3b>(x, y)[0] == 0 && Image.at<cv::Vec3b>(x, y)[1] == 0 && Image.at<cv::Vec3b>(x, y)[2] == 255) || (Image.at<cv::Vec3b>(x, y)[0] == 0 && Image.at<cv::Vec3b>(x, y)[1] == 255 && Image.at<cv::Vec3b>(x, y)[2] == 0))
            {
                bool have_inserted = false;

                for(int i = 0; i < SameColorPointSet.size(); ++i)
                {
                    if(have_inserted)
                    {
                        break;
                    }

                    for(int j = 1; j < SameColorPointSet[i].size(); ++j)
                    {
                        double dist = (SameColorPointSet[i][j][0] - x) * (SameColorPointSet[i][j][0] - x) + (SameColorPointSet[i][j][1] - y) * (SameColorPointSet[i][j][1] - y);

                        dist = sqrt(dist);

                        if(dist < 1.5 * delta_pixel)
                        {
                            if(Image.at<cv::Vec3b>(x, y)[0] == SameColorPointSet[i][0][0] && Image.at<cv::Vec3b>(x, y)[1] == SameColorPointSet[i][0][1] && Image.at<cv::Vec3b>(x, y)[2] == SameColorPointSet[i][0][2])
                            {
                                std::vector<int> point;
                                point.resize(2);
                                point[0] = x;
                                point[1] = y;

                                SameColorPointSet[i].emplace_back(point);

                                have_inserted = true;

                                break;
                            }
                        }
                    }
                }

                if(!have_inserted)
                {
                    std::vector<std::vector<int>> same_color_point;
                    same_color_point.resize(2);
                    same_color_point[0].resize(3);
                    same_color_point[1].resize(2);

                    same_color_point[0][0] = Image.at<cv::Vec3b>(x, y)[0];
                    same_color_point[0][1] = Image.at<cv::Vec3b>(x, y)[1];
                    same_color_point[0][2] = Image.at<cv::Vec3b>(x, y)[2];

                    same_color_point[1][0] = x;
                    same_color_point[1][1] = y;

                    SameColorPointSet.emplace_back(same_color_point);
                }
            }
        }
    }

    MergeSameColorPoints();

    for(int i = SameColorPointSet.size() - 1; i >=0; --i)
    {
        if(SameColorPointSet[i].size() - 1 < min_size)
        {
            SameColorPointSet.erase(SameColorPointSet.begin() + i);
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "======== SameColorPointSet.Size : " << SameColorPointSet.size() << std::endl;
        for(int i = 0; i < SameColorPointSet.size(); ++i)
        {
            std::cout << "Color " << i << " : (" << SameColorPointSet[i][0][0] << "," << SameColorPointSet[i][0][1] << "," << SameColorPointSet[i][0][2] << ") , Set Size : " << SameColorPointSet[i].size() - 1 << std::endl;
        }
    }

    /*
    for(int i = 0; i < SameColorPointSet.size(); ++i)
    {
        for(int j = 1; j < SameColorPointSet[i].size(); ++j)
        {
            for(int k = 0; k < 3; ++k)
            {
                Image_to_draw.at<cv::Vec3b>(SameColorPointSet[i][j][0], SameColorPointSet[i][j][1])[k] = SameColorPointSet[i][0][k];
            }
        }
    }
    */
}

void FindBound::MergeSameColorPoints()
{
    for(int i = SameColorPointSet.size() - 1; i > 0; --i)
    {
        bool have_merged = false;

        for(int j = 0; j < i; ++j)
        {
            if(have_merged)
            {
                break;
            }

            if(SameColorPointSet[i][0][0] == SameColorPointSet[j][0][0] && SameColorPointSet[i][0][1] == SameColorPointSet[j][0][1] && SameColorPointSet[i][0][2] == SameColorPointSet[j][0][2])
            {
                for(int ii = 1; ii < SameColorPointSet[i].size(); ++ii)
                {
                    if(have_merged)
                    {
                        break;
                    }

                    for(int jj = 1; jj < SameColorPointSet[j].size(); ++jj)
                    {
                        double dist = 0;
                        dist += (SameColorPointSet[i][ii][0] - SameColorPointSet[j][jj][0]) * (SameColorPointSet[i][ii][0] - SameColorPointSet[j][jj][0]);
                        dist += (SameColorPointSet[i][ii][1] - SameColorPointSet[j][jj][1]) * (SameColorPointSet[i][ii][1] - SameColorPointSet[j][jj][1]);
                        dist = sqrt(dist);

                        if(dist < 1.5 * delta_pixel)
                        {
                            for(int k = 1; k < SameColorPointSet[i].size(); ++k)
                            {
                                SameColorPointSet[j].emplace_back(SameColorPointSet[i][k]);
                            }

                            SameColorPointSet.erase(SameColorPointSet.begin() + i);

                            have_merged = true;

                            break;
                        }
                    }
                }
            }
        }
    }
}

void FindBound::FindSameColorBoundPoints(int min_size, int delta_dist)
{
    SameColorBoundPointSet.clear();

    FindSameColorPoints(min_size);

    SameColorBoundPointSet.resize(SameColorPointSet.size());

    for(int i = 0; i < SameColorBoundPointSet.size(); ++i)
    {
        SameColorBoundPointSet[i].resize(1);

        SameColorBoundPointSet[i][0].resize(3);

        for(int j = 0; j < 3; ++j)
        {
            SameColorBoundPointSet[i][0][j] = SameColorPointSet[i][0][j];
        }
    }

    for(int i = 0; i < SameColorPointSet.size(); ++i)
    {
        for(int j = 0; j < SameColorPointSet.size(); ++j)
        {
            if(j != i)
            {
                for(int ii = 1; ii < SameColorPointSet[i].size(); ++ii)
                {
                    for(int jj = 1; jj < SameColorPointSet[j].size(); ++jj)
                    {
                        double dist = 0;
                        dist += (SameColorPointSet[i][ii][0] - SameColorPointSet[j][jj][0]) * (SameColorPointSet[i][ii][0] - SameColorPointSet[j][jj][0]);
                        dist += (SameColorPointSet[i][ii][1] - SameColorPointSet[j][jj][1]) * (SameColorPointSet[i][ii][1] - SameColorPointSet[j][jj][1]);
                        dist = sqrt(dist);

                        if(dist < 1.5 * delta_pixel)
                        {
                            SameColorBoundPointSet[i].emplace_back(SameColorPointSet[i][ii]);

                            break;
                        }
                    }
                }
            }
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "======== SameColorBoundPointSet.Size : " << SameColorBoundPointSet.size() << std::endl;
        for(int i = 0; i < SameColorBoundPointSet.size(); ++i)
        {
            std::cout << "Color " << i << " : (" << SameColorBoundPointSet[i][0][0] << "," << SameColorBoundPointSet[i][0][1] << "," << SameColorBoundPointSet[i][0][2] << ") , Set Size : " << SameColorBoundPointSet[i].size() - 1 << std::endl;
        }
    }
}

void FindBound::FindSameColorBoundPointsWithBoundColor(int min_size, int delta_dist)
{
    if(DEBUG_MODE)
    {
        std::cout << "    Start FindSameColorBoundPoints..." << std::endl;
    }

    SameColorBoundPointSet.clear();

    FindSameColorPoints(min_size);

    SameColorBoundPointSet.resize(SameColorPointSet.size());

    for(int i = 0; i < SameColorBoundPointSet.size(); ++i)
    {
        SameColorBoundPointSet[i].resize(1);

        SameColorBoundPointSet[i][0].resize(3);

        for(int j = 0; j < 3; ++j)
        {
            SameColorBoundPointSet[i][0][j] = SameColorPointSet[i][0][j];
        }
    }

    for(int i = 0; i < SameColorPointSet.size(); ++i)
    {
        for(int j = 1; j < SameColorPointSet[i].size(); ++j)
        {
            bool NEAR_TO_BOUND = false;

            for(int k = 1; k < delta_dist; ++k)
            {
                if(NEAR_TO_BOUND)
                {
                    break;
                }
                for(int kk = 0; kk < directions.size(); ++kk)
                {
                    if(0 <= SameColorPointSet[i][j][0] + int(k*directions[kk][0]) && SameColorPointSet[i][j][0] + int(k*directions[kk][0]) < Image.rows && 0 <= SameColorPointSet[i][j][1] + int(k*directions[kk][1]) && SameColorPointSet[i][j][1] + int(k*directions[kk][1]) < Image.cols)
                    {
                        if(Image.at<cv::Vec3b>(SameColorPointSet[i][j][0] + int(k*directions[kk][0]), SameColorPointSet[i][j][1] + int(k*directions[kk][1]))[0] + Image.at<cv::Vec3b>(SameColorPointSet[i][j][0] + int(k*directions[kk][0]), SameColorPointSet[i][j][1] + int(k*directions[kk][1]))[1] + Image.at<cv::Vec3b>(SameColorPointSet[i][j][0] + int(k*directions[kk][0]), SameColorPointSet[i][j][1] + int(k*directions[kk][1]))[2] == 0)
                        {
                            NEAR_TO_BOUND = true;

                            break;
                        }
                    }
                }
            }

            if(NEAR_TO_BOUND)
            {
                SameColorBoundPointSet[i].emplace_back(SameColorPointSet[i][j]);
            }
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "======== SameColorBoundPointSet.Size : " << SameColorBoundPointSet.size() << std::endl;
        for(int i = 0; i < SameColorBoundPointSet.size(); ++i)
        {
            std::cout << "Color " << i << " : (" << SameColorBoundPointSet[i][0][0] << "," << SameColorBoundPointSet[i][0][1] << "," << SameColorBoundPointSet[i][0][2] << ") , Set Size : " << SameColorBoundPointSet[i].size() - 1 << std::endl;
        }
        std::cout << "    Finish FindSameColorBoundPoints..." << std::endl;
    }
}

bool FindBound::IsDifferentColorOf(int bound_point_color_idx, int color_x, int color_y)
{
    if(color_x < 0 || color_x >= Image.rows || color_y < 0 || color_y >= Image.cols)
    {
        return false;
    }

    if(Image.at<cv::Vec3b>(color_x, color_y)[0] == SameColorBoundPointSet[bound_point_color_idx][0][0] && Image.at<cv::Vec3b>(color_x, color_y)[1] == SameColorBoundPointSet[bound_point_color_idx][0][1] && Image.at<cv::Vec3b>(color_x, color_y)[2] == SameColorBoundPointSet[bound_point_color_idx][0][2])
    {
        return false;
    }

    for(int i = 0; i < SameColorBoundPointSet.size(); ++i)
    {
        if(i != bound_point_color_idx)
        {
            if(Image.at<cv::Vec3b>(color_x, color_y)[0] == SameColorBoundPointSet[i][0][0] && Image.at<cv::Vec3b>(color_x, color_y)[1] == SameColorBoundPointSet[i][0][1] && Image.at<cv::Vec3b>(color_x, color_y)[2] == SameColorBoundPointSet[i][0][2])
            {
                return true;
            }
        }
    }

    return false;
}

void FindBound::FindRealBoundPoints(int delta_pixel)
{
    if(DEBUG_MODE)
    {
        std::cout << "    Start FindRealBoundPoints..." << std::endl;
    }

    for(int i = 0; i < SameColorBoundPointSet.size(); ++i)
    {
        for(int j = SameColorBoundPointSet[i].size() - 1; j > 0; --j)
        {
            bool CROSSED_BOUND = false;
            bool CROSSED_OTHER_COLOR = false;

            for(int ii = 0; ii < directions.size(); ++ii)
            {
                if(CROSSED_OTHER_COLOR)
                {
                    break;
                }

                CROSSED_BOUND = false;

                for(int k = 1; k < delta_pixel; ++k)
                {
                    if(CROSSED_OTHER_COLOR)
                    {
                        break;
                    }

                    if(0 <= SameColorBoundPointSet[i][j][0] + int(k*directions[ii][0]) && SameColorBoundPointSet[i][j][0] + int(k*directions[ii][0]) < Image.rows && 0 <= SameColorBoundPointSet[i][j][1] + int(k*directions[ii][1]) && SameColorBoundPointSet[i][j][1] + int(k*directions[ii][1]) < Image.cols)
                    {
                        if(!CROSSED_BOUND && Image.at<cv::Vec3b>(SameColorBoundPointSet[i][j][0] + int(k*directions[ii][0]), SameColorBoundPointSet[i][j][1] + int(k*directions[ii][1]))[0] + Image.at<cv::Vec3b>(SameColorBoundPointSet[i][j][0] + int(k*directions[ii][0]), SameColorBoundPointSet[i][j][1] + int(k*directions[ii][1]))[1] + Image.at<cv::Vec3b>(SameColorBoundPointSet[i][j][0] + int(k*directions[ii][0]), SameColorBoundPointSet[i][j][1] + int(k*directions[ii][1]))[2] == 0)
                        {
                            CROSSED_BOUND = true;
                        }
                        else if(CROSSED_BOUND && IsDifferentColorOf(i, SameColorBoundPointSet[i][j][0] + int(k*directions[ii][0]), SameColorBoundPointSet[i][j][1] + int(k*directions[ii][1])))
                        {
                            CROSSED_OTHER_COLOR = true;
                        }
                    }
                }
            }

            if(!CROSSED_OTHER_COLOR)
            {
                SameColorBoundPointSet[i].erase(SameColorBoundPointSet[i].begin()+j);
            }
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "======== RealBoundPointSet.Size : " << SameColorBoundPointSet.size() << std::endl;
        for(int i = 0; i < SameColorBoundPointSet.size(); ++i)
        {
            std::cout << "Color " << i << " : (" << SameColorBoundPointSet[i][0][0] << "," << SameColorBoundPointSet[i][0][1] << "," << SameColorBoundPointSet[i][0][2] << ") , Set Size : " << SameColorBoundPointSet[i].size() - 1 << std::endl;
        }
        std::cout << "    Finish FindRealBoundPoints..." << std::endl;
    }
}

void FindBound::GetReliableBoundPoints(int delta_dist, int max_reliable)
{
    if(DEBUG_MODE)
    {
        std::cout << "    Start GetReliableBoundPoints..." << std::endl;
    }

    ReliableBoundPointSet.clear();

    ReliableBoundPointSet.resize(SameColorBoundPointSet.size());
    for(int i = 0; i < ReliableBoundPointSet.size(); ++i)
    {
        ReliableBoundPointSet[i].emplace_back(SameColorBoundPointSet[i][0]);
    }

    HaveChoosedAtBoundPointSet.resize(SameColorBoundPointSet.size());
    for(int i = 0; i < HaveChoosedAtBoundPointSet.size(); ++i)
    {
        HaveChoosedAtBoundPointSet[i].resize(SameColorBoundPointSet[i].size() - 1);
        for(int j = 0; j < HaveChoosedAtBoundPointSet[i].size(); ++j)
        {
            HaveChoosedAtBoundPointSet[i][j] = false;
        }
    }

    for(int i = 0; i < SameColorBoundPointSet.size(); ++i)
    {
        for(int j = 0; j < SameColorBoundPointSet.size(); ++j)
        {
            if(SameColorBoundPointSet[i][0][0] == SameColorBoundPointSet[j][0][0] && SameColorBoundPointSet[i][0][1] == SameColorBoundPointSet[j][0][1] && SameColorBoundPointSet[i][0][2] == SameColorBoundPointSet[j][0][2])
            {
                continue;
            }

            for(int ii = 1; ii < SameColorBoundPointSet[i].size(); ++ii)
            {
                bool have_saved = false;

                for(int jj = 1; jj < SameColorBoundPointSet[j].size(); ++jj)
                {
                    if(have_saved)
                    {
                        break;
                    }

                    double dist = 0;
                    dist += (SameColorBoundPointSet[i][ii][0] - SameColorBoundPointSet[j][jj][0]) * (SameColorBoundPointSet[i][ii][0] - SameColorBoundPointSet[j][jj][0]);
                    dist += (SameColorBoundPointSet[i][ii][1] - SameColorBoundPointSet[j][jj][1]) * (SameColorBoundPointSet[i][ii][1] - SameColorBoundPointSet[j][jj][1]);
                    dist = sqrt(dist);

                    if(dist < delta_dist * max_reliable)
                    {
                        ReliableBoundPointSet[i].emplace_back(SameColorBoundPointSet[i][ii]);
                        HaveChoosedAtBoundPointSet[i][ii] = true;

                        have_saved = true;
                    }
                }
            }
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "======== ReliableBoundPointSet.Size : " << ReliableBoundPointSet.size() << std::endl;
        for(int i = 0; i < ReliableBoundPointSet.size(); ++i)
        {
            std::cout << "Color " << i << " : (" << ReliableBoundPointSet[i][0][0] << "," << ReliableBoundPointSet[i][0][1] << "," << ReliableBoundPointSet[i][0][2] << ") , Set Size : " << ReliableBoundPointSet[i].size() - 1 << std::endl;
        }
        std::cout << "    Finish GetReliableBoundPoints..." << std::endl;
    }
}

void FindBound::ExpandReliableBoundPoints(int expand_dist)
{
    if(DEBUG_MODE)
    {
        std::cout << "    Start ExpandReliableBoundPoints..." << std::endl;
    }

    bool point_sets_changed = true;

    while(point_sets_changed)
    {
        point_sets_changed = false;

        for(int i = 0; i < SameColorBoundPointSet.size(); ++i)
        {
            if(point_sets_changed)
            {
                break;
            }

            for(int ii = 1; ii < SameColorBoundPointSet[i].size(); ++ii)
            {
                if(point_sets_changed)
                {
                    break;
                }

                if(!HaveChoosedAtBoundPointSet[i][ii])
                {
                    for(int j = 0; j < ReliableBoundPointSet.size(); ++j)
                    {
                        if(point_sets_changed)
                        {
                            break;
                        }

                        for(int jj = 1; jj < ReliableBoundPointSet[j].size(); ++jj)
                        {
                            double dist = 0;
                            dist += (SameColorBoundPointSet[i][ii][0] - ReliableBoundPointSet[j][jj][0]) * (SameColorBoundPointSet[i][ii][0] - ReliableBoundPointSet[j][jj][0]);
                            dist += (SameColorBoundPointSet[i][ii][1] - ReliableBoundPointSet[j][jj][1]) * (SameColorBoundPointSet[i][ii][1] - ReliableBoundPointSet[j][jj][1]);
                            dist = sqrt(dist);

                            if(dist <= expand_dist)
                            {
                                ReliableBoundPointSet[j].emplace_back(SameColorBoundPointSet[i][ii]);
                                HaveChoosedAtBoundPointSet[i][ii] = true;

                                point_sets_changed = true;

                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "    Finish ExpandReliableBoundPoints..." << std::endl;
    }
}

int FindBound::GetNeighboorNum(int x, int y, int max_dist)
{
    int neighboor_point_num = -1;

    for(int i = 0; i < ReliableBoundPointSet.size(); ++i)
    {
        for(int ii = 1; ii < ReliableBoundPointSet[i].size(); ++ii)
        {
            double dist = 0;
            dist += (ReliableBoundPointSet[i][ii][0] - x) * (ReliableBoundPointSet[i][ii][0] - x);
            dist += (ReliableBoundPointSet[i][ii][1] - y) * (ReliableBoundPointSet[i][ii][1] - y);
            dist = sqrt(dist);

            if(dist <= max_dist)
            {
                ++neighboor_point_num;
            }
        }
    }

    return neighboor_point_num;
}

void FindBound::RemoveUselessPoints(int min_neighboor_point_num)
{
    if(DEBUG_MODE)
    {
        std::cout << "    Start RemoveUselessPoints..." << std::endl;
    }

    std::vector<std::vector<int>> points_to_delete;

    for(int i = 0; i < ReliableBoundPointSet.size(); ++i)
    {
        for(int ii = ReliableBoundPointSet[i].size() - 1; ii > 0; --ii)
        {
            if(GetNeighboorNum(ReliableBoundPointSet[i][ii][0], ReliableBoundPointSet[i][ii][1]) < min_neighboor_point_num)
            {
                std::vector<int> point_to_delete;
                point_to_delete.resize(2);
                point_to_delete[0] = i;
                point_to_delete[1] = ii;

                points_to_delete.emplace_back(point_to_delete);
            }
        }
    }

    for(int i = 0; i < points_to_delete.size(); ++i)
    {
        ReliableBoundPointSet[points_to_delete[i][0]].erase(ReliableBoundPointSet[points_to_delete[i][0]].begin() + points_to_delete[i][1]);
    }

    if(DEBUG_MODE)
    {
        std::cout << "    Finish RemoveUselessPoints..." << std::endl;
    }
}

void FindBound::GetFinalReliableBoundPoints(int delta_dist)
{
    FinalReliableBoundPointSet.clear();

    ReliableBoundPointUsed.resize(ReliableBoundPointSet.size());
    for(int i = 0; i < ReliableBoundPointSet.size(); ++i)
    {
        ReliableBoundPointUsed[i].resize(ReliableBoundPointSet[i].size());
        for(int j = 0; j < ReliableBoundPointUsed[i].size(); ++j)
        {
            ReliableBoundPointUsed[i][j] = false;
        }
    }

    for(int i = 0; i < ReliableBoundPointSet.size(); ++i)
    {
        for(int ii = 1; ii < ReliableBoundPointSet[i].size(); ++ii)
        {
            for(int j = i + 1; j < ReliableBoundPointSet.size(); ++j)
            {
                if(ReliableBoundPointSet[i][0][0] == ReliableBoundPointSet[j][0][0] && ReliableBoundPointSet[i][0][1] == ReliableBoundPointSet[j][0][1] && ReliableBoundPointSet[i][0][2] == ReliableBoundPointSet[j][0][2])
                {
                    continue;
                }

                for(int jj = 1; jj < ReliableBoundPointSet[j].size(); ++jj)
                {
                    double dist = 0;
                    dist += (ReliableBoundPointSet[i][ii][0] - ReliableBoundPointSet[j][jj][0]) * (ReliableBoundPointSet[i][ii][0] - ReliableBoundPointSet[j][jj][0]);
                    dist += (ReliableBoundPointSet[i][ii][1] - ReliableBoundPointSet[j][jj][1]) * (ReliableBoundPointSet[i][ii][1] - ReliableBoundPointSet[j][jj][1]);
                    dist = sqrt(dist);

                    if(dist <= delta_dist)
                    {
                        double x1 = (double)ReliableBoundPointSet[i][ii][0];
                        double y1 = (double)ReliableBoundPointSet[i][ii][1];
                        double x2 = (double)ReliableBoundPointSet[j][jj][0];
                        double y2 = (double)ReliableBoundPointSet[j][jj][1];

                        double dx = 0;
                        double dy = 0;

                        int step_num = 0;

                        if(x1 == x2)
                        {
                            dx = 0;
                            if(y1 < y2)
                            {
                                dy = 1;
                                step_num = y2 - y1;
                            }
                            else
                            {
                                dy = -1;
                                step_num = y1 - y2;
                            }
                        }
                        else if(y1 == y2)
                        {
                            dy = 0;
                            if(x1 < x2)
                            {
                                dx = 1;
                                step_num = x2 - x1;
                            }
                            else
                            {
                                dx = -1;
                                step_num = x1 - x2;
                            }
                        }
                        else if(fabs(x2 - x1) < fabs(y2 - y1))
                        {
                            if(x1 < x2)
                            {
                                dx = 1;
                                dy = (y2 - y1) / (x2 - x1);
                                step_num = x2 - x1;
                            }
                            else
                            {
                                dx = -1;
                                dy = (y2 - y1) / (x1 - x2);
                                step_num = x1 - x2;
                            }
                        }
                        else
                        {
                            if(y1 < y2)
                            {
                                dy = 1;
                                dx = (x2 - x1) / (y2 - y1);
                                step_num = y2 - y1;
                            }
                            else
                            {
                                dy = -1;
                                dx = (x2 - x1) / (y1 - y2);
                                step_num = y1 - y2;
                            }
                        }

                        double crossed_bound_point_x = 0;
                        double crossed_bound_point_y = 0;
                        int crossed_bound_point_num = 0;

                        for(int step = 1; step < step_num; ++step)
                        {
                            x1 += dx;
                            y1 += dy;

                            if(Image.at<cv::Vec3b>((int)x1, (int)y1)[0] + Image.at<cv::Vec3b>((int)x1, (int)y1)[1] + Image.at<cv::Vec3b>((int)x1, (int)y1)[2] == 0)
                            {
                                crossed_bound_point_x += x1;
                                crossed_bound_point_y += y1;
                                ++crossed_bound_point_num;
                            }
                        }
                        if(crossed_bound_point_num > 0)
                        {
                            crossed_bound_point_x /= crossed_bound_point_num;
                            crossed_bound_point_y /= crossed_bound_point_num;

                            std::vector<int> bound_point;
                            bound_point.resize(2);
                            bound_point[0] = (int)crossed_bound_point_x;
                            bound_point[1] = (int)crossed_bound_point_y;

                            FinalReliableBoundPointSet.emplace_back(bound_point);

                            ReliableBoundPointUsed[i][ii] = true;
                            ReliableBoundPointUsed[j][jj] = true;
                        }
                    }
                }
            }
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "======== FinalReliableBoundPointSet.Size : " << FinalReliableBoundPointSet.size() << std::endl;
    }
}

void FindBound::GetFinalExpandBoundPoints(int max_dist)
{
    FinalExpandBoundPointSet.clear();

    for(int i = 0; i < ReliableBoundPointSet.size(); ++i)
    {
        for(int ii = 1; ii < ReliableBoundPointSet[i].size(); ++ii)
        {
            if(!ReliableBoundPointUsed[i][ii])
            {
                bool find_bound = false;

                double bound_point_x = 0;
                double bound_point_y = 0;
                int bound_point_num = 0;

                for(int k = 1; k < max_dist; ++k)
                {
                    for(int direction = 0; direction < directions.size(); ++direction)
                    {
                        if(0 <= ReliableBoundPointSet[i][ii][0] + int(k*directions[direction][0]) && ReliableBoundPointSet[i][ii][0] + int(k*directions[direction][0]) < Image.rows && 0 <= ReliableBoundPointSet[i][ii][1] + int(k*directions[direction][1]) && ReliableBoundPointSet[i][ii][1] + int(k*directions[direction][1]) < Image.cols)
                        {
                            if(Image.at<cv::Vec3b>(ReliableBoundPointSet[i][ii][0] + int(k*directions[direction][0]), ReliableBoundPointSet[i][ii][1] + int(k*directions[direction][1]))[0] + Image.at<cv::Vec3b>(ReliableBoundPointSet[i][ii][0] + int(k*directions[direction][0]), ReliableBoundPointSet[i][ii][1] + int(k*directions[direction][1]))[1] + Image.at<cv::Vec3b>(ReliableBoundPointSet[i][ii][0] + int(k*directions[direction][0]), ReliableBoundPointSet[i][ii][1] + int(k*directions[direction][1]))[2] == 0)
                            {
                                bound_point_x += ReliableBoundPointSet[i][ii][0] + int(k*directions[direction][0]);
                                bound_point_y += ReliableBoundPointSet[i][ii][1] + int(k*directions[direction][1]);
                                ++bound_point_num;

                                find_bound = true;
                            }
                        }
                    }
                }

                if(find_bound)
                {
                    bound_point_x /= bound_point_num;
                    bound_point_y /= bound_point_num;

                    std::vector<int> bound_point;
                    bound_point.resize(2);
                    bound_point[0] = (int)bound_point_x;
                    bound_point[1] = (int)bound_point_y;

                    double dist = 10000;

                    for(int j = 0; j < FinalReliableBoundPointSet.size(); ++j)
                    {
                        double current_dist = 0;
                        current_dist += (FinalReliableBoundPointSet[j][0] - bound_point[0]) * (FinalReliableBoundPointSet[j][0] - bound_point[0]);
                        current_dist += (FinalReliableBoundPointSet[j][1] - bound_point[1]) * (FinalReliableBoundPointSet[j][1] - bound_point[1]);
                        current_dist = sqrt(current_dist);

                        if(current_dist < dist)
                        {
                            dist = current_dist;
                        }
                    }

                    for(int j = 0; j < FinalExpandBoundPointSet.size(); ++j)
                    {
                        double current_dist = 0;
                        current_dist += (FinalExpandBoundPointSet[j][0] - bound_point[0]) * (FinalExpandBoundPointSet[j][0] - bound_point[0]);
                        current_dist += (FinalExpandBoundPointSet[j][1] - bound_point[1]) * (FinalExpandBoundPointSet[j][1] - bound_point[1]);
                        current_dist = sqrt(current_dist);

                        if(current_dist < dist)
                        {
                            dist = current_dist;
                        }
                    }

                    if(dist <= 15)
                    {
                        FinalExpandBoundPointSet.emplace_back(bound_point);

                        ReliableBoundPointUsed[i][ii] = true;
                    }
                }
            }
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "======== FinalExpandBoundPointSet.Size : " << FinalExpandBoundPointSet.size() << std::endl;
    }
}

void FindBound::GetFinalBoundPoints(int min_dist_to_image_bound)
{
    FinalBoundPointSet.clear();

    for(int i = 0; i < FinalReliableBoundPointSet.size(); ++i)
    {
        FinalBoundPointSet.emplace_back(FinalReliableBoundPointSet[i]);
    }

    for(int i = 0; i < FinalExpandBoundPointSet.size(); ++i)
    {
        FinalBoundPointSet.emplace_back(FinalExpandBoundPointSet[i]);
    }

    for(int i = FinalBoundPointSet.size() - 1; i >= 0; --i)
    {
        if(FinalBoundPointSet[i][0] <= min_dist_to_image_bound || FinalBoundPointSet[i][0] >= Image.rows - min_dist_to_image_bound || FinalBoundPointSet[i][1] <= min_dist_to_image_bound || FinalBoundPointSet[i][1] >= Image.cols - min_dist_to_image_bound)
        {
            FinalBoundPointSet.erase(FinalBoundPointSet.begin() + i);
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "======== FinalBoundPointSet.Size : " << FinalBoundPointSet.size() << std::endl;
    }
}

void FindBound::UpdateFinalBoundPointsWithSobel(int search_dist)
{
    FinalBoundPointSetWithSobel.clear();

    for(int i = 0; i < FinalBoundPointSet.size(); ++i)
    {
        bool have_inserted = false;

        for(int k = 0; k < search_dist; ++k)
        {
            if(have_inserted)
            {
                break;
            }

            for(int direction = 0; direction < directions.size(); ++direction)
            {
                if(have_inserted)
                {
                    break;
                }

                if(0 <= FinalBoundPointSet[i][0] + int(k*directions[direction][0]) && FinalBoundPointSet[i][0] + int(k*directions[direction][0]) < Image.rows && 0 <= FinalBoundPointSet[i][1] + int(k*directions[direction][1]) && FinalBoundPointSet[i][1] + int(k*directions[direction][1]) < Image.cols)
                {
                    cv::Vec3b color = g_dstImage.at<cv::Vec3b>(FinalBoundPointSet[i][0] + int(k*directions[direction][0]), FinalBoundPointSet[i][1] + int(k*directions[direction][1]));

                    int color_gray = (color[0] + color[1] + color[2]) / 3;

                    if((10 < color_gray && color_gray < 150) && 1.5*color[0] < color[2] && color[2] < 3*color[0])
                    {
                        FinalBoundPointSetWithSobel.emplace_back(FinalBoundPointSet[i]);

                        have_inserted = true;

                        break;
                    }
                }
            }
        }
    }
}

void FindBound::RemoveSigleFinalPoints(int search_dist, int min_group_num)
{
    bool have_changed = true;

    while(have_changed)
    {
        have_changed = false;

        for(int i = FinalBoundPointSetWithSobel.size() - 1; i >= 0; --i)
        {
            if(have_changed)
            {
                break;
            }

            int neighboor_final_points_num = 0;

            for(int j = 0; j < FinalBoundPointSetWithSobel.size(); ++j)
            {
                double dist = 0;
                dist += (FinalBoundPointSetWithSobel[i][0] - FinalBoundPointSetWithSobel[j][0]) * (FinalBoundPointSetWithSobel[i][0] - FinalBoundPointSetWithSobel[j][0]);
                dist += (FinalBoundPointSetWithSobel[i][1] - FinalBoundPointSetWithSobel[j][1]) * (FinalBoundPointSetWithSobel[i][1] - FinalBoundPointSetWithSobel[j][1]);
                dist = sqrt(dist);

                if(dist <= search_dist)
                {
                    ++neighboor_final_points_num;
                }
            }

            if(neighboor_final_points_num - 1 < min_group_num)
            {
                FinalBoundPointSetWithSobel.erase(FinalBoundPointSetWithSobel.begin() + i);

                have_changed = true;
            }
        }
    }
}

void FindBound::RemoveSigleMergedFinalPoints(int search_dist, int min_group_num)
{
    bool have_changed = true;

    while(have_changed)
    {
        have_changed = false;

        for(int i = MergedFinalBoundPointSetWithSobel.size() - 1; i >= 0; --i)
        {
            if(have_changed)
            {
                break;
            }

            int neighboor_final_points_num = 0;

            for(int j = 0; j < MergedFinalBoundPointSetWithSobel.size(); ++j)
            {
                double dist = 0;
                dist += (MergedFinalBoundPointSetWithSobel[i][0] - MergedFinalBoundPointSetWithSobel[j][0]) * (MergedFinalBoundPointSetWithSobel[i][0] - MergedFinalBoundPointSetWithSobel[j][0]);
                dist += (MergedFinalBoundPointSetWithSobel[i][1] - MergedFinalBoundPointSetWithSobel[j][1]) * (MergedFinalBoundPointSetWithSobel[i][1] - MergedFinalBoundPointSetWithSobel[j][1]);
                dist = sqrt(dist);

                if(dist <= search_dist)
                {
                    ++neighboor_final_points_num;
                }
            }

            if(neighboor_final_points_num - 1 < min_group_num)
            {
                MergedFinalBoundPointSetWithSobel.erase(MergedFinalBoundPointSetWithSobel.begin() + i);

                have_changed = true;
            }
        }
    }
}

void FindBound::RemoveSigleBoundMergedFinalPoints(int search_dist, int min_group_num, int max_bound_dist)
{
    bool have_changed = true;

    while(have_changed)
    {
        have_changed = false;

        for(int i = MergedFinalBoundPointSetWithSobel.size() - 1; i >= 0; --i)
        {
            if(have_changed)
            {
                break;
            }

            if(MergedFinalBoundPointSetWithSobel[i][0] < max_bound_dist || MergedFinalBoundPointSetWithSobel[i][0] > Image.rows - max_bound_dist || MergedFinalBoundPointSetWithSobel[i][1] < max_bound_dist || MergedFinalBoundPointSetWithSobel[i][1] > Image.cols - max_bound_dist)
            {
                int neighboor_final_points_num = 0;

                for(int j = 0; j < MergedFinalBoundPointSetWithSobel.size(); ++j)
                {
                    double dist = 0;
                    dist += (MergedFinalBoundPointSetWithSobel[i][0] - MergedFinalBoundPointSetWithSobel[j][0]) * (MergedFinalBoundPointSetWithSobel[i][0] - MergedFinalBoundPointSetWithSobel[j][0]);
                    dist += (MergedFinalBoundPointSetWithSobel[i][1] - MergedFinalBoundPointSetWithSobel[j][1]) * (MergedFinalBoundPointSetWithSobel[i][1] - MergedFinalBoundPointSetWithSobel[j][1]);
                    dist = sqrt(dist);

                    if(dist <= search_dist)
                    {
                        ++neighboor_final_points_num;
                    }
                }

                if(neighboor_final_points_num - 1 < min_group_num)
                {
                    MergedFinalBoundPointSetWithSobel.erase(MergedFinalBoundPointSetWithSobel.begin() + i);

                    have_changed = true;
                }
            }
        }
    }
}

void FindBound::DrawSelectedColor(int length)
{
    if(DEBUG_MODE)
    {
        std::cout << "    Start DrawSelectedColor..." << std::endl;
    }

    int bound_width = 3;

    for(int i = 0; i < ColorsSortedByArea.size(); ++i)
    {
        for(int x = 0; x < length; ++x)
        {
            for(int y = 0; y < length; ++y)
            {
                Image.at<cv::Vec3b>(y + bound_width, x + bound_width*(2*i + 1) + i*length)[0] = ColorsSortedByArea[i][0];
                Image.at<cv::Vec3b>(y + bound_width, x + bound_width*(2*i + 1) + i*length)[1] = ColorsSortedByArea[i][1];
                Image.at<cv::Vec3b>(y + bound_width, x + bound_width*(2*i + 1) + i*length)[2] = ColorsSortedByArea[i][2];
            }
        }

        for(int x = 0; x < length + 2*bound_width; ++x)
        {
            for(int j = 0; j < 3; ++j)
            {
                for(int k = 0; k < 3; ++k)
                {
                    Image.at<cv::Vec3b>(x, j + bound_width*2*i + i*length)[k] = 255;
                    Image.at<cv::Vec3b>(x, j + bound_width*(2*i + 1) + (i + 1)*length)[k] = 255;
                    Image.at<cv::Vec3b>(j, x + bound_width*2*i + i*length)[k] = 255;
                    Image.at<cv::Vec3b>(j + bound_width + length, x + bound_width*2*i + i*length)[k] = 255;
                }
            }
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "    Finish DrawSelectedColor..." << std::endl;
    }
}

void FindBound::DrawColorAreaWithFixedColors(int draw_tolerance)
{
    ColorsSortedByArea.resize(2);

    ColorsSortedByArea[0].resize(3);
    ColorsSortedByArea[1].resize(3);

    ColorsSortedByArea[0][0] = 240;
    ColorsSortedByArea[0][1] = 180;
    ColorsSortedByArea[0][2] = 180;

    ColorsSortedByArea[1][0] = 230;
    ColorsSortedByArea[1][1] = 230;
    ColorsSortedByArea[1][2] = 230;

    if(DEBUG_MODE)
    {
        for(int i = 0; i < ColorsSortedByArea.size(); ++i)
        {
            std::cout << "==== Selected Color : (" << ColorsSortedByArea[i][0] << "," << ColorsSortedByArea[i][1] << "," << ColorsSortedByArea[i][2] << ") , " << ColorsSortedByArea[i][3] << std::endl;
        }
    }

    std::vector<std::vector<int>> colors_to_draw;
    if(ColorsSortedByArea.size() < 7)
    {
        colors_to_draw.resize(7);
    }
    else
    {
        colors_to_draw.resize(ColorsSortedByArea.size());
    }
    for(int i = 0; i <colors_to_draw.size(); ++i)
    {
        colors_to_draw[i].resize(3);
    }

    colors_to_draw[0][2] = 255;
    for(int i = 1; i < colors_to_draw.size(); ++i)
    {
        colors_to_draw[i][1] = 255;
    }

    for(int i = 0; i < ColorsSortedByArea.size(); ++i)
    {
        for(int x = 0; x < Image.rows; ++x)
        {
            for(int y = 0; y < Image.cols; ++y)
            {
                if(!HaveDrawnAtImage[x][y])
                {
                    if(true)
                    {
                        double color_dist = 0;

                        color_dist += (Image.at<cv::Vec3b>(x, y)[0] - ColorsSortedByArea[i][0]) * (Image.at<cv::Vec3b>(x, y)[0] - ColorsSortedByArea[i][0]);
                        color_dist += (Image.at<cv::Vec3b>(x, y)[1] - ColorsSortedByArea[i][1]) * (Image.at<cv::Vec3b>(x, y)[1] - ColorsSortedByArea[i][1]);
                        color_dist += (Image.at<cv::Vec3b>(x, y)[2] - ColorsSortedByArea[i][2]) * (Image.at<cv::Vec3b>(x, y)[2] - ColorsSortedByArea[i][2]);
                        color_dist = sqrt(color_dist);

                        if((GetHSVDist(HSV_ColorSorted[i], HSV_Color[x][y]) < draw_tolerance && HSV_Color[x][y][2] > 128 && color_dist < 6000) || color_dist < 40)
                        {
                            HaveDrawnAtImage[x][y] = true;

                            Image.at<cv::Vec3b>(x, y)[0] = colors_to_draw[i][0];
                            Image.at<cv::Vec3b>(x, y)[1] = colors_to_draw[i][1];
                            Image.at<cv::Vec3b>(x, y)[2] = colors_to_draw[i][2];
                        }
                    }
                    else
                    {
                        double color_dist = 0;

                        color_dist += (Image.at<cv::Vec3b>(x, y)[0] - ColorsSortedByArea[i][0]) * (Image.at<cv::Vec3b>(x, y)[0] - ColorsSortedByArea[i][0]);
                        color_dist += (Image.at<cv::Vec3b>(x, y)[1] - ColorsSortedByArea[i][1]) * (Image.at<cv::Vec3b>(x, y)[1] - ColorsSortedByArea[i][1]);
                        color_dist += (Image.at<cv::Vec3b>(x, y)[2] - ColorsSortedByArea[i][2]) * (Image.at<cv::Vec3b>(x, y)[2] - ColorsSortedByArea[i][2]);

                        color_dist = sqrt(color_dist);

                        if(color_dist < 60)
                        {
                            Image.at<cv::Vec3b>(x, y)[0] = colors_to_draw[i][0];
                            Image.at<cv::Vec3b>(x, y)[1] = colors_to_draw[i][1];
                            Image.at<cv::Vec3b>(x, y)[2] = colors_to_draw[i][2];
                        }
                    }
                }
            }
        }
    }
}

void FindBound::DrawColorAreaWithMostOfColorsInImage(int draw_tolerance)
{
    if(DEBUG_MODE)
    {
        std::cout << "    Start DrawColorAreaWithMostOfColorsInImage..." << std::endl;
        for(int i = 0; i < ColorsSortedByArea.size(); ++i)
        {
            std::cout << "==== Selected Color : (" << ColorsSortedByArea[i][0] << "," << ColorsSortedByArea[i][1] << "," << ColorsSortedByArea[i][2] << ") , " << ColorsSortedByArea[i][3] << std::endl;
        }
    }
    std::vector<std::vector<int>> colors_to_draw;
    if(ColorsSortedByArea.size() < 7)
    {
        colors_to_draw.resize(7);
    }
    else
    {
        colors_to_draw.resize(ColorsSortedByArea.size());
    }
    for(int i = 0; i <colors_to_draw.size(); ++i)
    {
        colors_to_draw[i].resize(3);
    }

    colors_to_draw[0][2] = 255;
    for(int i = 1; i < colors_to_draw.size(); ++i)
    {
        colors_to_draw[i][1] = 255;
    }

    if(ColorsSortedByArea.size() > 2)
    {
        ColorsSortedByArea.resize(3);
        double dist = 0;
        int fartest_idx = 0;

        double current_dist = 0;
        current_dist += (ColorsSortedByArea[0][0] - ColorsSortedByArea[1][0])*(ColorsSortedByArea[0][0] - ColorsSortedByArea[1][0]);
        current_dist += (ColorsSortedByArea[0][1] - ColorsSortedByArea[1][1])*(ColorsSortedByArea[0][1] - ColorsSortedByArea[1][1]);
        current_dist += (ColorsSortedByArea[0][2] - ColorsSortedByArea[1][2])*(ColorsSortedByArea[0][2] - ColorsSortedByArea[1][2]);

        if(current_dist > dist)
        {
            dist = current_dist;
            fartest_idx = 2;
        }

        current_dist = 0;
        current_dist += (ColorsSortedByArea[0][0] - ColorsSortedByArea[2][0])*(ColorsSortedByArea[0][0] - ColorsSortedByArea[2][0]);
        current_dist += (ColorsSortedByArea[0][1] - ColorsSortedByArea[2][1])*(ColorsSortedByArea[0][1] - ColorsSortedByArea[2][1]);
        current_dist += (ColorsSortedByArea[0][2] - ColorsSortedByArea[2][2])*(ColorsSortedByArea[0][2] - ColorsSortedByArea[2][2]);

        if(current_dist > dist)
        {
            dist = current_dist;
            fartest_idx = 1;
        }

        current_dist = 0;
        current_dist += (ColorsSortedByArea[2][0] - ColorsSortedByArea[1][0])*(ColorsSortedByArea[2][0] - ColorsSortedByArea[1][0]);
        current_dist += (ColorsSortedByArea[2][1] - ColorsSortedByArea[1][1])*(ColorsSortedByArea[2][1] - ColorsSortedByArea[1][1]);
        current_dist += (ColorsSortedByArea[2][2] - ColorsSortedByArea[1][2])*(ColorsSortedByArea[2][2] - ColorsSortedByArea[1][2]);

        if(current_dist > dist)
        {
            dist = current_dist;
            fartest_idx = 0;
        }

        ColorsSortedByArea.erase(ColorsSortedByArea.begin() + fartest_idx);
        HSV_ColorSorted.erase(HSV_ColorSorted.begin() + fartest_idx);

        double dist_1 = 0;
        dist_1 += (ColorsSortedByArea[0][0] - 200) * (ColorsSortedByArea[0][0] - 200);
        dist_1 += (ColorsSortedByArea[0][1] - 200) * (ColorsSortedByArea[0][1] - 200);
        dist_1 += (ColorsSortedByArea[0][2] - 200) * (ColorsSortedByArea[0][2] - 200);

        double dist_2 = 0;
        dist_2 += (ColorsSortedByArea[1][0] - 200) * (ColorsSortedByArea[1][0] - 200);
        dist_2 += (ColorsSortedByArea[1][1] - 200) * (ColorsSortedByArea[1][1] - 200);
        dist_2 += (ColorsSortedByArea[1][2] - 200) * (ColorsSortedByArea[1][2] - 200);


        if(dist_1 < dist_2)
        {
            int change_color = ColorsSortedByArea[0][0];
            ColorsSortedByArea[0][0] = ColorsSortedByArea[1][0];
            ColorsSortedByArea[1][0] = change_color;
            change_color = ColorsSortedByArea[0][1];
            ColorsSortedByArea[0][1] = ColorsSortedByArea[1][1];
            ColorsSortedByArea[1][1] = change_color;
            change_color = ColorsSortedByArea[0][2];
            ColorsSortedByArea[0][2] = ColorsSortedByArea[1][2];
            ColorsSortedByArea[1][2] = change_color;

            change_color = HSV_ColorSorted[0][0];
            HSV_ColorSorted[0][0] = HSV_ColorSorted[1][0];
            HSV_ColorSorted[1][0] = change_color;
            change_color = HSV_ColorSorted[0][1];
            HSV_ColorSorted[0][1] = HSV_ColorSorted[1][1];
            HSV_ColorSorted[1][1] = change_color;
            change_color = HSV_ColorSorted[0][2];
            HSV_ColorSorted[0][2] = HSV_ColorSorted[1][2];
            HSV_ColorSorted[1][2] = change_color;
        }

        if(DEBUG_MODE)
        {
            for(int i = 0; i < ColorsSortedByArea.size(); ++i)
            {
                std::cout << "==== Selected Color Again : (" << ColorsSortedByArea[i][0] << "," << ColorsSortedByArea[i][1] << "," << ColorsSortedByArea[i][2] << ") , " << ColorsSortedByArea[i][3] << std::endl;
            }
        }
    }

    for(int x = 0; x < Image.rows; ++x)
    {
        for(int y = 0; y < Image.cols; ++y)
        {
            double dist = 10000000;
            int color_idx = 0;

            for(int i = 0; i < ColorsSortedByArea.size(); ++i)
            {
                double color_dist = 0;

                color_dist += (Image.at<cv::Vec3b>(x, y)[0] - ColorsSortedByArea[i][0]) * (Image.at<cv::Vec3b>(x, y)[0] - ColorsSortedByArea[i][0]);
                color_dist += (Image.at<cv::Vec3b>(x, y)[1] - ColorsSortedByArea[i][1]) * (Image.at<cv::Vec3b>(x, y)[1] - ColorsSortedByArea[i][1]);
                color_dist += (Image.at<cv::Vec3b>(x, y)[2] - ColorsSortedByArea[i][2]) * (Image.at<cv::Vec3b>(x, y)[2] - ColorsSortedByArea[i][2]);

                if(color_dist < dist)
                {
                    dist = color_dist;
                    color_idx = i;
                }
            }

            if(sqrt(dist) < 100)
            {
                Image.at<cv::Vec3b>(x, y)[0] = colors_to_draw[color_idx][0];
                Image.at<cv::Vec3b>(x, y)[1] = colors_to_draw[color_idx][1];
                Image.at<cv::Vec3b>(x, y)[2] = colors_to_draw[color_idx][2];
            }
        }
    }

    for(int x = 0; x < Image.rows; ++x)
    {
        for(int y = 0; y < Image.cols; ++y)
        {
            cv::Vec3b color = Image.at<cv::Vec3b>(x, y);
            if((color[0] + color[1] + color[2] > 0) && (color[0] != 0 || color[1] != 0 || color[2] != 255) && (color[0] != 0 || color[1] != 255 || color[2] != 0))
            {
                Image.at<cv::Vec3b>(x, y)[0] = 255;
                Image.at<cv::Vec3b>(x, y)[1] = 255;
                Image.at<cv::Vec3b>(x, y)[2] = 255;
            }
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "    Finish DrawColorAreaWithMostOfColorsInImage..." << std::endl;
    }
}

void FindBound::RemoveHoles(int search_dist)
{
    if(DEBUG_MODE)
    {
        std::cout << "    Start RemoveHoles..." << std::endl;
    }

    bool have_changed_color = true;

    std::vector<bool> have_different_color;
    have_different_color.resize(directions.size());

    while(have_changed_color)
    {
        have_changed_color = false;

        for(int i = 0; i < Image.rows; ++i)
        {
            if(have_changed_color)
            {
                break;
            }

            for(int j = 0; j < Image.cols; ++j)
            {
                cv::Vec3b color = Image.at<cv::Vec3b>(i, j);

                if(color[0] == 0 && color[1] + color[2] == 255 && (color[1] == 0 || color[2] == 0))
                {
                    for(int k = 0; k < have_different_color.size(); ++k)
                    {
                        have_different_color[k] = false;
                    }

                    for(int direction = 0; direction < directions.size(); ++direction)
                    {
                        if(have_different_color[direction])
                        {
                            break;
                        }

                        for(int k = 1; k < search_dist; ++k)
                        {
                            if(0 <= i + int(k*directions[direction][0]) && i + int(k*directions[direction][0]) < Image.rows && 0 <= j + int(k*directions[direction][1]) && j + int(k*directions[direction][1]) < Image.cols)
                            {
                                cv::Vec3b current_color = Image.at<cv::Vec3b>(i + int(k*directions[direction][0]), j + int(k*directions[direction][1]));

                                if(current_color[0] != color[0] || current_color[1] != color[1] || current_color[2] != color[2])
                                {
                                    have_different_color[direction] = true;

                                    break;
                                }
                            }
                            else
                            {
                                have_different_color[direction] = true;

                                break;
                            }
                        }
                    }

                    bool need_to_remove = true;

                    for(int k = 0; k < have_different_color.size(); ++k)
                    {
                        if(!have_different_color[k])
                        {
                            need_to_remove = false;
                        }
                    }

                    if(need_to_remove)
                    {
                        Image.at<cv::Vec3b>(i, j)[0] = 255;
                        Image.at<cv::Vec3b>(i, j)[1] = 255;
                        Image.at<cv::Vec3b>(i, j)[2] = 255;

                        //have_changed_color = true;

                        //break;
                    }
                }
            }
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "    Finish RemoveHoles..." << std::endl;
    }
}

void FindBound::RemoveBlackPoint(int search_dist, double black_percent)
{
    if(DEBUG_MODE)
    {
        std::cout << "    Start RemoveBlackPoint..." << std::endl;
    }

    for(int i = 0; i < Image.rows; ++i)
    {
        for(int j = 0; j < Image.cols; ++j)
        {
            if(Image.at<cv::Vec3b>(i, j)[0] + Image.at<cv::Vec3b>(i, j)[1] + Image.at<cv::Vec3b>(i, j)[2] == 0)
            {
                std::vector<std::vector<int>> black_points;
                std::vector<std::vector<int>> total_points;

                for(int k = 0; k < search_dist; ++k)
                {
                    for(int direction = 0; direction < directions.size(); ++direction)
                    {
                        if(0 <= i + int(k*directions[direction][0]) && i + int(k*directions[direction][0]) < Image.rows && 0 <= j + int(k*directions[direction][1]) && j + int(k*directions[direction][1]) < Image.cols)
                        {
                            cv::Vec3b current_color = Image.at<cv::Vec3b>(i + int(k*directions[direction][0]), j + int(k*directions[direction][1]));

                            if(current_color[0] + current_color[1] + current_color[2] == 0)
                            {
                                bool have_selected = false;

                                for(int l = 0; l < black_points.size(); ++l)
                                {
                                    if(i + int(k*directions[direction][0]) == black_points[l][0] && j + int(k*directions[direction][1]) == black_points[l][1])
                                    {
                                        have_selected = true;

                                        break;
                                    }
                                }

                                if(!have_selected)
                                {
                                    std::vector<int> black_point;
                                    black_point.emplace_back(i + int(k*directions[direction][0]));
                                    black_point.emplace_back(j + int(k*directions[direction][1]));
                                    black_points.emplace_back(black_point);
                                }
                            }

                            bool have_selected = false;

                            for(int l = 0; l < total_points.size(); ++l)
                            {
                                if(i + int(k*directions[direction][0]) == total_points[l][0] && j + int(k*directions[direction][1]) == total_points[l][1])
                                {
                                    have_selected = true;

                                    break;
                                }
                            }

                            if(!have_selected)
                            {
                                std::vector<int> total_point;
                                total_point.emplace_back(i + int(k*directions[direction][0]));
                                total_point.emplace_back(j + int(k*directions[direction][1]));
                                total_points.emplace_back(total_point);
                            }
                        }
                    }
                }

                if(double(black_points.size()) / double(total_points.size()) < black_percent)
                {
                    Image.at<cv::Vec3b>(i, j)[0] = 255;
                    Image.at<cv::Vec3b>(i, j)[1] = 255;
                    Image.at<cv::Vec3b>(i, j)[2] = 255;
                }
            }
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "    Finish RemoveBlackPoint..." << std::endl;
    }
}

void FindBound::MergeHoles(int max_merge_dist)
{
    if(DEBUG_MODE)
    {
        std::cout << "    Start MergeHoles..." << std::endl;
    }

    for(int i = 0; i < Image.rows; ++i)
    {
        for(int j = 0; j < Image.cols; ++j)
        {
            cv::Vec3b color = Image.at<cv::Vec3b>(i, j);

            if(color[0] == 255 && color[1] + color[2] == 0)
            {
                bool have_merged = false;

                for(int k = 0; k < max_merge_dist; ++k)
                {
                    if(have_merged)
                    {
                        break;
                    }

                    for(int direction = 0; direction < directions.size(); ++direction)
                    {
                        if(0 <= i + int(k*directions[direction][0]) && i + int(k*directions[direction][0]) < Image.rows && 0 <= j + int(k*directions[direction][1]) && j + int(k*directions[direction][1]) < Image.cols)
                        {
                            cv::Vec3b search_color = Image.at<cv::Vec3b>(i + int(k*directions[direction][0]), j + int(k*directions[direction][1]));

                            if(search_color[0] == 0 && search_color[1] + search_color[2] == 255 && (search_color[1] == 0 || search_color[2] == 0))
                            {
                                Image.at<cv::Vec3b>(i, j)[0] = search_color[0];
                                Image.at<cv::Vec3b>(i, j)[1] = search_color[1];
                                Image.at<cv::Vec3b>(i, j)[2] = search_color[2];

                                have_merged = true;

                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "    Finish MergeHoles..." << std::endl;
    }
}

void FindBound::MergeFinalPoints(int merge_dist, double min_point_dist)
{
    MergedFinalBoundPointSetWithSobel.clear();

    for(int i = 0; i < Image.rows; i+=merge_dist)
    {
        for(int j = 0; j < Image.cols; j+=merge_dist)
        {
            double merge_point_x = 0;
            double merge_point_y = 0;
            int merge_point_num = 0;

            for(int k = 0; k < FinalBoundPointSetWithSobel.size(); ++k)
            {
                if(0 <= FinalBoundPointSetWithSobel[k][0] - i && FinalBoundPointSetWithSobel[k][0] - i < merge_dist && 0 <= FinalBoundPointSetWithSobel[k][1] - j && FinalBoundPointSetWithSobel[k][1] - j < merge_dist)
                {
                    merge_point_x += FinalBoundPointSetWithSobel[k][0];
                    merge_point_y += FinalBoundPointSetWithSobel[k][1];
                    ++merge_point_num;
                }
            }

            if(merge_point_num > 0)
            {
                merge_point_x /= merge_point_num;
                merge_point_y /= merge_point_num;

                std::vector<int> merged_point;
                merged_point.emplace_back(int(merge_point_x));
                merged_point.emplace_back(int(merge_point_y));

                MergedFinalBoundPointSetWithSobel.emplace_back(merged_point);
            }
        }
    }

    bool merge_changed = true;

    while(merge_changed)
    {
        merge_changed = false;

        double min_dist = 100000000;
        int min_dist_idx_1 = 0;
        int min_dist_idx_2 = 0;

        for(int i = MergedFinalBoundPointSetWithSobel.size() - 1; i >= 0; --i)
        {
            for(int j = 0; j < i; ++j)
            {
                double dist = 0;

                dist += (MergedFinalBoundPointSetWithSobel[i][0] - MergedFinalBoundPointSetWithSobel[j][0]) * (MergedFinalBoundPointSetWithSobel[i][0] - MergedFinalBoundPointSetWithSobel[j][0]);
                dist += (MergedFinalBoundPointSetWithSobel[i][1] - MergedFinalBoundPointSetWithSobel[j][1]) * (MergedFinalBoundPointSetWithSobel[i][1] - MergedFinalBoundPointSetWithSobel[j][1]);

                if(dist < min_dist)
                {
                    min_dist = dist;
                    min_dist_idx_1 = j;
                    min_dist_idx_2 = i;
                }
            }
        }

        if(sqrt(min_dist) <= min_point_dist)
        {
            MergedFinalBoundPointSetWithSobel[min_dist_idx_1][0] = int((MergedFinalBoundPointSetWithSobel[min_dist_idx_1][0] + MergedFinalBoundPointSetWithSobel[min_dist_idx_2][0]) / 2.0);
            MergedFinalBoundPointSetWithSobel[min_dist_idx_1][1] = int((MergedFinalBoundPointSetWithSobel[min_dist_idx_1][1] + MergedFinalBoundPointSetWithSobel[min_dist_idx_2][1]) / 2.0);
            MergedFinalBoundPointSetWithSobel.erase(MergedFinalBoundPointSetWithSobel.begin() + min_dist_idx_2);

            merge_changed = true;
        }
    }
}

double FindBound::Dot(std::vector<int> point_1, std::vector<int> point_2)
{
    return double(point_1[0]*point_2[0] + point_1[1]*point_2[1]);
}

double FindBound::TriangleArea(std::vector<int> point_1, std::vector<int> point_2, std::vector<int> point_3)
{
    return fabs(point_1[0]*point_2[1] + point_2[0]*point_3[1] + point_3[0]*point_1[1] - point_1[0]*point_3[1] - point_3[0]*point_2[1] - point_2[0]*point_1[1]) / 2.0;
}

bool FindBound::IsInTriangle(std::vector<int> point_1, std::vector<int> point_2, std::vector<int> point_3, std::vector<int> point_judge, double in_scale)
{
    std::vector<int> v0, v1, v2;

    v0.emplace_back(point_2[0] - point_1[0]);
    v0.emplace_back(point_2[1] - point_1[1]);

    v1.emplace_back(point_3[0] - point_1[0]);
    v1.emplace_back(point_3[1] - point_1[1]);

    v2.emplace_back(point_judge[0] - point_1[0]);
    v2.emplace_back(point_judge[1] - point_1[1]);

    double dot00 = Dot(v0, v0);
    double dot01 = Dot(v0, v1);
    double dot02 = Dot(v0, v2);
    double dot11 = Dot(v1, v1);
    double dot12 = Dot(v1, v2);

    double inverDeno = 1 / (dot00 * dot11 - dot01 * dot01);

    double u = (dot11 * dot02 - dot01 * dot12) * inverDeno;
    double v = (dot00 * dot12 - dot01 * dot02) * inverDeno;

    if((1.0 - in_scale) / 2.0 <= u && (1.0 - in_scale) / 2.0 <= v && u + v <= (1.0 + in_scale) / 2.0)
    {
        return true;
    }

    return false;
}

void FindBound::RemoveInsidePoints(double remove_scale, double min_triangle_area)
{
    if(DEBUG_MODE)
    {
        std::cout << "    Start RemoveInsidePoints..." << std::endl;
    }

    bool remove_changed = true;

    while(remove_changed)
    {
        remove_changed = false;

        for(int i = MergedFinalBoundPointSetWithSobel.size() - 1; i >= 0; --i)
        {
            if(remove_changed)
            {
                break;
            }

            for(int j = i - 1; j >= 0; --j)
            {
                if(remove_changed)
                {
                    break;
                }

                for(int k = j - 1; k >= 0; --k)
                {
                    if(remove_changed)
                    {
                        break;
                    }

                    if(TriangleArea(MergedFinalBoundPointSetWithSobel[i], MergedFinalBoundPointSetWithSobel[j], MergedFinalBoundPointSetWithSobel[k]) > min_triangle_area)
                    {
                        for(int l = MergedFinalBoundPointSetWithSobel.size() - 1; l >= 0; --l)
                        {
                            if(l != i && l != j && l != k)
                            {
                                if(IsInTriangle(MergedFinalBoundPointSetWithSobel[i], MergedFinalBoundPointSetWithSobel[j], MergedFinalBoundPointSetWithSobel[k], MergedFinalBoundPointSetWithSobel[l], remove_scale))
                                {
                                    MergedFinalBoundPointSetWithSobel.erase(MergedFinalBoundPointSetWithSobel.begin() + l);

                                    remove_changed = true;

                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if(DEBUG_MODE)
    {
        std::cout << "    Finish RemoveInsidePoints..." << std::endl;
    }
}

void FindBound::DrawBoundPoints()
{
    for(int i = 0; i < SameColorBoundPointSet.size(); ++i)
    {
        for(int j = 1; j < SameColorBoundPointSet[i].size(); ++j)
        {
            for(int k = 0; k < 3; ++k)
            {
                Image_to_draw.at<cv::Vec3b>(SameColorBoundPointSet[i][j][0], SameColorBoundPointSet[i][j][1])[k] = SameColorBoundPointSet[i][0][k];
            }
        }
    }
}

void FindBound::DrawReliableBoundPoints()
{
    for(int i = 0; i < ReliableBoundPointSet.size(); ++i)
    {
        for(int j = 1; j < ReliableBoundPointSet[i].size(); ++j)
        {
            for(int k = 0; k < 3; ++k)
            {
                Image_to_draw.at<cv::Vec3b>(ReliableBoundPointSet[i][j][0], ReliableBoundPointSet[i][j][1])[k] = ReliableBoundPointSet[i][0][k];
            }
        }
    }
}

void FindBound::DrawFinalBoundPoints(int point_size)
{
    for(int i = 0; i < FinalBoundPointSet.size(); ++i)
    {
        for(int j = 0; j < point_size; ++j)
        {
            for(int k = 0; k < point_size; ++k)
            {
                if(FinalBoundPointSet[i][0] + j < Image_to_draw.rows && FinalBoundPointSet[i][1] + k < Image_to_draw.cols)
                {
                    Image_to_draw.at<cv::Vec3b>(FinalBoundPointSet[i][0] + j, FinalBoundPointSet[i][1] + k)[0] = 0;
                    Image_to_draw.at<cv::Vec3b>(FinalBoundPointSet[i][0] + j, FinalBoundPointSet[i][1] + k)[1] = 0;
                    Image_to_draw.at<cv::Vec3b>(FinalBoundPointSet[i][0] + j, FinalBoundPointSet[i][1] + k)[2] = 255;
                }

                if(FinalBoundPointSet[i][0] + j < Image_to_draw.rows && FinalBoundPointSet[i][1] - k >= 0)
                {
                    Image_to_draw.at<cv::Vec3b>(FinalBoundPointSet[i][0] + j, FinalBoundPointSet[i][1] - k)[0] = 0;
                    Image_to_draw.at<cv::Vec3b>(FinalBoundPointSet[i][0] + j, FinalBoundPointSet[i][1] - k)[1] = 0;
                    Image_to_draw.at<cv::Vec3b>(FinalBoundPointSet[i][0] + j, FinalBoundPointSet[i][1] - k)[2] = 255;
                }

                if(FinalBoundPointSet[i][0] - j >= 0 && FinalBoundPointSet[i][1] + k < Image_to_draw.cols)
                {
                    Image_to_draw.at<cv::Vec3b>(FinalBoundPointSet[i][0] - j, FinalBoundPointSet[i][1] + k)[0] = 0;
                    Image_to_draw.at<cv::Vec3b>(FinalBoundPointSet[i][0] - j, FinalBoundPointSet[i][1] + k)[1] = 0;
                    Image_to_draw.at<cv::Vec3b>(FinalBoundPointSet[i][0] - j, FinalBoundPointSet[i][1] + k)[2] = 255;
                }

                if(FinalBoundPointSet[i][0] - j >= 0 && FinalBoundPointSet[i][1] - k >= 0)
                {
                    Image_to_draw.at<cv::Vec3b>(FinalBoundPointSet[i][0] - j, FinalBoundPointSet[i][1] - k)[0] = 0;
                    Image_to_draw.at<cv::Vec3b>(FinalBoundPointSet[i][0] - j, FinalBoundPointSet[i][1] - k)[1] = 0;
                    Image_to_draw.at<cv::Vec3b>(FinalBoundPointSet[i][0] - j, FinalBoundPointSet[i][1] - k)[2] = 255;
                }
            }
        }
    }
}

void FindBound::on_Canny(int, void *)
{
    //先使用3×3的内核进行降噪
    cv::blur(g_srcGrayImage, g_cannyDetectedEdges, cv::Size(3, 3));

    cv::Canny(g_cannyDetectedEdges, g_cannyDetectedEdges, g_cannyLowThreshold, g_cannyLowThreshold * 3, 3);
    //设置为0
    g_dstImage = cv::Scalar::all(0);
    //使用Canny算子输出的边缘图g_cannyDetectedEdge作为掩码，将原图g_srcImage拷贝到g_dstImage中
    g_srcImage.copyTo(g_dstImage, g_cannyDetectedEdges);

    cv::imshow("【效果图】Canny边缘检测", g_dstImage);
}

void FindBound::on_Sobel(int, void *)
{
    //求X方向的梯度
    cv::Sobel(g_srcImage, g_sobelGradient_X, CV_16S, 1, 0, (2 * g_sobelKernelSize + 1), 1, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(g_sobelGradient_X, g_sobelAbsGradient_X);

    //求Y方向的梯度
    cv::Sobel(g_srcImage, g_sobelGradient_Y, CV_16S, 0, 1, (2 * g_sobelKernelSize + 1), 1, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(g_sobelGradient_Y, g_sobelAbsGradient_Y);
    //合并梯度
    cv::addWeighted(g_sobelAbsGradient_X, 0.5, g_sobelAbsGradient_Y, 0.5, 0, g_dstImage);

    cv::cvtColor(g_dstImage, g_dstGrayImage, cv::COLOR_BGR2GRAY);

    for(int x = 0; x < g_dstGrayImage.rows; ++x)
    {
        for(int y = 0; y < g_dstGrayImage.cols; ++y)
        {
            int     b=g_dstImage.at<cv::Vec3b>(x, y)[0],
                    g=g_dstImage.at<cv::Vec3b>(x, y)[1],
                    r=g_dstImage.at<cv::Vec3b>(x, y)[2];
            std::vector<int> hsv_value=rgb2hsv(r,g,b);
            int v=(int)g_dstGrayImage.at<uchar>(x, y);
            if(v<20 || v>150 || abs(hsv_value[0]-39)>15)
            {
                g_dstGrayImage.at<uchar>(x, y) = (uchar)0;
                g_dstImage.at<cv::Vec3b>(x, y)=cv::Vec3b(0,0,0);
            }
            else
            {
                boundary_points_.insert(10000*x+y,1);
                Image.at<cv::Vec3b>(x, y)[0] = 0;
                Image.at<cv::Vec3b>(x, y)[1] = 0;
                Image.at<cv::Vec3b>(x, y)[2] = 0;
            }
        }
    }
//    cv::Mat tmp_img;
//    cv::Mat kernel= (cv::Mat_<double>(3, 3) << -1, -1, -1, -1, 8, -1, -1, -1, -1);

//    cv::filter2D(g_dstImage,g_dstImage,g_dstImage.depth(),kernel);
}

void FindBound::Scharr()
{
    //求x方向的梯度
    cv::Scharr(g_srcImage, g_scharrGradient_x, CV_16S, 1, 0, 1, 0, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(g_scharrGradient_x, g_scharrAbsGradient_x);

    //求y方向的梯度
    cv::Scharr(g_srcImage, g_scharrGradient_Y, CV_16S, 0, 1, 1, 0, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(g_scharrGradient_Y, g_scharrAbsGradient_Y);

    cv::addWeighted(g_scharrAbsGradient_x, 0.5, g_scharrAbsGradient_Y, 0.5, 0, g_dstImage);

    cv::imshow("【效果图】Scharr滤波器", g_dstImage);
}

void FindBound::GetBoundWithCV()
{
    if(DEBUG_MODE)
    {
        std::cout << "    Start GetBoundWithCV..." << std::endl;
    }

    //改变颜色
    system("color 2F");

    Image.copyTo(g_srcImage);

    //创建与src同类型和大小的矩阵dst
    g_dstImage.create(g_srcImage.size(), g_srcImage.type());

    cv::cvtColor(g_srcImage, g_srcGrayImage, cv::COLOR_BGR2GRAY);

    //on_Canny(0, 0);
    //Scharr();

    on_Sobel(0, 0);

    if(DEBUG_MODE)
    {
        std::cout << "    Finish GetBoundWithCV..." << std::endl;
    }
}
