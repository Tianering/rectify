//
// Created by shanyi on 2021/7/6.
//
#include "rectify.h"
#include <omp.h>

// 获取相机参数
Matrix<double, 3, 3> ParametersCamera(Matrix<double, 3, 3> &left_intrinsic, Matrix<double, 3, 3> &right_intrinsic,
                                      Matrix<double, 3, 3> &intrinsic, Matrix<double, 3, 3> &leftToright,
                                      Matrix<double, 3, 1> &translation_vector, Matrix<double, 1, 6> &left_radial_dis,
                                      Matrix<double, 1, 6> &right_radial_dis, Matrix<double, 1, 2> &left_tangential_dis,
                                      Matrix<double, 1, 2> &right_tangential_dis) {
    // 左右相机内参
    left_intrinsic
            << 1600.3, 0, 979.588,
            0, 1600.65, 677.279,
            0, 0, 1;
    right_intrinsic
            << 1497.11, 0, 959.216,
            0, 1496.61, 538.616,
            0, 0, 1;
    intrinsic << (left_intrinsic / 2) + (right_intrinsic / 2);
    //左相机到右相机旋转矩阵及平移向量
    leftToright
            << 0.999984134192982, -0.000269269734259, -0.005626620310850,
            0.000263998309506, 0.999999525611060, -0.000937594020693,
            0.005626870107336, 0.000936093726757, 0.999983730898323;
    translation_vector
            << -0.0230209, 0.00000972608, -0.000132853;
    //左右相机的畸变系数
    left_radial_dis << -13.3381, 241.874, 28.8881, -13.332, 242.115, 39.0898;
    right_radial_dis << -4.2663, 2.41158, 6.72838, -4.19631, 2.05607, 7.21546;
    left_tangential_dis << -0.0010641, -0.000137293;
    right_tangential_dis << -0.00249452, 0.000945203;

    Matrix<double, 3, 3> R_new;
    Matrix<double, 3, 1> x_new, y_old, y_new, z_old, z_new;

    x_new << leftToright.inverse() * translation_vector;
    x_new = -x_new;
    y_old << 0, 1, 0;
    z_old = x_new.cross(y_old);
    y_new = z_old.cross(x_new);
    z_new = x_new.cross(y_new);
    R_new << x_new.transpose() / x_new.norm(),
            y_new.transpose() / y_new.norm(),
            z_new.transpose() / z_new.norm();
    return R_new;
}


// 去畸变类
Matrix<double, 3, 1>
undistortion(Matrix<double, 3, 1> point, Matrix<double, 1, 6> radial_dis, Matrix<double, 1, 2> tangential_dis) {
    // 处理畸变
    // x_dn = x_un * (1 + k1 * r^2 + k2 * r^4 + k3 * r^6)/(1 + k4 * r^2 + k5 * r^4 + k6 * r^6) + 2 * p1 * x_un * y_un + p2 * (r^2 + 2 * x_un^2),
    // y_dn = y_un * (1 + k1 * r^2 + k2 * r^4 + k3 * r^6)/(1 + k4 * r^2 + k5 * r^4 + k6 * r^6) + p1 * (r^2 + 2 * y_un^2) + 2 * p2 * x_un * y_un,
    double r = sqrt(pow(point(0), 2) + pow(point(1), 2));
    double formula = (1 + (radial_dis(0) * pow(r, 2)) + (radial_dis(1) * pow(r, 4)) + (radial_dis(2) * pow(r, 6))) /
                     (1 + (radial_dis(3) * pow(r, 2)) + (radial_dis(4) * pow(r, 4)) + (radial_dis(5) * pow(r, 6)));
    double x_dn =
            point(0) * formula + 2 * tangential_dis(0) * point(0) * point(1) +
            tangential_dis(1) * (pow(r, 2) + 2 * pow(point(0), 2));
    double y_dn =
            point(1) * formula + tangential_dis(0) * (pow(r, 2) + 2 * pow(point(1), 2)) +
            2 * tangential_dis(1) * point(0) * point(1);
    point(0) = x_dn;
    point(1) = y_dn;
    point(2) = 1;
    return point;
    // 倾斜平面矩阵*point点
    // 经过判断对x，y点进行放缩
    // xd*fx+cx
    // 得到倾斜平面的投影矩阵
}

// 双线性插值法
vector<int> BilinearInterpolation(Matrix<double, 3, 1> point, cv::Mat image) {
    //cout << point << endl;
    vector<int> bgr1(3);
    bgr1[0] = 0;
    bgr1[1] = 0;
    bgr1[2] = 0;
    double y = point(0);
    double x = point(1);
    if (y > 1920 || y < 0 || x > 1080 || x < 0)
        return bgr1;
    int x1 = floor(x);
    int x2 = x1 + 1;
    int y1 = floor(y);
    int y2 = y1 + 1;
    //cout << x2 <<"\t"<<y2<<"\t"<<endl;
    //cv::Mat image = imread(image_path);
    // 第一次线性插值（底部）
    int rx1y1 = image.at<cv::Vec3b>(x1, y1)[2];
    int rx2y1 = image.at<cv::Vec3b>(x2, y1)[2];
    int gx1y1 = image.at<cv::Vec3b>(x1, y1)[1];
    int gx2y1 = image.at<cv::Vec3b>(x2, y1)[1];
    int bx1y1 = image.at<cv::Vec3b>(x1, y1)[0];
    int bx2y1 = image.at<cv::Vec3b>(x2, y1)[0];
    double R1 = ((x - x1) / (x2 - x1)) * rx1y1 + ((x2 - x) / (x2 - x1)) * rx2y1;
    double G1 = ((x - x1) / (x2 - x1)) * gx1y1 + ((x2 - x) / (x2 - x1)) * gx2y1;
    double B1 = ((x - x1) / (x2 - x1)) * bx1y1 + ((x2 - x) / (x2 - x1)) * bx2y1;
    // 第二次线性插值
    Vec3b point_x1y2 = image.at<cv::Vec3b>(x1, y2);
    int rx1y2 = point_x1y2[2];
    int gx1y2 = point_x1y2[1];
    int bx1y2 = point_x1y2[0];
    Vec3b point_x2y2 = image.at<cv::Vec3b>(x2, y2);
    int rx2y2 = point_x2y2[2];
    int gx2y2 = point_x2y2[1];
    int bx2y2 = point_x2y2[0];

    double R2 = ((x - x1) / (x2 - x1)) * rx1y2 + ((x2 - x) / (x2 - x1)) * rx2y2;
    double G2 = ((x - x1) / (x2 - x1)) * gx1y2 + ((x2 - x) / (x2 - x1)) * gx2y2;
    double B2 = ((x - x1) / (x2 - x1)) * bx1y2 + ((x2 - x) / (x2 - x1)) * bx2y2;
    // 第三次线性插值
    double R = ((y - y1) / (y2 - y1)) * R1 + ((y2 - y) / (y2 - y1)) * R2;
    double G = ((y - y1) / (y2 - y1)) * G1 + ((y2 - y) / (y2 - y1)) * G2;
    double B = ((y - y1) / (y2 - y1)) * B1 + ((y2 - y) / (y2 - y1)) * B2;

    bgr1[0] = B;
    bgr1[1] = G;
    bgr1[2] = R;
    return bgr1;
}

// 多通道图像赋值
void ColorChart(Mat &image, int x, int y, vector<int> point) {
    if (y % 50 == 0) {
        point[0] = 255;
        point[1] = 255;
        point[2] = 255;
    }
    image.at<cv::Vec3b>(y, x)[0] = point[0];
    image.at<cv::Vec3b>(y, x)[1] = point[1];
    image.at<cv::Vec3b>(y, x)[2] = point[2];
}

void
mapRectify(Mat &image, Matrix<double, 3, 3> intrinsic, Matrix<double, 3, 3> oneself_intrinsic,
           Matrix<double, 1, 6> radial_dis, Matrix<double, 1, 2> tangential_dis, Matrix<double, 3, 3> leftToright,
           Matrix<double, 3, 3> R_new, int lorr, Mat &mapx, Mat &mapy) {
    if (lorr == 0)
        R_new = oneself_intrinsic * leftToright * R_new.inverse();
    else
        R_new = oneself_intrinsic * R_new.inverse();
#pragma omp parallel for
    for (int y = 0; y < image.rows; y++) {
        double *mpx = mapx.ptr<double>(y);
        double *mpy = mapy.ptr<double>(y);
        for (int x = 0; x < image.cols; x++) {
            Matrix<double, 3, 1> img_pixel, img_image, img_rotating, img_undistortion;
            img_pixel << x, y, 1;
            img_image = intrinsic.inverse() * img_pixel;
            img_undistortion = undistortion(img_image, radial_dis, tangential_dis);
            img_rotating = R_new * img_undistortion;
            img_rotating(0) = img_rotating(0) / img_rotating(2);
            img_rotating(1) = img_rotating(1) / img_rotating(2);
            mpx[x] = img_rotating(0);
            mpy[x] = img_rotating(1);
        }
    }
}

void remapRectify(Mat &image, Mat &dst, Mat &res, Mat mapx, Mat mapy, int lorr) {
#pragma omp parallel for
    for (int y = 0; y < image.rows; y++) {
        double *mpx = mapx.ptr<double>(y);
        double *mpy = mapy.ptr<double>(y);
        for (int x = 0; x < image.cols; x++) {
            Matrix<double, 3, 1> point;
            point << mpx[x], mpy[y], 1;
            vector<int> bgr = BilinearInterpolation(point, image);
            // 三通道图赋值
            ColorChart(dst, x, y, bgr);
            if (lorr == 0)
                ColorChart(res, x, y, bgr);
            else if (lorr == -1)
                ColorChart(res, image.cols + x, y, bgr);
        }
    }
}

// 极线校正
Mat myPolarRectify(Mat &image, Mat &res, Matrix<double, 3, 3> oneself_intrinsic, Matrix<double, 3, 3> intrinsic,
                   Matrix<double, 1, 6> radial_dis, Matrix<double, 1, 2> tangential_dis,
                   Matrix<double, 3, 3> leftToright, Matrix<double, 3, 3> R_new, int lorr) {
    cv::Mat dst = Mat(image.rows, image.cols, CV_8UC3, Scalar(0));
    if (lorr == 0)
        R_new = oneself_intrinsic * leftToright * R_new.inverse();
    else
        R_new = oneself_intrinsic * R_new.inverse();
    cout << "left11" << endl;
    for (int x = 0; x < image.cols; x++) {
#pragma omp parallel for
        for (int y = 0; y < image.rows; y++) {
            Matrix<double, 3, 1> img_pixel, img_image, img_rotating, img_undistortion;
            img_pixel << x, y, 1;
            //cout << x << "," << y  << endl;
            // 像素坐标系转向相机坐标系
            img_image = intrinsic.inverse() * img_pixel;
            // 去畸变（图加畸变）
            img_undistortion = undistortion(img_image, radial_dis, tangential_dis);
            // 旋转坐标系
            img_rotating = R_new * img_undistortion;
            //坐标归一化
            img_rotating(0) = img_rotating(0) / img_rotating(2);
            img_rotating(1) = img_rotating(1) / img_rotating(2);
            // 通过插值法获取像素值
            vector<int> bgr = BilinearInterpolation(img_rotating, image);
            // 三通道图赋值
            ColorChart(dst, x, y, bgr);
            if (lorr == 0)
                ColorChart(res, x, y, bgr);
            else if (lorr == -1)
                ColorChart(res, image.cols + x, y, bgr);
        }
    }
    return image;
}
