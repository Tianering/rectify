//
// Created by shanyi on 2021/7/6.
//

#include <iostream>
// Eigen 核心部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/core/affine.hpp"
#include <vector>
#include <time.h>

using namespace std;
using namespace Eigen;
using namespace cv;

// 获取相机参数
Matrix<double, 3, 3> ParametersCamera(Matrix<double, 3, 3> &left_intrinsic, Matrix<double, 3, 3> &right_intrinsic,
                      Matrix<double, 3, 3> &intrinsic, Matrix<double, 3, 3> &leftToright,
                      Matrix<double, 3, 1> &translation_vector, Matrix<double, 1, 6> &left_radial_dis,
                      Matrix<double, 1, 6> &right_radial_dis, Matrix<double, 1, 2> &left_tangential_dis,
                      Matrix<double, 1, 2> &right_tangential_dis);

// 去畸变类
Matrix<double, 3, 1>
undistortion(Matrix<double, 3, 1> point, Matrix<double, 1, 6> radial_dis, Matrix<double, 1, 2> tangential_dis);

// 双线性插值法
vector<int> BilinearInterpolation(Matrix<double, 3, 1> point, cv::Mat Rimage);

//
Matrix<double, 3, 1>
ImgRotating(Matrix<double, 3, 1> point_pre, Matrix<double, 3, 3> intrinsic, Matrix<double, 3, 3> matrix_rot,
            Matrix<double, 3, 3> R_new, int lorr);

// 多通道图像赋值
void ColorChart(Mat image, int x, int y, vector<int> point);

// 极线校正
Mat myPolarRectify(Mat &image, Mat &res, Matrix<double, 3, 3> oneself_intrinsic, Matrix<double, 3, 3> intrinsic,
                   Matrix<double, 1, 6> radial_dis, Matrix<double, 1, 2> tangential_dis,
                   Matrix<double, 3, 3> leftToright, Matrix<double, 3, 3> R_new, int lorr);