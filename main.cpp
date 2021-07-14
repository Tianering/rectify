#include "rectify.h"

int main() {
    double start = omp_get_wtime();
    Matrix<double, 3, 3> left_intrinsic, right_intrinsic, intrinsic, R_new;
    Matrix<double, 3, 3> leftToright;
    Matrix<double, 3, 1> translation_vector;
    Matrix<double, 1, 6> left_radial_dis, right_radial_dis;
    Matrix<double, 1, 2> left_tangential_dis, right_tangential_dis;
    cv::Mat right_img = imread("../images/R_2.bmp");
    cv::Mat left_img = imread("../images/L_2.bmp");
    cv::Mat dst = Mat(left_img.rows, left_img.cols, CV_8UC3, Scalar(0));
    cv::Mat dst1 = Mat(right_img.rows, right_img.cols, CV_8UC3, Scalar(0));
    cv::Mat res = Mat(left_img.rows, left_img.cols * 2, CV_8UC3, Scalar(0));
    // 获取相机参数信息并取得新的旋转矩阵
    R_new = ParametersCamera(left_intrinsic, right_intrinsic, intrinsic, leftToright, translation_vector,
                             left_radial_dis,
                             right_radial_dis, left_tangential_dis, right_tangential_dis);
//    // 左相机处理
//    dst = myPolarRectify(left_img, res, left_intrinsic, intrinsic, left_radial_dis, left_tangential_dis, leftToright,
//                         R_new, 0);
//    // 右相机处理
//    dst1 = myPolarRectify(right_img, res, right_intrinsic, intrinsic, right_radial_dis, right_tangential_dis,
//                          leftToright, R_new, -1);
//-------------------------------映射表形式
    cv::Mat mapx_l = Mat(left_img.rows, left_img.cols, CV_64F, Scalar(0));
    cv::Mat mapy_l = Mat(left_img.rows, left_img.cols, CV_64F, Scalar(0));
    int x,y;
    double *mpx = mapx_l.ptr<double>(1079);
    mapRectify(left_img, intrinsic, left_intrinsic, left_radial_dis, left_tangential_dis, leftToright, R_new, 0, mapx_l,
               mapy_l);

    cv::Mat mapx_r = Mat(right_img.rows, right_img.cols, CV_64F, Scalar(0));
    cv::Mat mapy_r = Mat(right_img.rows, right_img.cols, CV_64F, Scalar(0));
    mapRectify(right_img, intrinsic, right_intrinsic, right_radial_dis, right_tangential_dis, leftToright, R_new, 0,
               mapx_r, mapy_r);

    cout << mpx[1919]<<endl;
    cout << mapy_l.ptr<double>(1079) [1919]<<endl;
    remapRectify(left_img, dst, res, mapx_l, mapy_l, 0);
    remapRectify(right_img, dst1, res, mapx_r, mapy_r, -1);

    imwrite("../images/left_2.png", dst);
    imwrite("../images/right_2.png", dst1);
    imwrite("../images/result_2.png", res);
    double end = omp_get_wtime();
    cout << "time:" << end - start << "s" << endl;
}
