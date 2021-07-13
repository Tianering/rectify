#include "rectify.h"

Matrix<Matrix<double, 3, 1>, 1, 4>
newPointsUndistort(Matrix<double, 3, 3> &intrinsic, cv::Mat &image, Matrix<double, 1, 6> radial_dis,
                   Matrix<double, 1, 2> tangential_dis) {
    double h = image.cols, w = image.rows;
    Matrix<Matrix<double, 3, 1>, 1, 4> points;
    points << Matrix<double, 3, 1>(0, 0, 1),
            Matrix<double, 3, 1>(0, w, 1),
            Matrix<double, 3, 1>(h, w, 1),
            Matrix<double, 3, 1>(h, 0, 1);
    points[0] = undistortion(points[0], radial_dis, tangential_dis);
    points[1] = undistortion(points[1], radial_dis, tangential_dis);
    points[2] = undistortion(points[2], radial_dis, tangential_dis);
    points[3] = undistortion(points[3], radial_dis, tangential_dis);
    // 倾斜平面矩阵*point点
    // 经过判断对x，y点进行放缩
    // xd*fx+cx
    // 得到倾斜平面的投影矩阵

    return points;
}


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
    Matrix<Matrix<double, 3, 1>, 1, 4> points = newPointsUndistort(intrinsic, left_img, left_radial_dis,
                                                                   left_tangential_dis);
    // 获取相机参数信息并取得新的旋转矩阵
    R_new = ParametersCamera(left_intrinsic, right_intrinsic, intrinsic, leftToright, translation_vector,
                             left_radial_dis,
                             right_radial_dis, left_tangential_dis, right_tangential_dis);
    // 左相机处理
    dst = myPolarRectify(left_img, res, left_intrinsic, intrinsic, left_radial_dis, left_tangential_dis, leftToright,
                         R_new, 0);
    // 右相机处理
    dst1 = myPolarRectify(right_img, res, right_intrinsic, intrinsic, right_radial_dis, right_tangential_dis,
                          leftToright, R_new, -1);
    imwrite("../images/left_2.png", dst);
    imwrite("../images/right_2.png", dst1);
    imwrite("../images/result_2.png", res);
    double end = omp_get_wtime();
    cout << "time:" << end - start << "s" << endl;
}
