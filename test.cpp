////
//// Created by shanyi on 2021/7/9.
////
//
//#include "rectify.h"
//
//void newPointsUndistort(Matrix<double, 3, 3> &intrinsic, cv::Mat &image, Matrix<double, 1, 6> radial_dis,
//                        Matrix<double, 1, 2> tangential_dis) {
//    double h = image.cols, w = image.rows;
//    Matrix<Matrix<double, 3, 1>, 1, 4> points;
//    points << Matrix<double, 3, 1>(0, 0, 1),
//            Matrix<double, 3, 1>(0, w, 1),
//            Matrix<double, 3, 1>(h, w, 1),
//            Matrix<double, 3, 1>(h, 0, 1);
//    points[0] = undistortion(points[0], radial_dis, tangential_dis);
//}
//
//int main() {
//    Vec4d k = Vec4d::all(0);
//    double w = 1920, h = 960;
//    cv::Mat points(1, 4, CV_64FC2);
//    Vec2d *pptr = points.ptr<Vec2d>();
//    pptr[0] = Vec2d(w / 2, 0);
//    pptr[1] = Vec2d(w, h / 2);
//    pptr[2] = Vec2d(w / 2, h);
//    pptr[3] = Vec2d(0, h / 2);
//    cout << points << endl;
////    undistortPoints(points, points, K, D, R);
//    cv::Scalar center_mass = mean(points);// 质心
//    cv::Vec2d cn(center_mass.val);
//    cout << "pptr0:\n" << pptr[0] << endl;
//    cout << center_mass << endl;
//    cout << cn << endl;
//    //fisheye::undistortPoints(points, points, K, D, R);
//    double aspect_ratio = 1600.3 / 1600.65;// 纵横比
//    cn[0] *= aspect_ratio;
//    cout << "纵横比后：\n" << cn << endl;
//
//    double minx = 0, miny = 0, maxx = 960, maxy = 480;
//    double f1 = w * 0.5 / (cn[0] - minx);
//    double f2 = w * 0.5 / (maxx - cn[0]);
//    double f3 = h * 0.5 * aspect_ratio / (cn[1] - miny);
//    double f4 = h * 0.5 * aspect_ratio / (maxy - cn[1]);
//
//    double fmin = std::min(f1, std::min(f2, std::min(f3, f4)));
//    double fmax = std::max(f1, std::max(f2, std::max(f3, f4)));
//
//    double f = fmin;
//    cout << fmin << "," << fmax << endl;
//    cv::Vec2d new_f(f, f), new_c = -cn * f + Vec2d(w, h * aspect_ratio) * 0.5;
//    cout << "新的焦距和主点：" << endl;
//    cout << new_f << "," << new_c << endl;
//    new_f[1] /= aspect_ratio;
//    new_c[1] /= aspect_ratio;
//    double rx = 1;
//    double ry = 1;
//
//    new_f[0] *= rx;
//    new_f[1] *= ry;
//    new_c[0] *= rx;
//    new_c[1] *= ry;
//    cout << "新的焦距和主点：" << endl;
//    cout << new_f << "," << new_c << endl;
//    Mat P;
//    cout << Mat(Matx33d(new_f[0], 0, new_c[0],
//                        0, new_f[1], new_c[1],
//                        0, 0, 1)) << endl;
//}