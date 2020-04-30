//
// Created by wzy on 4/25/20.
//
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;

#define PI (3.1415926535897932346f)

int main(int argc, char **argv)
{
    Eigen::AngleAxisd rotation_vector1 (PI/4,Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d rotation_matrix1 = Eigen::Matrix3d::Identity();
    //!tramsform matrix
    Eigen::Isometry3d transform_matrix2 = Eigen::Isometry3d::Identity();
    transform_matrix2.rotate(rotation_vector1);
    transform_matrix2.pretranslate(Eigen::Vector3d(2,2,2));
    rotation_matrix1 = rotation_vector1.matrix();
    //!eulerAngle to rotation_matrix
    Eigen::Vector3d euler_angles(0,-PI/4,0);
    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisd(euler_angles[0],Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(euler_angles[1],Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(euler_angles[2],Eigen::Vector3d::UnitX());
    Eigen::Vector3d Point_pre(1,0,1);
    Eigen::Vector3d Point_next;
    Point_next = rotation_matrix3 * Point_pre;
    cout << "Point_next" << Point_next << endl;
//    rotation_matrix2 = rotation_vector1.matrix();
    cout << "rotation matrix1 =\n" << rotation_matrix1 << endl;
    cout << "tramsform_matrix2 =\n" << transform_matrix2.matrix() << endl;
    cout << "rotation_matrix3 =\n" << rotation_matrix3 << endl;

    Eigen::Isometry3d Transf, pose1, pose2;
    Transf.matrix() << 1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 1,
                        0, 0, 0, 1;
    pose1.matrix() << 1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1;
    pose2 = Transf * pose1;
    pose1 = Transf.inverse() * pose2;
    cout << "pose2:  " << pose2.matrix() << endl;
    cout << "pose1:  " << pose1.matrix() << endl;


    return 0;
}