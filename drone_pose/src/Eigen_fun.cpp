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
    Eigen::Vector3d euler_angles(-PI/2,0,0);
    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisd(euler_angles[0],Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(euler_angles[1],Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(euler_angles[2],Eigen::Vector3d::UnitX());
    cout << "rotation_matrix3.matrix() " << endl << rotation_matrix3.matrix() << endl;
    Eigen::Vector3d Point_pre(1,0,1);
    Eigen::Vector3d Point_next;
    Point_next = rotation_matrix3 * Point_pre;
//    cout << "Point_next" << Point_next << endl;
////    rotation_matrix2 = rotation_vector1.matrix();
//    cout << "rotation matrix1 =\n" << rotation_matrix1 << endl;
//    cout << "tramsform_matrix2 =\n" << transform_matrix2.matrix() << endl;
//    cout << "rotation_matrix3 =\n" << rotation_matrix3 << endl;

    Eigen::Isometry3d Transf, pose1, pose2, drone_fuse_Isometry, pose_world_3, ugv_cali_pose, UGV;
    drone_fuse_Isometry = Eigen::Isometry3d::Identity();
    Eigen::Vector3d euler0 (1.57202, 1.56796,-1.56717);
    Eigen::Vector3d euler (PI/2, 0,0);
    Eigen::Vector3d euler1;
    cout << "euler1:  " << (euler0 + euler)/2 << endl;
//    Eigen::Vector3d euler (0, 0,0);
//    euler[1] += 0.4;
    cout << "euler: " << euler << endl;
    Eigen::Matrix3d rotation_matrix4;
    Eigen::AngleAxisd z_axis(euler[0],Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd y_axis(euler[1],Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd x_axis(euler[2],Eigen::Vector3d::UnitX());
    cout << "z_axis.matrix()"<< endl << z_axis.toRotationMatrix() << endl;
    cout << "y_axis.matrix()"<< endl << y_axis.matrix() << endl;
    cout << "x_axis.matrix()"<< endl << x_axis.matrix() << endl;
    Eigen::Vector3d p1(1,0,0),p2;
    p2 = z_axis.matrix() * p1;
    cout << "p2: " <<p2.transpose() << endl;
    rotation_matrix4 = Eigen::AngleAxisd(euler[0],Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(euler[1],Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(euler[2],Eigen::Vector3d::UnitX());
    Eigen::Quaterniond quan(rotation_matrix4);
    cout <<"w x y z" <<"   "<<quan.w() <<"   "<< quan.x() <<"   "<< quan.y() <<"   "<< quan.z() <<"   "<< endl;
    drone_fuse_Isometry.rotate(rotation_matrix4);
    drone_fuse_Isometry.pretranslate(Eigen::Vector3d(2.5,-0.5,0.95));
    cout << "drone_fuse_Isometry.matrix(): "<<endl<<drone_fuse_Isometry.matrix() << endl;

    Transf.matrix() << 1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 1,
                        0, 0, 0, 1;
    pose1.matrix() << 1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1;
//    drone_fuse_Isometry.matrix() <<
//    0.920819, -0.00446924,   -0.389966,     2.49067,
//    -0.00341708,   -0.999988,  0.00339177,   -0.527909,
//    -0.389976, -0.00179067,   -0.920823,    0.955705,
//    0,           0 ,          0,        1;
    pose_world_3.matrix() <<
    -0.037364,  -0.999229,  -0.012043,    3.53301,
    0.917009, -0.0390737,   0.396949,  -0.384137,
    -0.397113, 0.00378802,   0.917762,   0.836022,
    0,          0,          0,          1;
    ugv_cali_pose.matrix() <<  -0.0252433,    0.689777,   -0.723582,     3.44976,
    0.999304,   0.0372861, 0.000681834,     -4.0447,
    0.0274498,   -0.723061,   -0.690238,     1.15802,
    0 ,          0 ,          0 ,          1;
    UGV = Eigen::Isometry3d::Identity();
    UGV = drone_fuse_Isometry * pose_world_3.inverse();
    cout << "UGV:" << endl << UGV.matrix() << endl;
    pose2 = Transf * pose1;
    pose1 = Transf.inverse() * pose2;
    cout << "pose2:  " << pose2.matrix() << endl;
    cout << "pose1:  " << pose1.matrix() << endl;


    return 0;
}