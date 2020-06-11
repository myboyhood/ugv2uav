//
// Created by wzy on 2020/6/10.
//
#include <iostream>
#include "ugv2uav/ros_related.h"
#include "ugv2uav/UGV.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ugv2uav/custom_trans.h"
#include "math.h"
using namespace std;


#define PI (3.1415926535897932346f)

geometry_msgs::Pose ugv1_pose, ugv2_pose;
Eigen::Isometry3d ugv1_Trans,ugv2_Trans;
ugv2uav::custom_trans ugv1_trans_vec, ugv2_trans_vec;
nav_msgs::Odometry drone_odom;
UGV ugv1,ugv2;
Eigen::Matrix<double,4, 1> WTraj_s_long,WTraj_e_long,WTraj_s_short,WTraj_e_short;//!start point of long direction in World_Traj,  end point of short direction in World_Traj


void UGV_1_Pose_cb(const nav_msgs::OdometryConstPtr& msg)
{
    ugv1.ugv_odom = msg->pose.pose;

//    ugv1_pose = msg->pose.pose;
//    ugv1_Trans = Eigen::Isometry3d::Identity();
//    ugv1_Trans.rotate(Eigen::Quaterniond(ugv1_pose.orientation.w,ugv1_pose.orientation.x,ugv1_pose.orientation.y,ugv1_pose.orientation.z));
//    ugv1_Trans.pretranslate(Eigen::Vector3d(ugv1_pose.position.x,ugv1_pose.position.y,ugv1_pose.position.z));
//    cout << ugv1_Trans.matrix() << endl;
//    ugv1_trans_vec.a11 = ugv1_Trans(0,0); ugv1_trans_vec.a12 = ugv1_Trans(0,1);
//    ugv1_trans_vec.a13 = ugv1_Trans(0,2); ugv1_trans_vec.a14 = ugv1_Trans(0,3);
//
//    ugv1_trans_vec.a21 = ugv1_Trans(1,0); ugv1_trans_vec.a22 = ugv1_Trans(1,1);
//    ugv1_trans_vec.a23 = ugv1_Trans(1,2); ugv1_trans_vec.a24 = ugv1_Trans(1,3);
//
//    ugv1_trans_vec.a31 = ugv1_Trans(2,0); ugv1_trans_vec.a32 = ugv1_Trans(2,1);
//    ugv1_trans_vec.a33 = ugv1_Trans(2,2); ugv1_trans_vec.a34 = ugv1_Trans(2,3);
//
//    ugv1_trans_vec.a41 = ugv1_Trans(3,0); ugv1_trans_vec.a42 = ugv1_Trans(3,1);
//    ugv1_trans_vec.a43 = ugv1_Trans(3,2); ugv1_trans_vec.a44 = ugv1_Trans(3,3);

}

void UGV_2_Pose_cb(const nav_msgs::OdometryConstPtr& msg)
{
    ugv2.ugv_odom = msg->pose.pose;

//    ugv2_pose = msg->pose.pose;
//    ugv2_Trans = Eigen::Isometry3d::Identity();
//    ugv2_Trans.rotate(Eigen::Quaterniond(ugv2_pose.orientation.w,ugv2_pose.orientation.x,ugv2_pose.orientation.y,ugv2_pose.orientation.z));
//    ugv2_Trans.pretranslate(Eigen::Vector3d(ugv2_pose.position.x,ugv2_pose.position.y,ugv2_pose.position.z));
////    cout << "UGV2.ugv_pose.matrix():" << endl <<  UGV2.ugv_pose.matrix() << endl;
//    ugv2_trans_vec.a11 = ugv2_Trans(0,0); ugv2_trans_vec.a12 = ugv2_Trans(0,1);
//    ugv2_trans_vec.a13 = ugv2_Trans(0,2); ugv2_trans_vec.a14 = ugv2_Trans(0,3);
//
//    ugv2_trans_vec.a21 = ugv2_Trans(1,0); ugv2_trans_vec.a22 = ugv2_Trans(1,1);
//    ugv2_trans_vec.a23 = ugv2_Trans(1,2); ugv2_trans_vec.a24 = ugv2_Trans(1,3);
//
//    ugv2_trans_vec.a31 = ugv2_Trans(2,0); ugv2_trans_vec.a32 = ugv2_Trans(2,1);
//    ugv2_trans_vec.a33 = ugv2_Trans(2,2); ugv2_trans_vec.a34 = ugv2_Trans(2,3);
//
//    ugv2_trans_vec.a41 = ugv2_Trans(3,0); ugv2_trans_vec.a42 = ugv2_Trans(3,1);
//    ugv2_trans_vec.a43 = ugv2_Trans(3,2); ugv2_trans_vec.a44 = ugv2_Trans(3,3);
}


void drone_odom_cb(const nav_msgs::OdometryConstPtr & msg)
{
    drone_odom = *msg;
    ugv1.get_worldPoint(drone_odom.pose.pose.position.x,drone_odom.pose.pose.position.y,drone_odom.pose.pose.position.z);
    ugv2.get_worldPoint(drone_odom.pose.pose.position.x,drone_odom.pose.pose.position.y,drone_odom.pose.pose.position.z);

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "eigen_ugv_trans");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Subscriber ugv1_odom = n.subscribe("/ugv1/odom",1,UGV_1_Pose_cb);
    ros::Subscriber ugv2_odom = n.subscribe("/ugv2/odom",1,UGV_2_Pose_cb);
    ros::Subscriber uav_odom = n.subscribe("/hummingbird/ground_truth/odometry",5,drone_odom_cb);
    ros::Publisher ugv1_trans_pub = n.advertise<ugv2uav::custom_trans>("ugv1_trans_vec",1);
    ros::Publisher ugv2_trans_pub = n.advertise<ugv2uav::custom_trans>("ugv2_trans_vec",1);

//    //!====================================calculate  (u, v) in image plane======================================//!
//    //!camera internal matrix
//    Eigen::Matrix<double,3,4> K_ext;
//    K_ext.matrix() << 1206.89,0,960,0,
//                      0,1206.89,540,0,
//                      0,0,1,0;
//    //! camera to UGV installation transform matrix
//    Eigen::Isometry3d transform_cam2ugv;
//    Eigen::Vector3d euler_angles(0,-0.4,0);
//    Eigen::Matrix3d rotation_matrix;
//    rotation_matrix = Eigen::AngleAxisd(euler_angles[0],Eigen::Vector3d::UnitZ()) *
//                      Eigen::AngleAxisd(euler_angles[1],Eigen::Vector3d::UnitY()) *
//                      Eigen::AngleAxisd(euler_angles[2],Eigen::Vector3d::UnitX());
//    transform_cam2ugv = Eigen::Isometry3d::Identity();
//    transform_cam2ugv.rotate(rotation_matrix);
//    transform_cam2ugv.pretranslate(Eigen::Vector3d(0,0,0.1));//in order to convert to <4,4>matrix to multiply , use .matrix()
//    //! point in gazebo to camera coordinate
//    Eigen::Matrix4d gazebo2camera;
//    gazebo2camera << 0,-1,0,0,
//                     0,0,-1,0,
//                     1,0,0,0,
//                     0,0,0,1;
//
//
//    //! point in world coordinate
//    Eigen::Matrix<double,4,1> world_Point_ext;
//    //! point in pixel coordinate
//    Eigen::Matrix<double, 3, 1> pixel_uv_ext;
//    //!camera internal and external matrix
//    Eigen::Matrix<double, 3, 4> inter_exter_matrix;



    WTraj_s_long.matrix() << 2,0,1,1;
    WTraj_e_long.matrix() << 5,0,1,1;
    float score;


    while (ros::ok())
    {
//        ugv1_trans_pub.publish(ugv1_trans_vec);
//        ugv2_trans_pub.publish(ugv2_trans_vec);
//
//        world_Point_ext.matrix().Zero();
//        world_Point_ext.matrix() << drone_odom.pose.pose.position.x,
//                drone_odom.pose.pose.position.y,
//                drone_odom.pose.pose.position.z,
//                1;
//
//        cout << "world_Point_ext:" << world_Point_ext.matrix() << endl;
//        cout << "external matrix: " << transform_cam2ugv.matrix()* ugv2_Trans.matrix() << endl;
////        cout << "K_ext:" << K_ext << endl;
//        pixel_uv_ext = K_ext*  gazebo2camera * ((transform_cam2ugv.matrix() *ugv1_Trans.matrix()).inverse() *world_Point_ext);
//        pixel_uv_ext(0,0)/=pixel_uv_ext(2,0);
//        pixel_uv_ext(1,0)/=pixel_uv_ext(2,0);
//        pixel_uv_ext(2,0)=1;
//        cout << pixel_uv_ext.matrix() << endl;
        //! ugv1
        //!get yaw available interval
        ugv1.esti_yaw_interval(WTraj_s_long,WTraj_e_long);
        ugv1.find_max_Score(score,WTraj_s_long,WTraj_e_long);
        //! ugv2
//        ugv2.esti_yaw_interval(WTraj_s_long,WTraj_e_long);
//        ugv2.get_Score(score,WTraj_s_long,WTraj_e_long);

//        ugv1.get_uv_from_worldPoint();
//        ugv2.get_uv_from_worldPoint();
//        cout<< "ugv1.uv:" << endl << ugv1.pixel_uv_normal.x << "  "<< ugv1.pixel_uv_normal.y<< endl;
//        cout<< "ugv2.uv:" << endl << ugv2.pixel_uv_normal.x << "  "<< ugv2.pixel_uv_normal.y << endl;
//        ugv1.to_trans_vec_msg();
//        ugv2.to_trans_vec_msg();
//        ugv1_trans_pub.publish(ugv1.ugv_trans_vec);
//        ugv2_trans_pub.publish(ugv2.ugv_trans_vec);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
