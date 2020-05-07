//
// Created by wzy on 4/5/20.
//

#include "complx_tracker.h"

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
//add by wzy
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <math.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <queue>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#define ugv1_x_distance 3;//ugv1 is located at -3m along x-axis,so subtract 3m to get drone position
using namespace cv;
using namespace std;

Mat frame;
Mat showImg_1, showImg_2, showImg_3;
vector<int> compression_params;
//complx_tracker DroneTracker;

//!drone and UGV initiate
UGV UGV1,UGV2;//observer drone and give drone position
UGV UGV3;//observer drone and give UGV position from drone position

cv::Affine3d pose_raw_1, pose_raw_2, pose_raw_3;
Eigen::Isometry3d pose_world_1, drone_pos_gazebo_1, pose_world_2, drone_pos_gazebo_2, pose_world_3, drone_pos_gazebo_3;
Eigen::Isometry3d drone_pos_gazebo_1_pre,drone_pos_gazebo_2_pre, drone_pos_gazebo_3_pre;
Eigen::Quaterniond drone_pos_gazebo_Quat_1, drone_pos_gazebo_Quat_2, drone_pos_gazebo_Quat_3;
geometry_msgs::PoseStamped msg_drone_pos_world_1, msg_drone_pos_gazebo_1, msg_pos_raw_1;
geometry_msgs::PoseStamped msg_drone_pos_world_2, msg_drone_pos_gazebo_2, msg_pos_raw_2;
geometry_msgs::PoseStamped msg_drone_pos_world_3, msg_drone_pos_gazebo_3, msg_pos_raw_3;
geometry_msgs::Pose ugv1_pose, ugv2_pose, ugv3_pose;
geometry_msgs::Pose drone_groundtruth;

geometry_msgs::PoseStamped drone_fuse_1, drone_fuse_2,drone_fuse, msg_UGV3_pose_cali;
geometry_msgs::Point msg_ypr1,msg_ypr2;

void Euler2MsgOrientation(Eigen::Vector3d& euler, geometry_msgs::PoseStamped& msgPose)
{
    Eigen::Quaterniond quaternion;
    quaternion = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX());
    quaternion.normalize();
    msgPose.pose.orientation.w = quaternion.w();msgPose.pose.orientation.x = quaternion.x();
    msgPose.pose.orientation.y = quaternion.y();msgPose.pose.orientation.z = quaternion.z();

}
void MsgOrientation2Euler(geometry_msgs::PoseStamped& msgPose, Eigen::Vector3d& euler)
{
    euler = Eigen::Quaterniond(msgPose.pose.orientation.w,
                               msgPose.pose.orientation.x,
                               msgPose.pose.orientation.y,
                               msgPose.pose.orientation.z).matrix().eulerAngles(2,1,0);
}

void MsgOrientation2Euler(geometry_msgs::Pose& msgPose, Eigen::Vector3d& euler)
{
    euler = Eigen::Quaterniond(msgPose.orientation.w,
                               msgPose.orientation.x,
                               msgPose.orientation.y,
                               msgPose.orientation.z).matrix().eulerAngles(2,1,0);
}

void UGV_1_Pose_estimate(Mat &SrcImg){
    //! pose_raw === drone_pose in camera_image coordinate
    //! pose_world === drone_pose in UGV coordinate
    //! pose_gazebo === drone pose in gazebo_env coordinate

    if(UGV1.DroneTracker.apply(SrcImg,showImg_1,pose_world_1,pose_raw_1))
    {   ROS_INFO("output translation");
        msg_pos_raw_1.pose.position.x = pose_raw_1.translation()[0];
        msg_pos_raw_1.pose.position.y = pose_raw_1.translation()[1];
        msg_pos_raw_1.pose.position.z = pose_raw_1.translation()[2];
        msg_pos_raw_1.header.stamp = ros::Time::now();
        msg_drone_pos_world_1.pose.position.x = pose_world_1.translation()[0];// according to gazebo coordinate
        msg_drone_pos_world_1.pose.position.y = pose_world_1.translation()[1];
        msg_drone_pos_world_1.pose.position.z = pose_world_1.translation()[2];
        msg_drone_pos_world_1.header.stamp = ros::Time::now();

//        cout << "UGV1.ugv_pose: " << UGV1.ugv_pose.matrix() << endl;

        //! smooth value, drop out unusual value
        if(abs(UGV1.ugv_pose.translation()[0]) != 0)//ensure UGV1.ugv_pose has meaningful value
        {   //! dynamical changing .ugv_pose according to ugv motion
            //! convert ugv_coordinate to gazebo_coordinate
            drone_pos_gazebo_1 = UGV1.ugv_pose * pose_world_1;
            if (drone_pos_gazebo_1.translation()[0] != 0 &&
                UGV1.first_initial)//drop None detected drone_pos at initial stage
            {
                drone_pos_gazebo_1_pre = drone_pos_gazebo_1;
                UGV1.first_initial = false;
            }
            UGV1.position_change = abs(drone_pos_gazebo_1.translation()[0] - drone_pos_gazebo_1_pre.translation()[0]) +
                                   abs(drone_pos_gazebo_1.translation()[1] - drone_pos_gazebo_1_pre.translation()[1]) +
                                   abs(drone_pos_gazebo_1.translation()[2] - drone_pos_gazebo_1_pre.translation()[2]);
            Eigen::Quaterniond drone_pos_gazebo_1_pre_Quan(drone_pos_gazebo_1_pre.rotation().matrix());
            UGV1.rpy_pre = drone_pos_gazebo_1_pre_Quan.matrix().eulerAngles(2, 1, 0);

            Eigen::Quaterniond drone_pos_gazebo_1_now_Quan(drone_pos_gazebo_1.rotation().matrix());
            UGV1.rpy_now = drone_pos_gazebo_1_now_Quan.matrix().eulerAngles(2, 1, 0);

            UGV1.rpy_change = abs(UGV1.rpy_now[0] - UGV1.rpy_pre[0]) + abs(UGV1.rpy_now[1] - UGV1.rpy_pre[1]) +
                              abs(UGV1.rpy_now[2] - UGV1.rpy_pre[2]);
            if (UGV1.rpy_change > UGV1.rpy_threshold ||
                UGV1.position_change > UGV1.position_threshold) { drone_pos_gazebo_1 = drone_pos_gazebo_1_pre; }
            drone_pos_gazebo_1_pre = drone_pos_gazebo_1;// iteration

//        cout << "drone_pos_gazebo_1:  " << endl << drone_pos_gazebo_1.matrix() << endl;
        drone_pos_gazebo_Quat_1 = drone_pos_gazebo_1.rotation();
        drone_pos_gazebo_Quat_1.normalize();
        msg_drone_pos_gazebo_1.pose.orientation.w = drone_pos_gazebo_Quat_1.w();
        msg_drone_pos_gazebo_1.pose.orientation.x = drone_pos_gazebo_Quat_1.x();
        msg_drone_pos_gazebo_1.pose.orientation.y = drone_pos_gazebo_Quat_1.y();
        msg_drone_pos_gazebo_1.pose.orientation.z = drone_pos_gazebo_Quat_1.z();
        msg_drone_pos_gazebo_1.pose.position.x = drone_pos_gazebo_1.translation()[0];
        msg_drone_pos_gazebo_1.pose.position.y = drone_pos_gazebo_1.translation()[1];
        msg_drone_pos_gazebo_1.pose.position.z = drone_pos_gazebo_1.translation()[2];
        msg_drone_pos_gazebo_1.header.stamp = ros::Time::now();
        }
    }
}

void UGV_2_Pose_estimate(Mat &SrcImg){
    if(UGV2.DroneTracker.apply(SrcImg,showImg_2,pose_world_2,pose_raw_2))
    {
        msg_pos_raw_2.pose.position.x = pose_raw_2.translation()[0];
        msg_pos_raw_2.pose.position.y = pose_raw_2.translation()[1];
        msg_pos_raw_2.pose.position.z = pose_raw_2.translation()[2];
        msg_pos_raw_2.header.stamp = ros::Time::now();
        msg_drone_pos_world_2.pose.position.x = pose_world_2.translation()[0];// according to gazebo coordinate
        msg_drone_pos_world_2.pose.position.y = pose_world_2.translation()[1];
        msg_drone_pos_world_2.pose.position.z = pose_world_2.translation()[2];
        msg_drone_pos_world_2.header.stamp = ros::Time::now();

        //! smooth value, drop out unusual value
        //! ensure UGV2.ugv_pose has meaningful value
        if(abs(UGV2.ugv_pose.translation()[0])+abs(UGV2.ugv_pose.translation()[1])+abs(UGV2.ugv_pose.translation()[2]) >0.01)
        {
            //! dynamical changing .ugv_pose according to ugv motion
            //! convert ugv_coordinate to gazebo_coordinate
            drone_pos_gazebo_2 = UGV2.ugv_pose * pose_world_2;
            if (drone_pos_gazebo_2.translation()[0] != 0 && UGV2.first_initial)//drop None detected drone_pos at initial stage
            {
                drone_pos_gazebo_2_pre = drone_pos_gazebo_2;
                UGV2.first_initial = false;
            }
            UGV2.position_change = abs(drone_pos_gazebo_2.translation()[0] - drone_pos_gazebo_2_pre.translation()[0]) +
                                   abs(drone_pos_gazebo_2.translation()[1] - drone_pos_gazebo_2_pre.translation()[1]) +
                                   abs(drone_pos_gazebo_2.translation()[2] - drone_pos_gazebo_2_pre.translation()[2]);
            Eigen::Quaterniond drone_pos_gazebo_2_pre_Quan(drone_pos_gazebo_2_pre.rotation().matrix());
            UGV2.rpy_pre = drone_pos_gazebo_2_pre_Quan.matrix().eulerAngles(2, 1, 0);
            Eigen::Quaterniond drone_pos_gazebo_2_now_Quan(drone_pos_gazebo_2.rotation().matrix());
            UGV2.rpy_now = drone_pos_gazebo_2_now_Quan.matrix().eulerAngles(2, 1, 0);
            UGV2.rpy_change = abs(UGV2.rpy_now[0] - UGV2.rpy_pre[0]) + abs(UGV2.rpy_now[1] - UGV2.rpy_pre[1]) +
                              abs(UGV2.rpy_now[2] - UGV2.rpy_pre[2]);
            if (UGV2.rpy_change > UGV2.rpy_threshold ||
                UGV2.position_change > UGV2.position_threshold) { drone_pos_gazebo_2 = drone_pos_gazebo_2_pre; }
            drone_pos_gazebo_2_pre = drone_pos_gazebo_2;// iteration

//        cout << "drone_pos_gazebo_2:  " << endl << drone_pos_gazebo_2.matrix() << endl;
        drone_pos_gazebo_Quat_2 = drone_pos_gazebo_2.rotation();
//        drone_pos_gazebo_Quat_2.normalize();
        msg_drone_pos_gazebo_2.pose.orientation.w = drone_pos_gazebo_Quat_2.w();
        msg_drone_pos_gazebo_2.pose.orientation.x = drone_pos_gazebo_Quat_2.x();
        msg_drone_pos_gazebo_2.pose.orientation.y = drone_pos_gazebo_Quat_2.y();
        msg_drone_pos_gazebo_2.pose.orientation.z = drone_pos_gazebo_Quat_2.z();
        msg_drone_pos_gazebo_2.pose.position.x = drone_pos_gazebo_2.translation()[0];
        msg_drone_pos_gazebo_2.pose.position.y = drone_pos_gazebo_2.translation()[1];
        msg_drone_pos_gazebo_2.pose.position.z = drone_pos_gazebo_2.translation()[2];
//        cout << "x: "<<msg_drone_pos_gazebo_2.pose.position.x<<endl<<
//        "y: "<<msg_drone_pos_gazebo_2.pose.position.y <<endl<<
//        "z: "<<msg_drone_pos_gazebo_2.pose.position.z <<endl;
        msg_drone_pos_gazebo_2.header.stamp = ros::Time::now();
        }
    }
}

void UGV_3_Pose_estimate(Mat &SrcImg){
    if(UGV3.DroneTracker.apply(SrcImg,showImg_3,pose_world_3,pose_raw_3))
    {
        msg_pos_raw_3.pose.position.x = pose_raw_3.translation()[0];
        msg_pos_raw_3.pose.position.y = pose_raw_3.translation()[1];
        msg_pos_raw_3.pose.position.z = pose_raw_3.translation()[2];
        msg_pos_raw_3.header.stamp = ros::Time::now();
        msg_drone_pos_world_3.pose.position.x = pose_world_3.translation()[0];// according to gazebo coordinate
        msg_drone_pos_world_3.pose.position.y = pose_world_3.translation()[1];
        msg_drone_pos_world_3.pose.position.z = pose_world_3.translation()[2];
        msg_drone_pos_world_3.header.stamp = ros::Time::now();
        //! dynamical changing .ugv_pose according to ugv motion
//        cout << "UGV2.ugv_pose: " << UGV2.ugv_pose.matrix() << endl;

        //! calibrate ugv3 pose
        Eigen::Matrix3d drone_fuse_rotationMatrix;
        Eigen::Isometry3d  drone_fuse_Isometry = Eigen::Isometry3d::Identity();
        Eigen::Quaterniond drone_fuse_Quat(drone_fuse.pose.orientation.w,drone_fuse.pose.orientation.x,drone_fuse.pose.orientation.y,drone_fuse.pose.orientation.z);
        drone_fuse_Quat.normalize();
        cout << "drone_fuse position : " << drone_fuse.pose.position <<endl;
        cout << "drone_fuse_Q: " << drone_fuse.pose.orientation << endl;
        //! pitch += 0.4
        Eigen::Vector3d drone_fuse_rpy;
        drone_fuse_rpy = drone_fuse_Quat.matrix().eulerAngles(2,1,0);
        cout << "drone_fuse_rpy: " << drone_fuse_rpy.transpose() << endl;
        drone_fuse_rpy.transpose()[1] += 0.4;
        drone_fuse_rotationMatrix = Eigen::AngleAxisd (drone_fuse_rpy[0],Eigen::Vector3d::UnitZ())*
                                    Eigen::AngleAxisd (drone_fuse_rpy[1], Eigen::Vector3d::UnitY())*
                                    Eigen::AngleAxisd (drone_fuse_rpy[2],Eigen::Vector3d::UnitX());
        //! z -= 0.12
        drone_fuse.pose.position.z -= 0.12;
        Eigen::Vector3d drone_fuse_tran(drone_fuse.pose.position.x,drone_fuse.pose.position.y,drone_fuse.pose.position.z);
        drone_fuse_Isometry.prerotate(drone_fuse_rotationMatrix);
        drone_fuse_Isometry.pretranslate(drone_fuse_tran);
        UGV3.ugv_cali_pose = Eigen::Isometry3d::Identity();
        UGV3.ugv_cali_pose = drone_fuse_Isometry * pose_world_3.inverse();
        cout << "drone_fuse_Isometry  " << endl << drone_fuse_Isometry.matrix() << endl;
        cout << "pose_world_3:  " << endl << pose_world_3.matrix() << endl;
        cout << "UGV3.ugv_cali_pose:  " << endl << UGV3.ugv_cali_pose.matrix() << endl;

        Eigen::Quaterniond ugv3_Quan(UGV3.ugv_cali_pose.rotation().matrix());
        msg_UGV3_pose_cali.header = msg_drone_pos_gazebo_1.header;
        msg_UGV3_pose_cali.pose.orientation.w = ugv3_Quan.w();
        msg_UGV3_pose_cali.pose.orientation.x = ugv3_Quan.x();
        msg_UGV3_pose_cali.pose.orientation.y = ugv3_Quan.y();
        msg_UGV3_pose_cali.pose.orientation.z = ugv3_Quan.z();
        msg_UGV3_pose_cali.pose.position.x = UGV3.ugv_cali_pose.translation()[0];
        msg_UGV3_pose_cali.pose.position.y = UGV3.ugv_cali_pose.translation()[1];
        msg_UGV3_pose_cali.pose.position.z = UGV3.ugv_cali_pose.translation()[2];


        //! estimate drone from UGV3 groundtruth pose
        if(abs(UGV3.ugv_pose.translation()[0])+abs(UGV3.ugv_pose.translation()[1])+abs(UGV3.ugv_pose.translation()[2]) >0.01)
        {
            //! estimate drone from UGV3 groundtruth pose  && smooth value, drop out unusual value
            //! convert ugv_coordinate to gazebo_coordinate
            drone_pos_gazebo_3 = UGV3.ugv_pose * pose_world_3;
            if (drone_pos_gazebo_3.translation()[0] != 0 && UGV3.first_initial)//drop None detected drone_pos at initial stage
            {
                drone_pos_gazebo_3_pre = drone_pos_gazebo_3;
                UGV3.first_initial = false;
            }
            UGV3.position_change = abs(drone_pos_gazebo_3.translation()[0] - drone_pos_gazebo_3_pre.translation()[0]) +
                                   abs(drone_pos_gazebo_3.translation()[1] - drone_pos_gazebo_3_pre.translation()[1]) +
                                   abs(drone_pos_gazebo_3.translation()[2] - drone_pos_gazebo_3_pre.translation()[2]);
            Eigen::Quaterniond drone_pos_gazebo_3_pre_Quan(drone_pos_gazebo_3_pre.rotation().matrix());
            UGV3.rpy_pre = drone_pos_gazebo_3_pre_Quan.matrix().eulerAngles(2, 1, 0);

            Eigen::Quaterniond drone_pos_gazebo_3_now_Quan(drone_pos_gazebo_3.rotation().matrix());
            UGV3.rpy_now = drone_pos_gazebo_3_now_Quan.matrix().eulerAngles(2, 1, 0);

            UGV3.rpy_change = abs(UGV3.rpy_now[0] - UGV3.rpy_pre[0]) + abs(UGV3.rpy_now[1] - UGV3.rpy_pre[1]) +
                              abs(UGV3.rpy_now[2] - UGV3.rpy_pre[2]);
            if (UGV3.rpy_change > UGV3.rpy_threshold ||
                UGV3.position_change > UGV3.position_threshold) { drone_pos_gazebo_3 = drone_pos_gazebo_3_pre; }
            drone_pos_gazebo_3_pre = drone_pos_gazebo_3;// iteration

//        cout << "drone_pos_gazebo_1:  " << endl << drone_pos_gazebo_1.matrix() << endl;
            drone_pos_gazebo_Quat_3 = drone_pos_gazebo_3.rotation();
            msg_drone_pos_gazebo_3.pose.orientation.w = drone_pos_gazebo_Quat_3.w();
            msg_drone_pos_gazebo_3.pose.orientation.x = drone_pos_gazebo_Quat_3.x();
            msg_drone_pos_gazebo_3.pose.orientation.y = drone_pos_gazebo_Quat_3.y();
            msg_drone_pos_gazebo_3.pose.orientation.z = drone_pos_gazebo_Quat_3.z();
            msg_drone_pos_gazebo_3.pose.position.x = drone_pos_gazebo_3.translation()[0];
            msg_drone_pos_gazebo_3.pose.position.y = drone_pos_gazebo_3.translation()[1];
            msg_drone_pos_gazebo_3.pose.position.z = drone_pos_gazebo_3.translation()[2];
            msg_drone_pos_gazebo_3.header.stamp = ros::Time::now();
        }
    }
}

void UGV_1_Img_cb(const sensor_msgs::CompressedImageConstPtr& msg)
{
//    ROS_INFO("HERE?");
    try
    {    chrono::time_point<chrono::steady_clock> begin_time_cb = chrono::steady_clock::now();
//        ROS_INFO("ready to copy to frame");
        Mat UGV_Img = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
        UGV_1_Pose_estimate(UGV_Img);
        chrono::time_point<chrono::steady_clock> end_time_cb = chrono::steady_clock::now();
//        cout <<"time cb consume: "<< chrono::duration_cast<chrono::milliseconds>(end_time_cb - begin_time_cb).count() << endl;
//        UGV_Img.copyTo(frame);
//        imwrite("bird01Img.png",frame,compression_params);
//        ROS_INFO("write frame");
//        imshow("Frame",frame);
//        resizeWindow("Frame", 500,500);
//        waitKey(3);
//        ROS_INFO("copy to frame");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert to image!");
    }
}

void UGV_2_Img_cb(const sensor_msgs::CompressedImageConstPtr& msg)
{
    try
    {
        Mat UGV_Img = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
        UGV_2_Pose_estimate(UGV_Img);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert to image!");
    }
}

void UGV_3_Img_cb(const sensor_msgs::CompressedImageConstPtr& msg)
{
    try
    {
        Mat UGV_Img = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
        UGV_3_Pose_estimate(UGV_Img);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert to image!");
    }
}

void UGV_1_Pose_cb(const nav_msgs::OdometryConstPtr& msg)
{
    ugv1_pose = msg->pose.pose;
    UGV1.ugv_pose = Eigen::Isometry3d::Identity();
    UGV1.ugv_pose.rotate(Eigen::Quaterniond(ugv1_pose.orientation.w,ugv1_pose.orientation.x,ugv1_pose.orientation.y,ugv1_pose.orientation.z));
    UGV1.ugv_pose.pretranslate(Eigen::Vector3d(ugv1_pose.position.x,ugv1_pose.position.y,ugv1_pose.position.z));

}

void UGV_2_Pose_cb(const nav_msgs::OdometryConstPtr& msg)
{
    ugv2_pose = msg->pose.pose;
    UGV2.ugv_pose = Eigen::Isometry3d::Identity();
    UGV2.ugv_pose.rotate(Eigen::Quaterniond(ugv2_pose.orientation.w,ugv2_pose.orientation.x,ugv2_pose.orientation.y,ugv2_pose.orientation.z));
    UGV2.ugv_pose.pretranslate(Eigen::Vector3d(ugv2_pose.position.x,ugv2_pose.position.y,ugv2_pose.position.z));
    cout << "UGV2.ugv_pose.matrix():" << endl <<  UGV2.ugv_pose.matrix() << endl;
}

void UGV_3_Pose_cb(const nav_msgs::OdometryConstPtr& msg)
{
    ugv3_pose = msg->pose.pose;
    UGV3.ugv_pose = Eigen::Isometry3d::Identity();
    UGV3.ugv_pose.rotate(Eigen::Quaterniond(ugv3_pose.orientation.w,ugv3_pose.orientation.x,ugv3_pose.orientation.y,ugv3_pose.orientation.z));
    UGV3.ugv_pose.pretranslate(Eigen::Vector3d(ugv3_pose.position.x,ugv3_pose.position.y,ugv3_pose.position.z));
}

void drone1_cb(const geometry_msgs::PoseStamped& msg)
{
    drone_fuse_1 = msg;
}

void drone2_cb(const geometry_msgs::PoseStamped& msg)
{
    drone_fuse_2 = msg;
}
void drone_pose_fuse(int numberOfdata, geometry_msgs::PoseStamped& drone_pose_1, geometry_msgs::PoseStamped& drone_pose_2, geometry_msgs::PoseStamped& drone_fusedPose)
{   //! To Do:
    //! 1. give different weight to each observed drone_pose
    drone_fusedPose.pose.position.x = (drone_pose_1.pose.position.x + drone_pose_2.pose.position.x)/numberOfdata;
    drone_fusedPose.pose.position.y = (drone_pose_1.pose.position.y + drone_pose_2.pose.position.y)/numberOfdata;
    drone_fusedPose.pose.position.z = (drone_pose_1.pose.position.z + drone_pose_2.pose.position.z)/numberOfdata;
//    cout << "drone_pose_1.pose.position.z: " << drone_pose_1.pose.position.z << endl <<
//    "drone_pose_2.pose.position.z: " << drone_pose_2.pose.position.z <<endl;
    Eigen::Vector3d ypr_1 = Eigen::Quaterniond(drone_pose_1.pose.orientation.w,
                                             drone_pose_1.pose.orientation.x,
                                             drone_pose_1.pose.orientation.y,
                                             drone_pose_1.pose.orientation.z).matrix().eulerAngles(2,1,0);
    Eigen::Vector3d ypr_2 = Eigen::Quaterniond(drone_pose_2.pose.orientation.w,
                                               drone_pose_2.pose.orientation.x,
                                               drone_pose_2.pose.orientation.y,
                                               drone_pose_2.pose.orientation.z).matrix().eulerAngles(2,1,0);
    msg_ypr1.x = ypr_1[0];msg_ypr1.y = ypr_1[1];msg_ypr1.z = ypr_1[2];
    msg_ypr2.x = ypr_1[0];msg_ypr2.y = ypr_2[1];msg_ypr2.z = ypr_2[2];
    Eigen::Vector3d ypr_fuse = (ypr_1 + ypr_2)/numberOfdata;
    Euler2MsgOrientation(ypr_fuse,drone_fusedPose);
//    Eigen::Quaterniond quaternion_fuse;
//    quaternion_fuse = Eigen::AngleAxisd(ypr_fuse[0], Eigen::Vector3d::UnitZ()) *
//                                    Eigen::AngleAxisd(ypr_fuse[1], Eigen::Vector3d::UnitY()) *
//                                    Eigen::AngleAxisd(ypr_fuse[2], Eigen::Vector3d::UnitX());
//    quaternion_fuse.normalize();
//    drone_fusedPose.pose.orientation.w = quaternion_fuse.w();drone_fusedPose.pose.orientation.x = quaternion_fuse.x();
//    drone_fusedPose.pose.orientation.y = quaternion_fuse.y();drone_fusedPose.pose.orientation.z = quaternion_fuse.z();

    drone_fusedPose.header = drone_pose_1.header;
}
void drone_truth_cb(const geometry_msgs::PoseConstPtr & msg)
{
    drone_groundtruth = *msg;
}

Eigen::Vector3d angle_constrain(Eigen::Vector3d angle)
{
    if(angle[0] > 2)
    {angle[0] = 3.1415926 - angle[0];
    } else if (angle[0] < -2)
    {angle[0] = 3.1415926 + angle[0];}

    if(angle[1] > 2)
    {angle[1] = 3.1415926 - angle[1];
    } else if (angle[1] < -2)
    {angle[1] = 3.1415926 + angle[1];}

    if(angle[2] > 2)
    {angle[2] = 3.1415926 - angle[2];
    } else if (angle[2] < -2)
    {angle[2] = 3.1415926 + angle[2];}

    return angle;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "gazebo_img");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    //! UGV1 sub
    ros::Subscriber ugv1_img_sub = n.subscribe("/ugv1/camera/rgb/image_raw_L/compressed",3,UGV_1_Img_cb);
    ros::Subscriber ugv1_pos_sub = n.subscribe("/ugv1/odom",3,UGV_1_Pose_cb);
    //! UGV2 sub
    ros::Subscriber ugv2_img_sub = n.subscribe("/ugv2/camera/rgb/image_raw_L/compressed",3,UGV_2_Img_cb);
    ros::Subscriber ugv2_pos_sub = n.subscribe("/ugv2/odom",3,UGV_2_Pose_cb);

    //! UGV3 sub
    ros::Subscriber ugv3_img_sub = n.subscribe("/ugv3/camera/rgb/image_raw_L/compressed",3,UGV_3_Img_cb);
    ros::Subscriber ugv3_pos_sub = n.subscribe("/ugv3/odom",3,UGV_3_Pose_cb);
    ros::Subscriber ugv3_dronePos1_sub = n.subscribe("/dronePoseGazebo_1",1,drone1_cb);
    ros::Subscriber ugv3_dronePos2_sub = n.subscribe("/dronePoseGazebo_2",1,drone2_cb);
    //! UGV1 pub
    ros::Publisher drone_pos_raw_1_pub = n.advertise<geometry_msgs::PoseStamped>("dronePoseRaw_1",1);
    ros::Publisher drone_pos_world_1_pub = n.advertise<geometry_msgs::PoseStamped>("dronePoseWorld_1",1);
    ros::Publisher drone_pos_gazebo_1_pub = n.advertise<geometry_msgs::PoseStamped>("dronePoseGazebo_1",1);
    ros::Publisher camera_rotate_pub = n.advertise<std_msgs::Float64>("ugv1/joint1_position_controller/command", 3);
    //! UGV2 pub
    ros::Publisher drone_pos_raw_2_pub = n.advertise<geometry_msgs::PoseStamped>("dronePoseRaw_2",1);
    ros::Publisher drone_pos_world_2_pub = n.advertise<geometry_msgs::PoseStamped>("dronePoseWorld_2",1);
    ros::Publisher drone_pos_gazebo_2_pub = n.advertise<geometry_msgs::PoseStamped>("dronePoseGazebo_2",1);
    //! drone_pos_fuse pub
    ros::Publisher drone_pos_fuse_pub = n.advertise<geometry_msgs::PoseStamped>("dronePosFuse",1);
    //! UGV3 pub
    ros::Publisher drone_pos_gazebo_3_pub = n.advertise<geometry_msgs::PoseStamped>("dronePoseGazebo_3",1);
    ros::Publisher ugv_pos_gazebo_3_pub = n.advertise<geometry_msgs::PoseStamped>("UGV3_PoseGazebo_cali",1);
    //! drone pose groundtruth
    ros::Subscriber drone_pos_truth_sub = n.subscribe("/hummingbird/ground_truth/pose",10,drone_truth_cb);
    //! ypr1 and ypr2 pub
    ros::Publisher ypr1_pub = n.advertise<geometry_msgs::Point>("ypr1",1);
    ros::Publisher ypr2_pub = n.advertise<geometry_msgs::Point>("ypr2",1);
    //! the param for writing image to file
    compression_params.push_back(IMWRITE_PNG_COMPRESSION);
    //! To Do : adding rotation platform on UGV
    std_msgs::Float64 camera_pos;
    ros::Time start_time = ros::Time::now();
    ros::Duration elapse;
    //! dertermine which UGV to be calibrated, default: false
    UGV3.calib_UGVself = true;
    //! initialize drone eulerAngle
    Eigen::Vector3d drone_eulerAngle1, drone_eulerAngle2, drone_eulerAngle3, drone_eulerAngle_fuse, drone_eulerAngle_truth;
    //! UGV3 eulerAngle
    Eigen::Vector3d UGV3_eulerAngle;
    //!file write
    ofstream trajectory_esti_file_1,trajectory_esti_file_2, trajectory_esti_file_3, trajectory_esti_file_fuse,trajectory_truth_file;
    ofstream UGV3_pose_file;
    chrono::time_point<chrono::steady_clock> begin_time_cb = chrono::steady_clock::now();
    trajectory_esti_file_1.open("/home/wzy/catkin_ws/src/drone_pose/trajectory/ugv1_esti_pos.txt");
    trajectory_esti_file_2.open("/home/wzy/catkin_ws/src/drone_pose/trajectory/ugv2_esti_pos.txt");
    trajectory_esti_file_3.open("/home/wzy/catkin_ws/src/drone_pose/trajectory/ugv3_esti_pos.txt");
    trajectory_esti_file_fuse.open("/home/wzy/catkin_ws/src/drone_pose/trajectory/fuse_esti_pos.txt");
    trajectory_truth_file.open("/home/wzy/catkin_ws/src/drone_pose/trajectory/truth_pos.txt");
    UGV3_pose_file.open("/home/wzy/catkin_ws/src/drone_pose/trajectory/ugv3_pose.txt");
    bool close_file = true;

    while(ros::ok())
    {   ROS_INFO("start loop");
        elapse = ros::Time::now() - start_time;
        drone_pose_fuse(2,drone_fuse_1,drone_fuse_2,drone_fuse);
        //! eulerAngle
        MsgOrientation2Euler(msg_drone_pos_gazebo_1,drone_eulerAngle1);
        MsgOrientation2Euler(msg_drone_pos_gazebo_2,drone_eulerAngle2);
        MsgOrientation2Euler(msg_drone_pos_gazebo_3,drone_eulerAngle3);
        MsgOrientation2Euler(drone_fuse,drone_eulerAngle_fuse);
        MsgOrientation2Euler(drone_groundtruth,drone_eulerAngle_truth);
        MsgOrientation2Euler(msg_UGV3_pose_cali,UGV3_eulerAngle);
//        drone_eulerAngle1 = Eigen::Quaterniond(msg_drone_pos_gazebo_1.pose.orientation.w,
//                                               msg_drone_pos_gazebo_1.pose.orientation.x,
//                                               msg_drone_pos_gazebo_1.pose.orientation.y,
//                                               msg_drone_pos_gazebo_1.pose.orientation.z).matrix().eulerAngles(2,1,0);
//        drone_eulerAngle2 = Eigen::Quaterniond(msg_drone_pos_gazebo_2.pose.orientation.w,
//                                               msg_drone_pos_gazebo_2.pose.orientation.x,
//                                               msg_drone_pos_gazebo_2.pose.orientation.y,
//                                               msg_drone_pos_gazebo_2.pose.orientation.z).matrix().eulerAngles(2,1,0);
//        drone_eulerAngle3 = Eigen::Quaterniond(msg_drone_pos_gazebo_3.pose.orientation.w,
//                                               msg_drone_pos_gazebo_3.pose.orientation.x,
//                                               msg_drone_pos_gazebo_3.pose.orientation.y,
//                                               msg_drone_pos_gazebo_3.pose.orientation.z).matrix().eulerAngles(2,1,0);
//        drone_eulerAngle_fuse = Eigen::Quaterniond(drone_fuse.pose.orientation.w,
//                                                   drone_fuse.pose.orientation.x,
//                                                   drone_fuse.pose.orientation.y,
//                                                   drone_fuse.pose.orientation.z).matrix().eulerAngles(2,1,0);
//        drone_eulerAngle_truth = Eigen::Quaterniond(drone_groundtruth.orientation.w,
//                                               drone_groundtruth.orientation.x,
//                                               drone_groundtruth.orientation.y,
//                                               drone_groundtruth.orientation.z).matrix().eulerAngles(2,1,0);
//        UGV3_eulerAngle = Eigen::Quaterniond(msg_UGV3_pose_cali.pose.orientation.w,
//                                             msg_UGV3_pose_cali.pose.orientation.x,
//                                             msg_UGV3_pose_cali.pose.orientation.y,
//                                             msg_UGV3_pose_cali.pose.orientation.z).matrix().eulerAngles(2,1,0);

        //! angle constrain in -3.14 ~ 3.14
        drone_eulerAngle1 = angle_constrain(drone_eulerAngle1);
        drone_eulerAngle2 = angle_constrain(drone_eulerAngle2);
        drone_eulerAngle_fuse = angle_constrain(drone_eulerAngle_fuse);
        drone_eulerAngle_truth = angle_constrain(drone_eulerAngle_truth);
        UGV3_eulerAngle = angle_constrain(UGV3_eulerAngle);
        //! correct gazebo Z axis height error and Pitch error
        msg_drone_pos_gazebo_1.pose.position.z += 0.12;
        msg_drone_pos_gazebo_2.pose.position.z += 0.12;
        msg_drone_pos_gazebo_3.pose.position.z += 0.12;
        drone_eulerAngle1[1] -=0.4;
        drone_eulerAngle2[1] -=0.4;
        drone_eulerAngle3[1] -=0.4;
        Euler2MsgOrientation(drone_eulerAngle1,msg_drone_pos_gazebo_1);
        Euler2MsgOrientation(drone_eulerAngle2,msg_drone_pos_gazebo_2);
        Euler2MsgOrientation(drone_eulerAngle3,msg_drone_pos_gazebo_3);

        if(trajectory_esti_file_1.is_open())
        {   // drop the first several lines with 0
            if(msg_drone_pos_gazebo_1.pose.position.x != 0 ) {
                trajectory_esti_file_1 << msg_drone_pos_gazebo_1.header.stamp.sec << " " <<
                                       msg_drone_pos_gazebo_1.pose.position.x << " " <<
                                       msg_drone_pos_gazebo_1.pose.position.y << " " <<
                                       msg_drone_pos_gazebo_1.pose.position.z << " " <<
                                       drone_eulerAngle1.transpose() << " " << endl;
            }
        }
        if(trajectory_esti_file_2.is_open())
        {
            if(msg_drone_pos_gazebo_2.pose.position.x != 0 ) {
                trajectory_esti_file_2 << msg_drone_pos_gazebo_2.header.stamp.sec << " " <<
                                       msg_drone_pos_gazebo_2.pose.position.x << " " <<
                                       msg_drone_pos_gazebo_2.pose.position.y << " " <<
                                       msg_drone_pos_gazebo_2.pose.position.z << " " <<
                                       drone_eulerAngle2.transpose() << " " << endl;
            }
        }
        if(trajectory_esti_file_3.is_open())
        {
            if(msg_drone_pos_gazebo_3.pose.position.x != 0 ) {
                trajectory_esti_file_3 << msg_drone_pos_gazebo_3.header.stamp.sec << " " <<
                                       msg_drone_pos_gazebo_3.pose.position.x << " " <<
                                       msg_drone_pos_gazebo_3.pose.position.y << " " <<
                                       msg_drone_pos_gazebo_3.pose.position.z << " " <<
                                       drone_eulerAngle3.transpose() << " " << endl;
            }
        }
        if(trajectory_esti_file_fuse.is_open())
        {
            if(drone_fuse.pose.position.x != 0 ) {
                trajectory_esti_file_fuse << drone_fuse.header.stamp.sec << " " <<
                                          drone_fuse.pose.position.x << " " <<
                                          drone_fuse.pose.position.y << " " <<
                                          drone_fuse.pose.position.z << " " <<
                                          drone_eulerAngle_fuse.transpose() << " " << endl;
            }
        }

        if(trajectory_truth_file.is_open())
        {
            trajectory_truth_file << msg_drone_pos_gazebo_1.header.stamp.sec << " " <<
                                     drone_groundtruth.position.x << " " <<
                                     drone_groundtruth.position.y << " " <<
                                     drone_groundtruth.position.z << " " <<
                                     drone_eulerAngle_truth.transpose() << " " <<endl;
        }

        if(UGV3_pose_file.is_open())
        {
            UGV3_pose_file << msg_UGV3_pose_cali.header.stamp.sec << " " <<
                               msg_UGV3_pose_cali.pose.position.x << " " <<
                               msg_UGV3_pose_cali.pose.position.y << " " <<
                               msg_UGV3_pose_cali.pose.position.z << " " <<
                               UGV3_eulerAngle.transpose() << " " <<endl;
        }
        drone_pos_raw_1_pub.publish(msg_pos_raw_1);
        drone_pos_world_1_pub.publish(msg_drone_pos_world_1);
        drone_pos_gazebo_1_pub.publish(msg_drone_pos_gazebo_1);

        drone_pos_raw_2_pub.publish(msg_pos_raw_2);
        drone_pos_world_2_pub.publish(msg_drone_pos_world_2);
        drone_pos_gazebo_2_pub.publish(msg_drone_pos_gazebo_2);

        drone_pos_fuse_pub.publish(drone_fuse);
        drone_pos_gazebo_3_pub.publish(msg_drone_pos_gazebo_3);
        ugv_pos_gazebo_3_pub.publish(msg_UGV3_pose_cali);

        ypr1_pub.publish(msg_ypr1);
        ypr2_pub.publish(msg_ypr2);
        camera_pos.data = 3.1415926*0.5*sin(2*3.1415926*0.1*elapse.toSec());
        camera_rotate_pub.publish(camera_pos);
        ros::spinOnce();
//        resizeWindow("Frame", 500,500);
//        imshow("Frame",frame);
//        waitKey(2);
        loop_rate.sleep();
        chrono::time_point<chrono::steady_clock> end_time_cb = chrono::steady_clock::now();

        if(chrono::duration_cast<chrono::seconds>(end_time_cb - begin_time_cb).count() > 40) {
            ROS_WARN("------------------------------FILE CLOSED------------------------------");
            if (close_file)
            {
                trajectory_esti_file_1.close();
                trajectory_esti_file_2.close();
                trajectory_esti_file_3.close();
                trajectory_esti_file_fuse.close();
                trajectory_truth_file.close();
                close_file = false;
            }
        }
        ROS_INFO("end loop");
    }

    return 0;

}
