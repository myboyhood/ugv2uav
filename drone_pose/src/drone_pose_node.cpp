//
// Created by wzy on 4/5/20.
//

#include "complx_tracker.h"

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
//add by wzy
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <chrono>

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
Mat showImg_1, showImg_2;
vector<int> compression_params;
//complx_tracker DroneTracker;

//!drone and UGV initiate
UGV UGV1,UGV2;

cv::Affine3d pose_raw_1, pose_raw_2;
Eigen::Isometry3d pose_world_1, drone_pos_gazebo_1, pose_world_2, drone_pos_gazebo_2;
Eigen::Quaterniond drone_pos_gazebo_Quat_1, drone_pos_gazebo_Quat_2;
geometry_msgs::PoseStamped msg_drone_pos_world_1, msg_drone_pos_gazebo_1, msg_pos_raw_1;
geometry_msgs::PoseStamped msg_drone_pos_world_2, msg_drone_pos_gazebo_2, msg_pos_raw_2;
geometry_msgs::Pose ugv1_pose, ugv2_pose;

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
        //! dynamical changing .ugv_pose according to ugv motion
        cout << "UGV1.ugv_pose: " << UGV1.ugv_pose.matrix() << endl;
        //! convert ugv_coordinate to gazebo_coordinate
        drone_pos_gazebo_1 = UGV1.ugv_pose * pose_world_1;
        cout << "drone_pos_gazebo_1:  " << endl << drone_pos_gazebo_1.matrix() << endl;
        drone_pos_gazebo_Quat_1 = drone_pos_gazebo_1.rotation();
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
        //! dynamical changing .ugv_pose according to ugv motion
        cout << "UGV2.ugv_pose: " << UGV2.ugv_pose.matrix() << endl;
        //! convert ugv_coordinate to gazebo_coordinate
        drone_pos_gazebo_2 = UGV2.ugv_pose * pose_world_2;
        cout << "drone_pos_gazebo_2:  " << endl << drone_pos_gazebo_2.matrix() << endl;
        drone_pos_gazebo_Quat_2 = drone_pos_gazebo_2.rotation();
        msg_drone_pos_gazebo_2.pose.orientation.w = drone_pos_gazebo_Quat_2.w();
        msg_drone_pos_gazebo_2.pose.orientation.x = drone_pos_gazebo_Quat_2.x();
        msg_drone_pos_gazebo_2.pose.orientation.y = drone_pos_gazebo_Quat_2.y();
        msg_drone_pos_gazebo_2.pose.orientation.z = drone_pos_gazebo_Quat_2.z();
        msg_drone_pos_gazebo_2.pose.position.x = drone_pos_gazebo_2.translation()[0];
        msg_drone_pos_gazebo_2.pose.position.y = drone_pos_gazebo_2.translation()[1];
        msg_drone_pos_gazebo_2.pose.position.z = drone_pos_gazebo_2.translation()[2];
        msg_drone_pos_gazebo_2.header.stamp = ros::Time::now();
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
        cout <<"time cb consume: "<< chrono::duration_cast<chrono::milliseconds>(end_time_cb - begin_time_cb).count() << endl;
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
        chrono::time_point<chrono::steady_clock> end_time_cb = chrono::steady_clock::now();
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

}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "gazebo_img");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    //! UGV1 sub
//    ros::Subscriber ugv1_img_sub = n.subscribe("/ugv1/camera/rgb/image_raw_L/compressed",3,UGV_1_Img_cb);
//    ros::Subscriber ugv1_pos_sub = n.subscribe("/ugv1/odom",3,UGV_1_Pose_cb);
    //! UGV2 sub
    ros::Subscriber ugv2_img_sub = n.subscribe("/ugv2/camera/rgb/image_raw_L/compressed",3,UGV_2_Img_cb);
    ros::Subscriber ugv2_pos_sub = n.subscribe("/ugv2/odom",3,UGV_2_Pose_cb);
    //! UGV1 pub
    ros::Publisher drone_pos_raw_1_pub = n.advertise<geometry_msgs::PoseStamped>("dronePoseRaw_1",1);
    ros::Publisher drone_pos_world_1_pub = n.advertise<geometry_msgs::PoseStamped>("dronePoseWorld_1",1);
    ros::Publisher drone_pos_gazebo_1_pub = n.advertise<geometry_msgs::PoseStamped>("dronePoseGazebo_1",1);
    //! UGV2 pub
    ros::Publisher drone_pos_raw_2_pub = n.advertise<geometry_msgs::PoseStamped>("dronePoseRaw_2",1);
    ros::Publisher drone_pos_world_2_pub = n.advertise<geometry_msgs::PoseStamped>("dronePoseWorld_2",1);
    ros::Publisher drone_pos_gazebo_2_pub = n.advertise<geometry_msgs::PoseStamped>("dronePoseGazebo_2",1);
//    ros::spin();
//    loop_rate.sleep();
    compression_params.push_back(IMWRITE_PNG_COMPRESSION);



    while(ros::ok())
    {
        drone_pos_raw_1_pub.publish(msg_pos_raw_1);
        drone_pos_world_1_pub.publish(msg_drone_pos_world_1);
        drone_pos_gazebo_1_pub.publish(msg_drone_pos_gazebo_1);
        drone_pos_raw_2_pub.publish(msg_pos_raw_2);
        drone_pos_world_2_pub.publish(msg_drone_pos_world_2);
        drone_pos_gazebo_2_pub.publish(msg_drone_pos_gazebo_2);
        ros::spinOnce();
//        resizeWindow("Frame", 500,500);
//        imshow("Frame",frame);
//        waitKey(2);
        loop_rate.sleep();
    }
    return 0;

}
