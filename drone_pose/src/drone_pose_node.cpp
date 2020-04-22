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

//add by wzy
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <queue>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

using namespace cv;
using namespace std;

Mat frame;
Mat showImg;
vector<int> compression_params;
complx_tracker DroneTracker;
Affine3d pose_world, pose_raw;
geometry_msgs::PoseStamped msg_pos, msg_pos_old, msg_pos_raw, msg_pos_raw_old;

void Pose_estimate(Mat &SrcImg){
    if(DroneTracker.apply(SrcImg,showImg,pose_world,pose_raw))
    {   ROS_INFO("output translation");
        msg_pos_raw.pose.position.x = pose_raw.translation()[0];
        msg_pos_raw.pose.position.y = pose_raw.translation()[1];
        msg_pos_raw.pose.position.z = pose_raw.translation()[2];
        msg_pos_raw.header.stamp = ros::Time::now();
    }
}

void UGVImg_cb(const sensor_msgs::CompressedImageConstPtr& msg)
{
//    ROS_INFO("HERE?");
    try
    {
//        ROS_INFO("ready to copy to frame");
        Mat UGV_Img = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
        Pose_estimate(UGV_Img);

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


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "gazebo_img");
    ros::NodeHandle n;
    ros::Rate loop_rate(6);

    ros::Subscriber ugv_img_sub = n.subscribe("/ugv1/camera/rgb/image_raw_L/compressed",10,UGVImg_cb);
    ros::Publisher pos_pub = n.advertise<geometry_msgs::PoseStamped>("dronePose",5);
//    ros::spin();
//    loop_rate.sleep();
    compression_params.push_back(IMWRITE_PNG_COMPRESSION);


    while(ros::ok())
    {
        pos_pub.publish(msg_pos_raw);
        ros::spinOnce();
//        resizeWindow("Frame", 500,500);
//        imshow("Frame",frame);
//        waitKey(2);
        loop_rate.sleep();
    }
    return 0;

}
