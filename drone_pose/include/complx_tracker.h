//
// Created by ljy on 19-4-22.
//

#ifndef SRC_COMPLX_TRACKER_H
#define SRC_COMPLX_TRACKER_H
#include "ros/ros.h"
#include "ros/package.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/video/background_segm.hpp>
#include "opencv2/bgsegm.hpp"
#include <vector>
#include <opencv2/viz.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "math.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#define STRUCTURE_INDEX 1
#define DETECTING_METHOD 1
#define PI (3.1415926535897932346f)


class complx_tracker;
class Drone{
public:
    friend class complex_tracker;
    cv::Point2f white_p, blue_p, green_p, red_p;
    cv::Point3f WHITE_P, BLUE_P, GREEN_P, RED_P;
};




/*
class myImgPoint : public cv::Point2f
{
public:
    float angle_with_center;
    float length_to_next_point;

    myImgPoint(float x, float y) : cv::Point2f(x, y) {}

    bool operator<(const myImgPoint &myImgPoint1) const {
        return angle_with_center < myImgPoint1.angle_with_center;
    }

    bool operator>(const myImgPoint &myImgPoint1) const {
        return angle_with_center > myImgPoint1.angle_with_center;
    }
    static float dist_btn_pts(cv::Point2f& p1, cv::Point2f& p2) {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) * 1.0);
    }
};
*/


/*
class pointsDetector {
public:
    cv::String algo="MOG";
    int count=0;
    std::vector<cv::Point2f> sortedImgPts;
    cv::Mat contoursImg,valueImg;
    std::vector<cv::Mat> imgRec;
    cv::Mat BdilatedImg, GdilatedImg,RdilatedImg;
    cv::Mat BerodedImg, GerodedImg, RerodedImg;
    std::vector<std::vector<cv::Point>> contours, contours_org_BW,contours_org_GW,contours_org_RW;
    std::vector<cv::Point2f> BMomentCenters,GMomentCenters,RMomentCenters;
    cv::Mat fgMask, frame_masked, binImg, update_img, result,BlueImg,GreenImg,RedImg,BbinImg,GbinImg,RbinImg;

    pointsDetector();
    int getPoints(const cv::Mat &frame, cv::Point2f &BlueoutputPoint,cv::Point2f &GreenoutputPoint,
                   cv::Point2f &RedoutputPoint,cv::Point2f &WhiteoutputPoint, bool using_points_position_guess = false);
    bool updateFrame(const cv::Mat &frame, bool using_points_position_guess);
    bool prefollowPoints(const cv::Mat & frame);
    //! drone class
    drone drone_0;

private:

    cv::Ptr<cv::BackgroundSubtractor> pBackSub;


};
*/

/*
class pointsFollower{
public:
    cv::Mat binImg,prevbinImg;
    std::vector<cv::Point2f> prevImgPts,nextImgPts;
    cv::Mat imgDronemarker;

    bool followPoints(const cv::Mat &frame, std::vector<cv::Point2f> &outputPointsVector);
};
*/


class complx_tracker{

public:

    cv::Mat cameraMatrix;
    std::vector<double> distCoeffs;
    ros::Time start_time;
    std::vector<cv::Point3f> objectPts;
    bool following_flag=false;
    std::vector<cv::Point2f> imgPts, BlueimgPts,GreenimgPts,RedimgPts;
    cv::Point2f BimgPts, GimgPts, RimgPts,WimgPts;
    cv::Affine3d camPose, pose, world_Affine_pose;
    Eigen::Isometry3d transform_cam;
    Eigen::Matrix3d camera2ugv_transform;
    cv::Vec3d outputRvecRaw,outputTvecRaw,outputRvec,outputTvec;
    Eigen::Vector3d tvec,rvec,worldtvec,worldrvec;
    Eigen::Matrix3d world_rMatrix;
    int firstStage = 1;
    bool enablePnP = false;

    //!points detection
//    pointsDetector pts_detector;
    //! img to show contours center

    cv::Mat img;
    cv::Mat showImg;
    bool followImgPtsSuccess;
    complx_tracker();
    bool apply(const cv::Mat &frame, cv::Mat &img_to_show, Eigen::Isometry3d &pose_world, cv::Affine3d &pose_raw);
    bool findImgPts();
    void virtualize();

    //!Points detector
    cv::String algo="MOG";
    int count=0;
    std::vector<cv::Point2f> sortedImgPts;
    cv::Mat contoursImg,valueImg;
    std::vector<cv::Mat> imgRec;
    cv::Mat BdilatedImg, GdilatedImg,RdilatedImg;
    cv::Mat BerodedImg, GerodedImg, RerodedImg;
    std::vector<std::vector<cv::Point>> contours, contours_org_BW,contours_org_GW,contours_org_RW;
    std::vector<cv::Point2f> BMomentCenters,GMomentCenters,RMomentCenters;
    cv::Mat fgMask, frame_masked, binImg, update_img, result,BlueImg,GreenImg,RedImg,BbinImg,GbinImg,RbinImg;
    void pointsDetector();
    int getPoints(const cv::Mat &frame, cv::Point2f &BlueoutputPoint,cv::Point2f &GreenoutputPoint,
                  cv::Point2f &RedoutputPoint,cv::Point2f &WhiteoutputPoint, bool using_points_position_guess = false);
    bool updateFrame(const cv::Mat &frame, bool using_points_position_guess);
    bool prefollowPoints(const cv::Mat & frame);



private:
    cv::viz::Viz3d myWindow;
    cv::viz::WCube cube_widget1;
    cv::viz::WCube cube_widget2;
    cv::Ptr<cv::BackgroundSubtractor> pBackSub;

};

#endif //SRC_COMPLX_TRACKER_H

class UGV{
public:
    Eigen::Isometry3d ugv_pose;
    Eigen::Isometry3d ugv_cali_pose;
    //!forword declearation only suite for point * or yinyong &
    complx_tracker DroneTracker;
    bool calib_UGVself = false;
    bool first_initial = true;
    float position_change, rpy_change;
    Eigen::Vector3d rpy_pre, rpy_now;
    float position_threshold = 1;
    float rpy_threshold = 1;
    UGV();
};





























