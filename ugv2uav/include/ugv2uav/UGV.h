//
// Created by wzy on 2020/6/10.
//

#ifndef UGV2UAV_UGV_H
#define UGV2UAV_UGV_H

#endif //UGV2UAV_UGV_H
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ugv2uav/custom_trans.h"
#include <opencv2/core.hpp>
#include "cstdio"
#include "iostream"
#include <GL/glut.h>

#define PI (3.1415926535897932346f)

class UGV
{
public:
    //! odom
    geometry_msgs::Pose ugv_odom;
    Eigen::Isometry3d ugv_Trans;
    Eigen::Isometry3d installation_Cam2Body;

    //! esti_yaw_interval
    float Quat_w,Quat_x,Quat_y,Quat_z;
    float Quat_yaw;
    float yaw,yaw2PI;
    std::vector<float>::iterator yaw_max_it, yaw_min_it;
    float yaw_max, yaw_min;
    std::vector<float> yaw_GoodRegion;
    Eigen::Vector3d Trans_yaw;
    float Eigen_Quat_yaw;
    Eigen::Matrix3d rotation_matrix_yaw;
    Eigen::Isometry3d Trans_Matrix_yaw;

    //!camera project model
    ugv2uav::custom_trans ugv_trans_vec;
    Eigen::Matrix<double,3, 4> internal_mtx_ext;
    Eigen::Matrix<double,4, 4> gazebo2cam_coord;
    Eigen::Matrix<double,4, 1> worldPoint_ext;
    Eigen::Matrix<double,3, 1> pixel_uv_ext;
    cv::Point2d pixel_uv_normal;
    Eigen::Matrix<double,3, 1> pixel_traj_normal_start;
    Eigen::Matrix<double,3, 1> pixel_traj_normal_end;

    //!world point and uv point to calculate score
    Eigen::Matrix<double,4, 1> WT_s_long,WT_e_long,WT_s_short,WT_e_short;
    cv::Point2d uv_s_long,uv_e_long,uv_s_short,uv_e_short;
    float weight_long_simple,weight_short_simple;
    float weight_long_func(float curr_score,float min_score,float max_score );
    float weight_short_func(float curr_score,float min_score,float max_score);

    //! find optimal yaw in FOV
    Eigen::Matrix3d rotation_mtx_yaw_opti;
    Eigen::Isometry3d Trans_mtx_yaw_opti;
    float yaw_opti;
    std::vector<float> yaw_opti_buff;

    std::vector<float>::iterator score_it;
    std::vector<float> score_buff;

    std::pair<float,float> yaw_score;
    std::vector<std::pair<float,float>> scale_yaw_score_buff;

    std::vector<float> score_long_buff,score_short_buff;
    std::vector<float>::iterator s_long_max_it, s_short_max_it,s_long_min_it, s_short_min_it;
    std::pair<float,float> forward_back_score(float &yaw_min, float &yaw_max,Eigen::Matrix<double,4, 1> &WT_s_long_,Eigen::Matrix<double,4, 1> &WT_e_long_);
    float score_max;
    float yaw_of_score_max;
    std::vector<float> score_max_buff;
    std::vector<float> yaw_of_score_max_buff;
    std::vector<float>::iterator score_max_it;
    std::vector<float>::iterator yaw_of_score_max_it;



//! function
    UGV();

    void get_worldPoint(double X,double Y,double Z);
    cv::Point2d get_uv_from_worldPoint(Eigen::Matrix<double,4, 1> &worldTraj_Point);
    void generate_4_points_of_traj(Eigen::Matrix<double,4, 1> &WT_s_long_,Eigen::Matrix<double,4, 1> &WT_e_long_);
    void odom2trans();
    void to_trans_vec_msg();
    void esti_yaw_interval(Eigen::Matrix<double,4, 1> &worldTraj_start_, Eigen::Matrix<double,4, 1> &worldTraj_end_);
    void get_Score(float &score,Eigen::Matrix<double,4, 1> &WT_s_long_,Eigen::Matrix<double,4, 1> &WT_e_long_);
    void find_max_Score(float &score,Eigen::Matrix<double,4, 1> &WT_s_long_,Eigen::Matrix<double,4, 1> &WT_e_long_);
    float score_fun(float &yaw_fun,Eigen::Matrix<double,4, 1> &WT_s_long_,Eigen::Matrix<double,4, 1> &WT_e_long_);
    cv::Point2d get_uv_from_sim_Opti(Eigen::Isometry3d &trans, Eigen::Matrix<double,4, 1> &worldTraj_Point);
//    ~UGV();
};