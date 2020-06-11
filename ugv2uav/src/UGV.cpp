//
// Created by wzy on 2020/6/10.
//
#include <ugv2uav/UGV.h>
#include "math.h"
UGV::UGV()
{
    internal_mtx_ext.matrix() <<  1206.89,0,960,0,
            0,1206.89,540,0,
            0,0,1,0;

    gazebo2cam_coord.matrix() <<
            0,-1,0,0,
            0,0,-1,0,
            1,0,0,0,
            0,0,0,1;

    //! camera to UGV installation transform matrix
    Eigen::Vector3d euler_angles(0,-0.4,0);
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(euler_angles[0],Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(euler_angles[1],Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(euler_angles[2],Eigen::Vector3d::UnitX());
    installation_Cam2Body = Eigen::Isometry3d::Identity();
    installation_Cam2Body.rotate(rotation_matrix);
    installation_Cam2Body.pretranslate(Eigen::Vector3d(0,0,0.1));//in order to convert to <4,4>matrix to multiply , use .matrix()

}

void UGV::get_worldPoint(double X,double Y,double Z)
{
    worldPoint_ext(0,0) = X;
    worldPoint_ext(1,0) = Y;
    worldPoint_ext(2,0) = Z;
    worldPoint_ext(3,0) = 1.0;
//    std::cout <<  worldPoint_ext.matrix() << std::endl;

}

void UGV::odom2trans()
{
    ugv_Trans = Eigen::Isometry3d::Identity();
    ugv_Trans.rotate(Eigen::Quaterniond(ugv_odom.orientation.w,ugv_odom.orientation.x,ugv_odom.orientation.y,ugv_odom.orientation.z));
    ugv_Trans.pretranslate(Eigen::Vector3d(ugv_odom.position.x,ugv_odom.position.y,ugv_odom.position.z));
//    std::cout << "ugv_Trans" << ugv_Trans.matrix() << std::endl;
    Quat_w=ugv_odom.orientation.w;
    Quat_x=ugv_odom.orientation.x;
    Quat_y=ugv_odom.orientation.y;
    Quat_z=ugv_odom.orientation.z;

}

cv::Point2d UGV::get_uv_from_worldPoint(Eigen::Matrix<double,4, 1> &worldTraj_Point)
{
//    odom2trans();
//    std::cout << "output matrix:" << std::endl
//    << "gazebo2cam_coord" << gazebo2cam_coord.matrix()
//    << "internal_mtx_ext" << internal_mtx_ext.matrix()
//    << "installation:" <<installation_Cam2Body.matrix()
//    << "ugv_Trans.matrix(): " << ugv_Trans.matrix()
//    << "worldPoint_ext: " << worldPoint_ext.matrix() << std::endl;
    pixel_uv_ext = internal_mtx_ext * gazebo2cam_coord * ((installation_Cam2Body.matrix()*ugv_Trans.matrix()).inverse() * worldTraj_Point);
//    std::cout << "pixel uv: " << pixel_uv_ext.matrix() << std::endl;
    if(pixel_uv_ext(2,0) < 0)
    {   //! return error
        pixel_uv_normal.x = -1;
        pixel_uv_normal.y = -1;
        return pixel_uv_normal;
    }

    pixel_uv_normal.x = pixel_uv_ext(0,0)/pixel_uv_ext(2,0);
    pixel_uv_normal.y = pixel_uv_ext(1,0)/pixel_uv_ext(2,0);
    return pixel_uv_normal;
}

cv::Point2d UGV::get_uv_from_sim_Opti(Eigen::Isometry3d &trans, Eigen::Matrix<double,4, 1> &worldTraj_Point)
{   cv::Point2d pixel_uv;
    Eigen::Matrix<double,3, 1> pixel_uv_extend;
    pixel_uv_extend = internal_mtx_ext * gazebo2cam_coord * ((installation_Cam2Body.matrix()*trans.matrix()).inverse() * worldTraj_Point);

    pixel_uv.x = pixel_uv_extend(0,0)/pixel_uv_extend(2,0);
    pixel_uv.y = pixel_uv_extend(1,0)/pixel_uv_extend(2,0);
    return pixel_uv;
}

void UGV::generate_4_points_of_traj(Eigen::Matrix<double,4, 1> &WT_s_long_,Eigen::Matrix<double,4, 1> &WT_e_long_)
{
    WT_s_long = WT_s_long_;
    WT_e_long = WT_e_long_;

    float theta = 0;
    theta = atan2( (WT_e_long(1,0)-WT_s_long(1,0)) , (WT_e_long(0,0)-WT_s_long(0,0)) );
    WT_s_short(0,0) = WT_s_long(0,0) + 0.5*cos(theta+PI/2);
    WT_s_short(1,0) = WT_s_long(1,0) + 0.5*sin(theta+PI/2);
    WT_s_short(2,0) = WT_s_long(2,0);
    WT_s_short(3,0) = WT_s_long(3,0);

    WT_e_short(0,0) = WT_s_long(0,0) + 0.5*cos(theta-PI/2);
    WT_e_short(1,0) = WT_s_long(1,0) + 0.5*sin(theta-PI/2);
    WT_e_short(2,0) = WT_s_long(2,0);
    WT_e_short(3,0) = WT_s_long(3,0);
    std::cout << "WT_s_short:" << WT_s_short(0,0) << WT_s_short(1,0) << std::endl
    << "WT_e_short:" << WT_e_short(0,0) << WT_e_short(1,0) << std::endl;

}

void UGV::get_Score(float &score,Eigen::Matrix<double,4, 1> &WT_s_long_,Eigen::Matrix<double,4, 1> &WT_e_long_)
{
    //!(Quaternion to euler) suppose anticlockwise, it has interval from  0~180,-180~0
    Quat_yaw = atan2(2*(Quat_w*Quat_z+Quat_x*Quat_y),1-2*(Quat_z*Quat_z+Quat_y*Quat_y));
    if(Quat_yaw > yaw_min && Quat_yaw < yaw_max)
    {
        generate_4_points_of_traj(WT_s_long_,WT_e_long_);
        uv_s_long = get_uv_from_worldPoint(WT_s_long);
        uv_e_long = get_uv_from_worldPoint(WT_e_long);
        uv_s_short = get_uv_from_worldPoint(WT_s_short);
        uv_e_short = get_uv_from_worldPoint(WT_e_short);
        std::cout << "uv_s_short: x y " << uv_s_short.x << "    " << uv_s_short.y << std::endl;
        float score_long,score_short;
        score_long = weight_long * ( sqrt((uv_s_long.y-uv_e_long.y)*(uv_s_long.y-uv_e_long.y) + (uv_s_long.x-uv_e_long.x)*(uv_s_long.x-uv_e_long.x))
                                     / sqrt((WT_s_long(0,0)-WT_e_long(0,0))*(WT_s_long(0,0)-WT_e_long(0,0)) + (WT_s_long(1,0)-WT_e_long(1,0))*(WT_s_long(1,0)-WT_e_long(1,0))) );

        score_short = weight_short * ( sqrt((uv_s_short.y-uv_e_short.y)*(uv_s_short.y-uv_e_short.y) + (uv_s_short.x-uv_e_short.x)*(uv_s_short.x-uv_e_short.x))
                                       / sqrt((WT_s_short(0,0)-WT_e_short(0,0))*(WT_s_short(0,0)-WT_e_short(0,0)) + (WT_s_short(1,0)-WT_e_short(1,0))*(WT_s_short(1,0)-WT_e_short(1,0))) );
        score = score_long + score_short;
        std::cout << "score_long: " << score_long << std::endl <<
                  "score_short: " << score_short << std::endl;
    }
    else
    {
        std::cout << "get score error!! not in field of view" << std::endl;
    }

}

void UGV::find_max_Score(float &score,Eigen::Matrix<double,4, 1> &WT_s_long_,Eigen::Matrix<double,4, 1> &WT_e_long_)
{
    yaw_opti_buff.clear();
    generate_4_points_of_traj(WT_s_long_,WT_e_long_);
    for(yaw_opti=yaw_min;yaw_opti<=yaw_max;) {

        rotation_mtx_yaw_opti = Eigen::AngleAxisd(yaw_opti, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

        Trans_mtx_yaw_opti = Eigen::Isometry3d::Identity();
        Trans_mtx_yaw_opti.rotate(rotation_matrix_yaw);
        Trans_mtx_yaw_opti.pretranslate(ugv_Trans.translation().matrix());
        uv_s_long = get_uv_from_sim_Opti(Trans_mtx_yaw_opti,WT_s_long);
        uv_e_long = get_uv_from_sim_Opti(Trans_mtx_yaw_opti,WT_e_long);
        uv_s_short = get_uv_from_sim_Opti(Trans_mtx_yaw_opti,WT_s_short);
        uv_e_short = get_uv_from_sim_Opti(Trans_mtx_yaw_opti,WT_e_short);
        float score_long,score_short;
        score_long = weight_long * ( sqrt((uv_s_long.y-uv_e_long.y)*(uv_s_long.y-uv_e_long.y) + (uv_s_long.x-uv_e_long.x)*(uv_s_long.x-uv_e_long.x))
                                     / sqrt((WT_s_long(0,0)-WT_e_long(0,0))*(WT_s_long(0,0)-WT_e_long(0,0)) + (WT_s_long(1,0)-WT_e_long(1,0))*(WT_s_long(1,0)-WT_e_long(1,0))) );

        score_short = weight_short * ( sqrt((uv_s_short.y-uv_e_short.y)*(uv_s_short.y-uv_e_short.y) + (uv_s_short.x-uv_e_short.x)*(uv_s_short.x-uv_e_short.x))
                                       / sqrt((WT_s_short(0,0)-WT_e_short(0,0))*(WT_s_short(0,0)-WT_e_short(0,0)) + (WT_s_short(1,0)-WT_e_short(1,0))*(WT_s_short(1,0)-WT_e_short(1,0))) );
        score = score_long + score_short;
        yaw_opti_buff.push_back(score);
        yaw_opti += 0.05;
    }
    score_it = std::max_element(std::begin(yaw_opti_buff),std::end(yaw_opti_buff));

    std::cout << "score_max: " << *score_it << "   at position: " << std::distance(std::begin(yaw_opti_buff),score_it) << std::endl;
    yaw_of_score_max = *score_it;

}


void UGV::to_trans_vec_msg()
{
    odom2trans();
    ugv_trans_vec.a11 = ugv_Trans(0,0); ugv_trans_vec.a12 = ugv_Trans(0,1);
    ugv_trans_vec.a13 = ugv_Trans(0,2); ugv_trans_vec.a14 = ugv_Trans(0,3);

    ugv_trans_vec.a21 = ugv_Trans(1,0); ugv_trans_vec.a22 = ugv_Trans(1,1);
    ugv_trans_vec.a23 = ugv_Trans(1,2); ugv_trans_vec.a24 = ugv_Trans(1,3);

    ugv_trans_vec.a31 = ugv_Trans(2,0); ugv_trans_vec.a32 = ugv_Trans(2,1);
    ugv_trans_vec.a33 = ugv_Trans(2,2); ugv_trans_vec.a34 = ugv_Trans(2,3);

    ugv_trans_vec.a41 = ugv_Trans(3,0); ugv_trans_vec.a42 = ugv_Trans(3,1);
    ugv_trans_vec.a43 = ugv_Trans(3,2); ugv_trans_vec.a44 = ugv_Trans(3,3);
}

void UGV::esti_yaw_interval(Eigen::Matrix<double,4, 1> &worldTraj_start_, Eigen::Matrix<double,4, 1> &worldTraj_end_)
{
    odom2trans();
//    //!(Quaternion to euler) suppose anticlockwise, it has interval from  0~180,-180~0
//    Quat_yaw = atan2(2*(Quat_w*Quat_z+Quat_x*Quat_y),1-2*(Quat_z*Quat_z+Quat_y*Quat_y));
//    std::cout << "Quan_yaw: " << Quat_yaw << std::endl;
//    //!(Quaternion to Transform to euler) suppose anticlockwise, it has interval from 0~180, 0~180
//    Trans_yaw = ugv_Trans.rotation().matrix().eulerAngles(2,1,0);
//    std::cout << "yaw pitch roll:  " << Trans_yaw.transpose() << std::endl;
//    //!(Quaternion to euler) suppose anticlockwise,
//    Eigen_Quat_yaw = Eigen::Quaterniond(ugv_odom.orientation.w,ugv_odom.orientation.x,ugv_odom.orientation.y,ugv_odom.orientation.z).matrix().eulerAngles(2,1,0)[0];
//    std::cout << "Eigen_Quat_yaw:  " << Eigen_Quat_yaw << std::endl;

//    if (Quat_yaw >= 0)
//        yaw2PI = Quat_yaw;
//    if (Quat_yaw < 0)
//        yaw2PI = 2*PI + Quat_yaw;

//    std::cout << "yaw2PI:  " << yaw2PI << std::endl;


    //! 将旋转矩阵转换成欧拉角是有点棘手的。该解决方案在大多数情况下不是唯一的
    //! Eigen always constrain yaw in 0~3.14,
    //! for UGV model, pitch and roll is 0,when yaw is between 0 ~ PI,
    //! pitch and roll is -3.14 and 3.14 respectively,when yaw is between -PI ~ 0,
    //! so according pitch and roll symbol , I can adapt yaw in -PI ~ PI

    //! pixel_traj_normal_start(2,0) > 0 , object is at front of camera
    yaw_GoodRegion.clear();
    float yaw_adapted;//I can adapt yaw into -PI ~ PI
    for(Quat_yaw=-PI;Quat_yaw<=PI;)
    {
        yaw_adapted = Quat_yaw;
        rotation_matrix_yaw = Eigen::AngleAxisd(yaw_adapted, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

        Trans_Matrix_yaw = Eigen::Isometry3d::Identity();
        Trans_Matrix_yaw.rotate(rotation_matrix_yaw);
        Trans_Matrix_yaw.pretranslate(ugv_Trans.translation().matrix());//in order to convert to <4,4>matrix to multiply , use .matrix()
//        std::cout << "Quat_yaw: " << Quat_yaw << std::endl;
//        std::cout << "yaw_adapted: " << yaw_adapted << std::endl;
//        std::cout << "test yaw: " << Trans_Matrix_yaw.rotation().matrix().eulerAngles(2,1,0)[0] << std::endl;
        Quat_yaw+=0.05;
        // get pixel uv normal
        pixel_traj_normal_start = internal_mtx_ext * gazebo2cam_coord * ((installation_Cam2Body.matrix()*Trans_Matrix_yaw.matrix()).inverse() * worldTraj_start_);
        //! if pixel_traj_normal_start(2,0) > 0, object is at front of camera,
        //! if pixel_traj_normal_start(2,0) < 0, object is at back of camera, we should drop it
//        std::cout << "pixel_traj_normal_start(2,0): " << pixel_traj_normal_start(2,0) << std::endl;
        pixel_traj_normal_start(0,0)/=pixel_traj_normal_start(2,0);
        pixel_traj_normal_start(1,0)/=pixel_traj_normal_start(2,0);
        pixel_traj_normal_end = internal_mtx_ext * gazebo2cam_coord * ((installation_Cam2Body.matrix()*Trans_Matrix_yaw.matrix()).inverse() * worldTraj_end_);
        pixel_traj_normal_end(0,0)/=pixel_traj_normal_end(2,0);
        pixel_traj_normal_end(1,0)/=pixel_traj_normal_end(2,0);

        //! if 200< u <1700,then record the yaw angle, object is at front of camera
        if(pixel_traj_normal_start(0,0)>200 && pixel_traj_normal_start(0,0)<1700 &&
                pixel_traj_normal_end(0,0)>200 && pixel_traj_normal_end(0,0)<1700 &&
                pixel_traj_normal_start(2,0) > 0 && pixel_traj_normal_end(2,0) > 0)
        {
            yaw_GoodRegion.push_back(Quat_yaw);
//            std::cout << "Good yaw!" << std::endl;
        }

    }
    yaw_max_it = std::max_element(std::begin(yaw_GoodRegion),std::end(yaw_GoodRegion));
    yaw_min_it = std::min_element(std::begin(yaw_GoodRegion),std::end(yaw_GoodRegion));
    std::cout << "yaw_max: " << *yaw_max_it << "   at position: " << std::distance(std::begin(yaw_GoodRegion),yaw_max_it) << std::endl;
    std::cout << "yaw_min: " << *yaw_min_it << "   at position: " << std::distance(std::begin(yaw_GoodRegion),yaw_min_it) << std::endl;
    yaw_max = *yaw_max_it;
    yaw_min = *yaw_min_it;


}


//UGV::~UGV()
//{}