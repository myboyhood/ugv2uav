#include <complx_tracker.h>

//
// Created by wzy on 4/30/20.
//
UGV::UGV() {
    //! initial pose of UGV in Gazebo coordinate
    ugv_pose.matrix() << 1, 0, 0, 0,
                         0, 1, 0, 0,
                         0, 0, 1, 0,
                         0, 0, 0, 1;
}
