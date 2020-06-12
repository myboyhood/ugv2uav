#!/usr/bin/env python
# encoding: utf-8

import rospy
import os

def add_model():
    GAZEBO_MODEL_PATH = "~/.gazebo/models/"
    model_to_add = 'mark_label_2' # string
    os.system("roslaunch ugv2uav uav_model.launch")
    # os.system("rosrun gazebo_ros spawn_model -file " + GAZEBO_MODEL_PATH + model_to_add +"/model.sdf -sdf -model " + model_to_add + "_1 -x 0 -y 0")

def main():
    rospy.init_node('load_model', anonymous=True)
    add_model()


if __name__ == '__main__':
    main()

