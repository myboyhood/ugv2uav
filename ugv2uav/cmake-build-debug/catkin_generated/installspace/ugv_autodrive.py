#!/usr/bin/env python2
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

"""global variable"""
# ugv1_odom = Odometry()
# ugv2_odom = Odometry()
# target1 = PointStamped()
# target2 = PointStamped()
# angle_iscorrect = False
# twist = Twist()

def ugv1_cb(data):
    ugv1.ugv_odom = data

def target1_cb(data):
    ugv1.target = data

def ugv2_cb(data):
    ugv2.ugv_odom = data

def target2_cb(data):
    ugv2.target = data


class UGV:
    def __init__(self):
        self.ugv_odom = Odometry()
        self.target = PointStamped()
        self.twist = Twist()
        self.ugv_reached = Bool()
        self.ugv_delta_x = 1
        self.ugv_delta_y = 1
        self.curr_yaw = 0
        self.curr_yaw360 = 0
        self.target360 = 0
        self.exe_angle_err = 0
        self.exe_adjust = False

    # def ugv_cb(self,data):
    #     UGV.ugv_odom = data
    #
    # def target_cb(self,data):
    #     UGV.target = data

    def quan2euler(self,w,x,y,z):
        # r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        # p = math.asin(2*(w*y-z*x))
        self.curr_yaw = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))

    def target_convert360(self,ugv_delta_x,ugv_delta_y):
        if ugv_delta_y >= 0:
            self.target360 = math.atan2(ugv_delta_y, ugv_delta_x)*180/math.pi
        if ugv_delta_y < 0:
            self.target360 = 360.0 + math.atan2(ugv_delta_y, ugv_delta_x)*180/math.pi

    def curr_yaw_convert360(self,yaw):
        if yaw >= 0:
            self.curr_yaw360 = yaw*180/math.pi
        if yaw < 0:
            self.curr_yaw360 = 360 + yaw*180/math.pi

    def decide_Angle_Linear(self,angle_err,linear_err, angleOrient, linearOrient):
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        print('angle_err:', angle_err)
        print('linear err:', linear_err)
        angle_adjust = 5
        if angle_err > angle_adjust:
            self.exe_adjust = True
        if angle_err > 2 and self.exe_adjust: # firstly correct angle
            self.twist.linear.x = 0
            self.twist.angular.z = angleOrient * 0.15
        if angle_err <= 2:
            self.exe_adjust = False
        if linear_err > 0.1 and not self.exe_adjust: # secondly correct translation
            self.twist.linear.x = linearOrient * 0.15
            self.twist.angular.z = 0.0
        if linear_err <= 0.1 and angle_err <= 3:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        print('ugv_twist:', self.twist.linear.x,self.twist.angular.z)


def ugv1_main():
    ugv1.ugv_delta_x = ugv1.target.point.x - ugv1.ugv_odom.pose.pose.position.x
    ugv1.ugv_delta_y = ugv1.target.point.y - ugv1.ugv_odom.pose.pose.position.y
    # print('delta_xy:', ugv_delta_x,ugv_delta_y)
    ugv1.target_convert360(ugv1.ugv_delta_x, ugv1.ugv_delta_y)

    w = ugv1.ugv_odom.pose.pose.orientation.w
    x = ugv1.ugv_odom.pose.pose.orientation.x
    y = ugv1.ugv_odom.pose.pose.orientation.y
    z = ugv1.ugv_odom.pose.pose.orientation.z
    ugv1.quan2euler(w, x, y, z)
    ugv1.curr_yaw_convert360(ugv1.curr_yaw)
    trans_err = math.sqrt(ugv1.ugv_delta_x**2 + ugv1.ugv_delta_y**2)
    print('ugv1_target360,curr360',ugv1.target360,ugv1.curr_yaw360)
    """first rotate car to correct angle"""
    """second translate car to correct position"""
    """
    set curr_yaw = 0, let target always bigger than curr_yaw
    make sure angle_err is between 0 ~ 360 ,then class 360 into four parts
    to decide angle orientation and linear orientation 
    """
    if ugv1.target360 >= ugv1.curr_yaw360:
        init_angle_err = ugv1.target360 - ugv1.curr_yaw360
        ugv1.target360 = init_angle_err
        ugv1.curr_yaw360 = 0.0
    else:
        init_angle_err = 360 + ugv1.target360 - ugv1.curr_yaw360
        ugv1.target360 = init_angle_err
        ugv1.curr_yaw360 = 0.0

    print('ugv1_initAngleErr:', init_angle_err)
    """four conditions"""
    if init_angle_err <= 90:
        ugv1.exe_angle_err = ugv1.target360 - ugv1.curr_yaw360
        ugv1.decide_Angle_Linear(ugv1.exe_angle_err,trans_err,angleOrient=1,linearOrient=1)
    elif init_angle_err <=180:
        ugv1.exe_angle_err = 180 - (ugv1.target360 - ugv1.curr_yaw360)
        ugv1.decide_Angle_Linear(ugv1.exe_angle_err,trans_err,angleOrient=-1,linearOrient=-1)
    elif init_angle_err <=270:
        ugv1.exe_angle_err = ugv1.target360 - ugv1.curr_yaw360 -180
        ugv1.decide_Angle_Linear(ugv1.exe_angle_err,trans_err,angleOrient=1,linearOrient=-1)
    else:
        ugv1.exe_angle_err = 360 - ugv1.target360 - ugv1.curr_yaw360
        ugv1.decide_Angle_Linear(ugv1.exe_angle_err,trans_err,angleOrient=-1,linearOrient=1)

    if ugv1.twist.linear.x or ugv1.twist.angular.z:  # is twist angle and linear zero?
        ugv1.ugv_reached = False
    else:
        ugv1.ugv_reached = True



def ugv2_main():
    ugv2.ugv_delta_x = ugv2.target.point.x - ugv2.ugv_odom.pose.pose.position.x
    ugv2.ugv_delta_y = ugv2.target.point.y - ugv2.ugv_odom.pose.pose.position.y
    # print('delta_xy:', ugv_delta_x,ugv_delta_y)
    ugv2.target_convert360(ugv2.ugv_delta_x, ugv2.ugv_delta_y)

    w = ugv2.ugv_odom.pose.pose.orientation.w
    x = ugv2.ugv_odom.pose.pose.orientation.x
    y = ugv2.ugv_odom.pose.pose.orientation.y
    z = ugv2.ugv_odom.pose.pose.orientation.z
    ugv2.quan2euler(w, x, y, z)
    ugv2.curr_yaw_convert360(ugv2.curr_yaw)
    trans_err = math.sqrt(ugv2.ugv_delta_x**2 + ugv2.ugv_delta_y**2)
    """first rotate car to correct angle"""
    """second translate car to correct position"""
    """
    set curr_yaw = 0, let target always bigger than curr_yaw
    make sure angle_err is between 0 ~ 360 ,then class 360 into four parts
    to decide angle orientation and linear orientation 
    """
    if ugv2.target360 >= ugv2.curr_yaw360:
        init_angle_err = ugv2.target360 - ugv2.curr_yaw360
        ugv2.target360 = init_angle_err
        ugv2.curr_yaw360 = 0.0
    else:
        init_angle_err = 360 + ugv2.target360 - ugv2.curr_yaw360
        ugv2.target360 = init_angle_err
        ugv2.curr_yaw360 = 0.0


    """four conditions"""
    if init_angle_err <= 90:
        ugv2.exe_angle_err = ugv2.target360 - ugv2.curr_yaw360
        ugv2.decide_Angle_Linear(ugv2.exe_angle_err,trans_err,angleOrient=1,linearOrient=1)
    elif init_angle_err <=180:
        ugv2.exe_angle_err = 180 - (ugv2.target360 - ugv2.curr_yaw360)
        ugv2.decide_Angle_Linear(ugv2.exe_angle_err,trans_err,angleOrient=-1,linearOrient=-1)
    elif init_angle_err <=270:
        ugv2.exe_angle_err = ugv2.target360 - ugv2.curr_yaw360 -180
        ugv2.decide_Angle_Linear(ugv2.exe_angle_err,trans_err,angleOrient=1,linearOrient=-1)
    else:
        ugv2.exe_angle_err = 360 - ugv2.target360 - ugv2.curr_yaw360
        ugv2.decide_Angle_Linear(ugv2.exe_angle_err,trans_err,angleOrient=-1,linearOrient=1)

    if ugv2.twist.linear.x or ugv2.twist.angular.z:  # is twist angle and linear zero?
        ugv2.ugv_reached = False
    else:
        ugv2.ugv_reached = True

# def ugv1_cb(data):
#     global ugv1_odom
#     ugv1_odom = data

# def ugv2_cb(data):
#     global ugv2_odom
#     ugv2_odom = data
#
# def target1_cb(data):
#     global target1
#     target1 = data
#
# def target2_cb(data):
#     global target2
#     target2 = data
#
# def quan2euler(w,x,y,z):
#     # r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
#     # p = math.asin(2*(w*y-z*x))
#     y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
#     return y
#
# def target_convert360(ugv_delta_x,ugv_delta_y):
#     if ugv_delta_y >= 0:
#         return math.atan2(ugv_delta_y, ugv_delta_x)*180/math.pi
#     if ugv_delta_y < 0:
#         return 360.0 + math.atan2(ugv_delta_y, ugv_delta_x)*180/math.pi
#
# def odom_convert360(yaw):
#     if yaw >= 0:
#         return yaw*180/math.pi
#     if yaw < 0:
#         return 360 + yaw*180/math.pi
#
# def decide_anglerate(angle_err):
#     global angle_iscorrect
#     global twist
#     twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
#     twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
#     if angle_err > 0.1:
#         twist.angular.z = 0.05
#     elif angle_err < -0.1:
#         twist.angular.z = -0.05
#     else:
#         twist.angular.z = 0.0
#         angle_iscorrect = True
#
#     return twist
#


if __name__=="__main__":
    ugv1 = UGV()
    ugv2 = UGV()

    rospy.init_node('ugv_autodrive')
    rospy.Subscriber('/ugv1/target_position', PointStamped, target1_cb, queue_size=3)
    rospy.Subscriber('/ugv2/target_position', PointStamped, target2_cb, queue_size=3)
    rospy.Subscriber('/ugv1/odom', Odometry, ugv1_cb, queue_size=1)
    rospy.Subscriber('/ugv2/odom', Odometry, ugv2_cb, queue_size=1)
    pub1 = rospy.Publisher('ugv1/cmd_vel', Twist, queue_size=1)
    pub2 = rospy.Publisher('ugv2/cmd_vel', Twist, queue_size=1)
    pub1_reach = rospy.Publisher('ugv1/reached', Bool, queue_size=1)
    pub2_reach = rospy.Publisher('ugv2/reached', Bool, queue_size=1)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        ugv1_main()
        ugv2_main()
        print('ugv1_deltaxy:', ugv1.ugv_delta_x,ugv1.ugv_delta_y)
        print('ugv1_exe_angleError:',ugv1.exe_angle_err)
        print('ugv1_twist:',ugv1.twist.linear.x,ugv1.twist.angular.z)

        print('ugv2_deltaxy:', ugv2.ugv_delta_x,ugv2.ugv_delta_y)
        print('ugv2_exe_angleError:',ugv2.exe_angle_err)
        print('ugv2_twist:',ugv2.twist.linear.x,ugv2.twist.angular.z)

        pub1_reach.publish(ugv1.ugv_reached)
        pub2_reach.publish(ugv2.ugv_reached)
        pub1.publish(ugv1.twist)
        pub2.publish(ugv2.twist)
        rate.sleep()


        # ugv1_delta_x = target1.point.x - ugv1_odom.pose.pose.position.x
        # ugv1_delta_y = target1.point.y - ugv1_odom.pose.pose.position.y
        # ugv2_delta_x = target2.point.x - ugv2_odom.pose.pose.position.x
        # ugv2_delta_y = target2.point.y - ugv2_odom.pose.pose.position.y
        #
        # taregt1_360 = target_convert360(ugv1_delta_x,ugv1_delta_y)
        #
        # w1 = ugv1_odom.pose.pose.orientation.w
        # x1 = ugv1_odom.pose.pose.orientation.x
        # y1 = ugv1_odom.pose.pose.orientation.y
        # z1 = ugv1_odom.pose.pose.orientation.z
        # w2 = ugv2_odom.pose.pose.orientation.w
        # x2 = ugv2_odom.pose.pose.orientation.x
        # y2 = ugv2_odom.pose.pose.orientation.y
        # z2 = ugv2_odom.pose.pose.orientation.z
        # yaw1 = quan2euler(w1, x1, y1, z1)
        # yaw2 = quan2euler(w2, x2, y2, z2)
        #
        # """first rotate car to correct angle"""
        # angle_iscorrect = False
        # """error angle"""
        # angle_1_err = math.atan2(ugv1_delta_y, ugv1_delta_x) - p1
        # angle_2_err = math.atan2(ugv2_delta_y, ugv2_delta_x) - p2
        # if angle_1_err < math.pi/2 and angle_1_err > -math.pi/2:
        #     orientation = 1  # forward steer
        # else:
        #     angle_1_err = angle_1_err - math.pi
        #     orientation = -1  # backward steer
        #
        #
        #
        # twist1 = decide_anglerate(angle_1_err)
        # twist2 = decide_anglerate(angle_2_err)
        #
        # if angle_iscorrect:
        #     L1 = math.sqrt(ugv1_delta_x**2 + ugv1_delta_y**2)
        #     L2 = math.sqrt(ugv2_delta_x**2 + ugv2_delta_y**2)

