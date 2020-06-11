#!/usr/bin/env python2
from scipy.optimize import *
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.patches as mpathes
import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

"""global variable"""
ugv1_odom = Odometry()
ugv2_odom = Odometry()
drone_odom = Odometry()
ugv1_reach = Bool()
ugv2_reach = Bool()

x0_0 = 0
x1_0 = 0
x2_0 = 0
x3_0 = 0
p_0 = 0
p_1 = 0
w_1 = 5
w_2 = 1
w_3 = 1

def func(x,sign=1.0):
    """ Objective function """

    return sign * -(
            w_1 * ((x[0]-p_0)**2 + (x[1]-p_1)**2 + (x[2]-p_0)**2 + (x[3]-p_1)**2) +
            w_2 * ((x[0]-x0_0)**2 + (x[1]-x1_0)**2 + (x[2]-x2_0)**2 + (x[3]-x3_0)**2) +
            w_3 * (((x[1]-p_1)/(x[0]-p_0))*((x[3]-p_1)/(x[2]-p_0)) + 1)**2
    )


def func_deriv(x,sign=1.0):
    """ Derivative of objective function """
    dfdx0 = sign * -(w_1*2*(x[0]-p_0) + w_2*2*(x[0]-x0_0) + w_3*(-((x[1]-p_1)/((x[0]-p_0)**2))*((x[3]-p_1)/(x[2]-p_0)))*2*(((x[1]-p_1)/(x[0]-p_0))*((x[3]-p_1)/(x[2]-p_0)) + 1))
    dfdx1 = sign * -(w_1*2*(x[1]-p_1) + w_2*2*(x[1]-x1_0) + w_3*((1/(x[0]-p_0))*(x[3]-p_1)/(x[2]-p_0)) *2* (((x[1]-p_1)/(x[0]-p_0))*((x[3]-p_1)/(x[2]-p_0)) + 1))
    dfdx2 = sign * -(w_1*2*(x[2]-p_0) + w_2*2*(x[2]-x2_0) + w_3*(-((x[1]-p_1)/(x[0]-p_0))*((x[3]-p_1)/((x[2]-p_0)**2)))*2*(((x[1]-p_1)/(x[0]-p_0))*((x[3]-p_1)/(x[2]-p_0)) + 1))
    dfdx3 = sign * -(w_1*2*(x[3]-p_1) + w_2*2*(x[1]-x3_0) + w_3*((1/(x[0]-p_0))*(x[3]-p_1)/(x[2]-p_0)) *2* (((x[1]-p_1)/(x[0]-p_0))*((x[3]-p_1)/(x[2]-p_0)) + 1))
    return np.array([dfdx0, dfdx1, dfdx2, dfdx3])

def opti_ugv_position(x0,x1,x2,x3,p0,p1,w1=5,w2=1,w3=1):
    """declear global"""
    global x0_0
    global x1_0
    global x2_0
    global x3_0
    global p_0
    global p_1
    global w_1
    global w_2
    global w_3
    """initial UGV position"""
    x0_0 = x0
    x1_0 = x1
    x2_0 = x2
    x3_0 = x3
    """initial UAV position"""
    p_0 = p0
    p_1 = p1
    """"weight for each part of loss func"""
    w_1 = w1 # distance between UGV and UAV
    w_2 = w2 # distance between startpoint and endpoint of UGV
    w_3 = w3 # the error to angle 90 of UGVs
    """variable x"""
    cons = (
        {
        'type': 'ineq',
        'fun': lambda x: np.array([-((x[0] - p_0)**2 + (x[1] - p_1)**2) + 25, -((x[2] - p_0)**2 + (x[3] - p_1)**2) + 25,
                                   (x[0] - p_0)**2 + (x[1] - p_1)**2 - 1, (x[2] - p_0)**2 + (x[3] - p_1)**2 - 1]),
        'jar': lambda x: np.array([[-2*(x[0]-p_0), -2*(x[1]-p_1)], [-2*(x[2]-p_0), -2*(x[3]-p_1)], [-2*(x[0]-p_0), -2*(x[1]-p_1)], [-2*(x[2]-p_0), -2*(x[3]-p_1)]])
        })

    bounds = Bounds([-1.0, -3, -1, -3], [5, 3, 5, 3]) # [low_b1, low_b2], [up_b1, up_b2]
    x_startpoint = np.array([x0_0, x1_0, x2_0, x3_0])
    res = minimize(func, x_startpoint, args=(-1.0,), method='SLSQP',
                   jac=func_deriv, constraints=cons,
               options={'disp': True})
    print(res.x)
    return res.x

def ugv1_cb(data):
    global ugv1_odom
    ugv1_odom = data

def ugv2_cb(data):
    global ugv2_odom
    ugv2_odom = data

def drone_cb(data):
    global drone_odom
    drone_odom = data

def reached1_cb(data):
    global ugv1_reach
    ugv1_reach = data

def reached2_cb(data):
    global ugv2_reach
    ugv2_reach = data

def ros_node():
    rospy.init_node('opti_ugv')
    rospy.Subscriber('/ugv1/odom', Odometry, ugv1_cb, queue_size=2)
    rospy.Subscriber('/ugv2/odom', Odometry, ugv2_cb, queue_size=2)
    rospy.Subscriber('/ugv1/reached', Bool, reached1_cb, queue_size=3)
    rospy.Subscriber('/ugv2/reached', Bool, reached2_cb, queue_size=3)
    rospy.Subscriber('/hummingbird/ground_truth/odometry', Odometry, drone_cb, queue_size=10)
    ugv1_tar_pub = rospy.Publisher('/ugv1/target_position', PointStamped, queue_size=1)
    ugv2_tar_pub = rospy.Publisher('/ugv2/target_position', PointStamped, queue_size=1)
    rate = rospy.Rate(10)
    ugv1_tar_msg = PointStamped()
    ugv2_tar_msg = PointStamped()
    """plot initial"""
    plt.ion()
    fig = plt.figure()
    ax = plt.subplot(111)
    traj_region_circle1 = mpathes.Circle((2,0),5,color='g')
    traj_region_rect1 = mpathes.Rectangle((2,-5),20,10,color='g')
    traj_region_circle2 = mpathes.Circle((22,0),5,color='g')
    traj_region_rect2 = mpathes.Rectangle((17,-15),10,15,color='g')
    traj_region_circle3 = mpathes.Circle((22,-15),5,color='g')
    traj_region_rect3 = mpathes.Rectangle((22,-20),10,10,color='g')
    plt.axis([-5, 20, -30, 10])
    plt.axis('equal')
    plt.grid()

    """set true at first"""
    ugv1_reach.data = True
    ugv2_reach.data = True
    ugv1_x = 0
    ugv1_y = 0
    ugv2_x = 0
    ugv2_y = 0
    count = 1
    while not rospy.is_shutdown():
        """firstly rosrun opti_ugv program, then rosrun ugv_autodrive"""
        if ugv1_reach.data and ugv2_reach.data:  # if ugv1 and ugv2 reached current target, calculate next target
            [ugv1_x, ugv1_y, ugv2_x, ugv2_y]=opti_ugv_position(ugv1_odom.pose.pose.position.x, ugv1_odom.pose.pose.position.y,
                              ugv2_odom.pose.pose.position.x, ugv2_odom.pose.pose.position.y,
                              drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y,
                              w1=15, w2=1, w3=1)

        print('x0:',x0_0, "  ", x1_0, "  ", x2_0, "  ", x3_0)
        print('p',p_0, "  ", p_1)
        print('ugv1_xy:', ugv1_x, ugv1_y)
        print('ugv2_xy:',ugv2_x, ugv2_y)
        """plot in real-time"""

        """static trajectory region: "Z"  """
        ax.add_patch(traj_region_circle1)
        ax.add_patch(traj_region_circle2)
        ax.add_patch(traj_region_circle3)
        ax.add_patch(traj_region_rect1)
        ax.add_patch(traj_region_rect2)
        ax.add_patch(traj_region_rect3)

        """dynamiclly change car and plane position"""
        plt.plot(ugv1_odom.pose.pose.position.x, ugv1_odom.pose.pose.position.y, 'x', markersize=4, color='y')
        plt.plot(ugv2_odom.pose.pose.position.x, ugv2_odom.pose.pose.position.y, 'o', markersize=4, color='y')
        plt.plot(drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, '*', markersize=4, color='k')
        drone_xy1 = (drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y) # turple ()
        plane_rect = mpathes.Rectangle((drone_xy1[0]-0.5, drone_xy1[1]-0.5),1,1,0.0,color='g')

        circle_out = mpathes.Circle(drone_xy1, 5,color='b')
        circle_in = mpathes.Circle(drone_xy1, 1,color='r')
        ax.add_patch(circle_out) # ensure small circle on the top of large circle
        ax.add_patch(circle_in)

        ax.add_patch(plane_rect)
        """plot result x"""
        # plt.plot(ugv1_x, ugv1_y, 'x', markersize=8, color='m')
        # plt.plot(ugv2_x, ugv2_y, 'o', markersize=8, color='m')
        count_str = str(count)
        count += 1
        plt.text(-1, 4, count_str, fontsize='medium')
        plt.show()
        plt.pause(0.1)
        # plt.clf() # clear plot before

        ugv1_tar_msg.point.x = ugv1_x
        ugv1_tar_msg.point.y = ugv1_y
        ugv2_tar_msg.point.x = ugv2_x
        ugv2_tar_msg.point.y = ugv2_y

        ugv1_tar_pub.publish(ugv1_tar_msg)
        ugv2_tar_pub.publish(ugv2_tar_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        ros_node()
    except rospy.ROSInterruptException:
        pass

# def func(x, sign=1.0):
#     """ Objective function """
#     return sign * (2 * x[0] * x[1] + 2 * x[0] - x[0] ** 2 - 2 * x[1] ** 2)
#
# def func_deriv(x, sign=1.0):
#     """ Derivative of objective function """
#     dfdx0 = sign * (-2 * x[0] + 2 * x[1] + 2)
#     dfdx1 = sign * (2 * x[0] - 4 * x[1])
#     return np.array([dfdx0, dfdx1])
#
# bounds = Bounds([0,-0.5],[1.0, 2.0])
# cons = ({'type': 'eq',
#          'fun': lambda x: np.array([x[0]**3 - x[1]]),
#          'jac': lambda x: np.array([3.0 * (x[0]**2), -1.0])},
#         {
#             'type': 'ineq',
#             'fun': lambda x: np.array([x[1] - 1]),
#             'jar': lambda x: np.array([0.0, 1.0])
#         })
#
# x0 = np.array([-1.0, 5.0])
# res = minimize(func, x0, args=(-1.0,), method='SLSQP', jac=func_deriv, constraints=cons,
#                options={'disp': True}, bounds=bounds)
# print(res.x)

