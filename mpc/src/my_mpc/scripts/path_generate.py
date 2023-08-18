#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from my_lqr.msg import way_point
from my_lqr.msg import way_points
from math import *
import numpy as np


def straight(init_coord, end_coord, init_angle, count):
    delta_x = (end_coord[0] - init_coord[0]) / (count-1)
    delta_y = (end_coord[1] - init_coord[1]) / (count - 1)
    xr = []
    yr = []
    yawr = []
    deltar = []
    for i in range(count):
        x = init_coord[0] + delta_x * i
        y = init_coord[1] + delta_y * i
        xr.append(x)
        yr.append(y)
        yawr.append(init_angle)
        deltar.append(0)
    return xr, yr, yawr, deltar


def arc(init_coord, end_coord, init_angle, end_angle, count):
    L = 0.26
    temp = (end_coord[0] - init_coord[0])**2 + (end_coord[1] - init_coord[1])**2
    N = sqrt(temp)
    M = sqrt(2*(1 - cos(end_angle-init_angle)))
    R = N/M
    delta_angle = (end_angle-init_angle) / (count-1)
    xr = []
    yr = []
    yawr = []
    deltar = []
    for i in range(count):
        if delta_angle > 0:
            x = init_coord[0] - R * sin(init_angle) + R * sin(init_angle + delta_angle * (i - 1))
            y = init_coord[1] + R * cos(init_angle) - R * cos(init_angle + delta_angle * (i - 1))
            yaw = init_angle + delta_angle * i
            delta = atan(k*L/R)
        else:
            x = init_coord[0] + R * sin(init_angle) - R * sin(init_angle + delta_angle * (i - 1))
            y = init_coord[1] - R * cos(init_angle) + R * cos(init_angle + delta_angle * (i - 1))
            yaw = init_angle + delta_angle * i
            delta = -atan(k*L / R)
        xr.append(x)
        yr.append(y)
        yawr.append(yaw)
        deltar.append(delta)
    return xr, yr, yawr, deltar


def global_planner(event):
    global_path = Path()
    ref_path = way_points()
    ros_time = rospy.Time.now()
    global_path.header.stamp = ros_time
    global_path.header.frame_id = "map"
    n = len(xr)
    for i in range(n):
        x = xr[i]
        y = yr[i]
        yaw = yawr[i]
        delta = deltar[i]
        # 发着给看
        current_point = PoseStamped()
        current_point.pose.position.x = x
        current_point.pose.position.y = y
        current_point.header.frame_id = "map"
        current_point.header.stamp = ros_time
        global_path.poses.append(current_point)
        # 发着给用
        ref_point = way_point()
        ref_point.x = x
        ref_point.y = y
        ref_point.yaw = yaw
        ref_point.delta = delta
        ref_path.way_points.append(ref_point)
    global_path_pub.publish(global_path)
    ref_path_pub.publish(ref_path)
    print("path has planned")


if __name__ == "__main__":
    rospy.init_node("path_generate", anonymous=True)
    global_path_pub = rospy.Publisher('/my_global_planner', Path, queue_size=1)
    ref_path_pub = rospy.Publisher('/ref_path', way_points, queue_size=1)
    T = 1
    timer = rospy.Timer(rospy.Duration(T), global_planner)
    count = 50
    k = 5
    [x1, y1, yaw1, delta1] = straight([0, 0], [20, 0], 0, count)
    [x2, y2, yaw2, delta2] = arc([20, 0], [30, 10], 0, pi / 2, count)
    [x3, y3, yaw3, delta3] = arc([30, 10], [40, 20], pi / 2, 0, count)
    [x4, y4, yaw4, delta4] = arc([40, 20], [40, 40], 0, pi, count)
    [x5, y5, yaw5, delta5] = arc([40, 40], [35, 35], pi, 3 * pi / 2, count)
    [x6, y6, yaw6, delta6] = arc([35, 35], [25, 35], 3 * pi / 2, pi / 2, count)
    [x7, y7, yaw7, delta7] = arc([25, 35], [15, 35], pi / 2, 3 * pi / 2, count)
    [x8, y8, yaw8, delta8] = arc([15, 35], [5, 35], 3 * pi / 2, pi / 2, count)
    [x9, y9, yaw9, delta9] = arc([5, 35], [-15, 35], pi / 2, 3 * pi / 2, count)
    [x10, y10, yaw10, delta10] = straight([-15, 35], [-15, 15], 3 * pi / 2, count)
    [x11, y11, yaw11, delta11] = arc([-15, 15], [0, 0], 3 * pi / 2, 2 * pi, count)
    xr = np.hstack((x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11))/k
    yr = np.hstack((y1, y2, y3, y4, y5, y6, y7, y8, y9, y10, y11))/k
    yawr = np.hstack((yaw1, yaw2, yaw3, yaw4, yaw5, yaw6, yaw7, yaw8, yaw9, yaw10, yaw11))
    deltar = np.hstack((delta1, delta2, delta3, delta4, delta5, delta6, delta7, delta8, delta9, delta10, delta11))
    rospy.spin()
