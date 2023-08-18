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
            delta = atan(L/R)
        else:
            x = init_coord[0] + R * sin(init_angle) - R * sin(init_angle + delta_angle * (i - 1))
            y = init_coord[1] - R * cos(init_angle) + R * cos(init_angle + delta_angle * (i - 1))
            yaw = init_angle + delta_angle * i
            delta = -atan(L / R)
        xr.append(x)
        yr.append(y)
        yawr.append(yaw)
        deltar.append(delta)
        print("delta", delta)
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
    [x1, y1, yaw1, delta1] =arc([0, 0], [0, 4], 0, pi, count)
    [x2, y2, yaw2, delta2] = arc([0, 4], [0, 0], pi, 2*pi, count)
    xr = np.hstack((x1, x2))
    yr = np.hstack((y1, y2))
    yawr = np.hstack((yaw1, yaw2))
    deltar = np.hstack((delta1, delta2))
    rospy.spin()
