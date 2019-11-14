#! /usr/bin/env python

import rospy

class parameters:
    def __init__(
        self,
        yaw_precision,
        course_correct_yaw_precision,
        dist_precision,
        scan_sector_size,
        max_lin_vel,
        fix_yaw_ang_vel,
        course_correct_ang_vel,
        avoidance_ang_vel_magnitude,
        avoid_distance
    ):
        self.yaw_precision = yaw_precision
        self.course_correct_yaw_precision = course_correct_yaw_precision
        self.dist_precision = dist_precision
        self.scan_sector_size = scan_sector_size
        self.max_lin_vel = max_lin_vel
        self.fix_yaw_ang_vel = fix_yaw_ang_vel
        self.course_correct_ang_vel = course_correct_ang_vel
        self.avoidance_ang_vel_magnitude = avoidance_ang_vel_magnitude
        self.avoid_distance = avoid_distance