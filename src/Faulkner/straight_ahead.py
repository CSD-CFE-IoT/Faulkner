#! /usr/bin/env python

from geometry_msgs.msg import Twist
import math
import rospy
import loghelper

class straight_ahead:
    
    def __init__(self, state_vars, params):
        self.state_vars = state_vars
        self.params = params

    def go(self):
        #print('Go_straight_ahead: [%f, %f], yaw: %f, desired_yaw: %f' % (position_.x, position_.y, yaw_, desired_yaw))
        if self.state_vars.err_pos > self.params.dist_precision:
            tw = Twist()
            tw.linear.x = self.params.max_lin_vel
            if math.fabs(self.state_vars.err_yaw) > self.params.course_correct_yaw_precision:
                tw.angular.z = self.params.course_correct_ang_vel if self.state_vars.err_yaw > 0 else -self.params.course_correct_ang_vel

            rospy.loginfo('%s Applied ang vel %f', loghelper.logheader('Straight ahead', self.state_vars), tw.angular.z)
            return tw