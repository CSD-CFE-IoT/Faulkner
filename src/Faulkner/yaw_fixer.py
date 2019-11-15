#! /usr/bin/env python
from geometry_msgs.msg import Twist
import math
import rospy
import loghelper

class yaw_fixer:

    def __init__(self, state_vars, params):
        self.state_vars = state_vars
        self.params = params

    def fix(self):
        tw = Twist()
        if math.fabs(self.state_vars.err_yaw) > self.params.yaw_precision:
            tw.angular.z = self.params.fix_yaw_ang_vel if self.state_vars.err_yaw > 0 else -self.params.fix_yaw_ang_vel
        rospy.loginfo('%s Applied ang vel %f', loghelper.logheader('Fix yaw', self.state_vars), tw.angular.z)
        return tw