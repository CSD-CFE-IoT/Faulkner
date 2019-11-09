#! /usr/bin/env python
from geometry_msgs.msg import Twist
import math

class yaw_fixer:

    def __init__(self, state_vars, params):
        self.state_vars = state_vars
        self.params = params

    def fix(self):
        # print('Fix_yaw: [%f, %f], yaw: %f, desired_yaw: %f, err_yaw: %f' % (position_.x, position_.y, yaw_, desired_yaw, err_yaw))
        tw = Twist()
        if math.fabs(self.state_vars.err_yaw) > self.params.yaw_precision:
            tw.angular.z = self.params.fix_yaw_ang_vel if self.state_vars.err_yaw > 0 else -self.params.fix_yaw_ang_vel

        return tw