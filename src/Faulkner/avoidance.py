#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

SECTOR_RADS = 0.628319 # 36 degrees
BURGER_MAX_LIN_VEL = 0.1
ANG_VEL_MAGNITUDE = 0.3
AVOID_DISTANCE = 0.5

class avoidance:
    def __init__(self):
        self.linear_x = 0
        self.angular_z = 0
        self.state_description = ''
        self.regions = {}
    
    def GetRegions(self, msg):
        s = int(round(SECTOR_RADS/msg.angle_increment))
        regions = {
            # FAULKNERFIX: make these hardcoded numbers configurable
            'right':  min(min(msg.ranges[(15*s)/2:(17*s)/2]), 10),
            'fright':   min(min(msg.ranges[(17*s)/2:(19*s)/2]), 10),
            'front':  min(min(msg.ranges[0:s/2]+msg.ranges[(19*s)/2:]), 10),
            'fleft': min(min(msg.ranges[s/2:(3*s)/2]), 10),
            'left':  min(min(msg.ranges[(3*s)/2:(5*s)/2]), 10),
        }

        return regions
        
    def UpdateAction(self, msg):
        self.linear_x = 0
        self.angular_z = 0
        self.state_description = ''
        self.regions = {}

        # Update the regions member
        self.regions = self.GetRegions(msg)

        # Select action based on regions
        if self.regions['front'] > AVOID_DISTANCE and self.regions['fleft'] > AVOID_DISTANCE and self.regions['fright'] > AVOID_DISTANCE:
            self.state_description = 'case 1 - nothing'
            self.linear_x = BURGER_MAX_LIN_VEL
            self.angular_z = 0
        elif self.regions['front'] < AVOID_DISTANCE and self.regions['fleft'] > AVOID_DISTANCE and self.regions['fright'] > AVOID_DISTANCE:
            self.state_description = 'case 2 - front'
            self.linear_x = 0
            self.angular_z = ANG_VEL_MAGNITUDE
        elif self.regions['front'] > AVOID_DISTANCE and self.regions['fleft'] > AVOID_DISTANCE and self.regions['fright'] < AVOID_DISTANCE:
            self.state_description = 'case 3 - fright'
            self.linear_x = 0
            self.angular_z = ANG_VEL_MAGNITUDE
        elif self.regions['front'] > AVOID_DISTANCE and self.regions['fleft'] < AVOID_DISTANCE and self.regions['fright'] > AVOID_DISTANCE:
            self.state_description = 'case 4 - fleft'
            self.linear_x = 0
            self.angular_z = -ANG_VEL_MAGNITUDE
        elif self.regions['front'] < AVOID_DISTANCE and self.regions['fleft'] > AVOID_DISTANCE and self.regions['fright'] < AVOID_DISTANCE:
            self.state_description = 'case 5 - front and fright'
            self.linear_x = 0
            self.angular_z = ANG_VEL_MAGNITUDE
        elif self.regions['front'] < AVOID_DISTANCE and self.regions['fleft'] < AVOID_DISTANCE and self.regions['fright'] > AVOID_DISTANCE:
            self.state_description = 'case 6 - front and fleft'
            self.linear_x = 0
            self.angular_z = -ANG_VEL_MAGNITUDE
        elif self.regions['front'] < AVOID_DISTANCE and self.regions['fleft'] < AVOID_DISTANCE and self.regions['fright'] < AVOID_DISTANCE:
            self.state_description = 'case 7 - front and fleft and fright'
            self.linear_x = 0
            self.angular_z = ANG_VEL_MAGNITUDE
        elif self.regions['front'] > AVOID_DISTANCE and self.regions['fleft'] < AVOID_DISTANCE and self.regions['fright'] < AVOID_DISTANCE:
            self.state_description = 'case 8 - fleft and fright'
            self.linear_x = BURGER_MAX_LIN_VEL/2.0
            self.angular_z = 0
        else:
            self.state_description = 'unknown case'