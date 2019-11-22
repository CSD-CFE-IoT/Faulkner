#!/usr/bin/python

import unittest
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from Faulkner import avoidance

# Turtlebot3 Burger Angle Increment (rad)
BURGER_ANGLE_INCREMENT = 0.0174533

class TestAvoidanceMethods(unittest.TestCase):

    def test_GetRegions_NoRanges(self):
        a = avoidance()
        msg = LaserScan()

        # Set ranges to zero
        msg.angle_increment = BURGER_ANGLE_INCREMENT
        msg.ranges = [0] * 360

        # Get detected regions
        detectedRegion = a.GetRegions(msg)

        # Check resulting region
        expectedRegion = {
            'right' : 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0
        }

        self.assertEqual(detectedRegion, expectedRegion)

    def test_GetRegions_MaxRanges(self):
        a = avoidance()
        msg = LaserScan()

        # Set ranges to 100 meters
        msg.angle_increment = BURGER_ANGLE_INCREMENT
        msg.ranges = [100] * 360

        # Get detected regions
        detectedRegion = a.GetRegions(msg)

        # Check resulting region
        expectedRegion = {
            'right' : 10,
            'fright': 10,
            'front': 10,
            'fleft': 10,
            'left': 10
        }

        self.assertEqual(detectedRegion, expectedRegion)

if __name__ == '__main__':
    unittest.main()