#! /usr/bin/env python

from geometry_msgs.msg import Twist
import rospy
import loghelper

class avoidance:
    def __init__(self, state_vars, params, history):
        self.state_vars = state_vars
        self.params = params
        self.history = history
    
    #def is_obstructed(self):
    #    return self.front() or self.fleft() or self.frontAndFright() or self.frontAndFleft() or self.frontAndFleftAndFright()
        #return self.front() or self.frontAndFright() or self.frontAndFleft() or self.frontAndFleftAndFright()

    #def is_obstructed(self):
    #    return self.state_vars.regions['front'] <= self.params.avoid_distance or self.state_vars.regions['fleft'] <= self.params.avoid_distance or self.state_vars.regions['fright'] <= self.params.avoid_distance
    def obstacle_near_centre(self, regions):
        return regions['front'] <= self.params.avoid_distance or regions['fleft'] <= self.params.avoid_distance or regions['fright'] <= self.params.avoid_distance
    
    def is_obstructed(self, regions):
        self.regions = regions
        history_window = self.history.getLastNItems(self.params.is_obstructed_window)
        obs = False
        for it in history_window:
            if self.obstacle_near_centre(it.regions):
                obs = True
                break
        return obs or self.obstacle_near_centre(self.regions)

    def front(self):
        return self.regions['front'] <= self.params.avoid_distance and self.regions['fleft'] > self.params.avoid_distance and self.regions['fright'] > self.params.avoid_distance

    def fright(self):
        return self.regions['front'] > self.params.avoid_distance and self.regions['fleft'] > self.params.avoid_distance and self.regions['fright'] <= self.params.avoid_distance

    def fleft(self):
        return self.regions['front'] > self.params.avoid_distance and self.regions['fleft'] <= self.params.avoid_distance and self.regions['fright'] > self.params.avoid_distance

    def frontAndFright(self):
        return self.regions['front'] <= self.params.avoid_distance and self.regions['fleft'] > self.params.avoid_distance and self.regions['fright'] <= self.params.avoid_distance

    def frontAndFleft(self):
        return self.regions['front'] <= self.params.avoid_distance and self.regions['fleft'] <= self.params.avoid_distance and self.regions['fright'] > self.params.avoid_distance

    def frontAndFleftAndFright(self):
        return self.regions['front'] <= self.params.avoid_distance and self.regions['fleft'] <= self.params.avoid_distance and self.regions['fright'] <= self.params.avoid_distance

    def fleftAndFright(self):
        return self.regions['front'] > self.params.avoid_distance and self.regions['fleft'] <= self.params.avoid_distance and self.regions['fright'] <= self.params.avoid_distance

    def avoid(self):
        tw = Twist()
        state_description = "unobstructed"
        if self.fright():
            state_description = 'fright'
            tw.linear.x = self.params.max_lin_vel
            tw.angular.z = 0
        elif self.front():
            state_description = 'front'
            # turn right to follow wall
            tw.linear.x = 0
            tw.angular.z = -self.params.avoidance_ang_vel_magnitude
        elif self.frontAndFright():
            state_description = 'front and fright'
            # turn left
            tw.linear.x = 0
            tw.angular.z = self.params.avoidance_ang_vel_magnitude
        elif self.fleft():
            state_description = 'fleft'
            # follow wall
            tw.linear.x = self.params.max_lin_vel
            tw.angular.z = 0
        elif self.frontAndFleft():
            state_description = 'front and fleft'
            # turn right
            tw.linear.x = 0
            tw.angular.z = -self.params.avoidance_ang_vel_magnitude
        elif self.frontAndFleftAndFright():
            state_description = 'front and fleft and fright'
            # turn right to follow wall
            tw.linear.x = 0
            tw.angular.z = -self.params.avoidance_ang_vel_magnitude

        rospy.loginfo(
            '%s Case %s, applied: %f/%f',
            loghelper.logheader('Avoidance', self.state_vars),
            state_description,
            tw.linear.x,
            tw.angular.z
        )
        return tw