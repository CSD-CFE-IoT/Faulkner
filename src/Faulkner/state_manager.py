#! /usr/bin/env python
from geometry_msgs.msg import Point
import math
import rospy

state_desc_ = ['Obstacle avoidance', 'Fix yaw', 'Straight ahead', 'Done']

class global_state_variables:
    def __init__(self, goal_pos):
        # pose and regions
        self.position = Point()
        self.yaw = 0.0
        self.regions = None

        #goal
        self.goal_pos = goal_pos

        # derived state
        self.desired_yaw = 0.0
        self.err_yaw = 0.0
        self.err_pos = 0.0

        # action state
        self.action_state = 1 # start by fixing yaw

class state_manager:
    def __init__(self, state_vars, params, avoidance):
        self.state_vars = state_vars
        self.params = params
        self.avoidance = avoidance

    def ready(self):
        return self.state_vars.regions != None

    def move_to_next_state(self):
        self.state_vars.desired_yaw = math.atan2(self.state_vars.goal_pos.y - self.state_vars.position.y, self.state_vars.goal_pos.x - self.state_vars.position.x)
        self.state_vars.err_yaw = self.normalize_angle(self.state_vars.desired_yaw - self.state_vars.yaw)
        self.state_vars.err_pos = math.sqrt(pow(self.state_vars.goal_pos.y - self.state_vars.position.y, 2) + pow(self.state_vars.goal_pos.x - self.state_vars.position.x, 2))

        # if near goal
        if self.state_vars.err_pos <= self.params.dist_precision:
            self.set_action_state(3) # done
            return
            
        # if path is obstructed
        if self.avoidance.is_obstructed():
            self.set_action_state(0) # avoid obstacle
            return

        # if yaw error below tolerance threshold, straight ahead, else fix yaw
        if math.fabs(self.state_vars.err_yaw) <= self.params.yaw_precision:
            self.set_action_state(2) # go straight ahead
        else:
            self.set_action_state(1) # fix yaw
    
    def normalize_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def set_action_state(self, new_action_state):
        if new_action_state != self.state_vars.action_state:
            rospy.loginfo(
                'Action state transition from %s to %s',
                state_desc_[self.state_vars.action_state],
                state_desc_[new_action_state]
            )
            self.state_vars.action_state = new_action_state

    def action_state(self):
        return self.state_vars.action_state