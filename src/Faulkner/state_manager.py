#! /usr/bin/env python
from geometry_msgs.msg import Point
import math
import rospy

state_desc_ = ['Obstacle avoidance', 'Fix yaw', 'Straight ahead', 'Done']

class global_state_variables:
    def __init__(self):
        # pose and regions
        self.position = Point()
        self.yaw = 0.0
        self.regions = None

        #goal
        self.goal_pos = self.position

        # derived state
        self.desired_yaw = 0.0
        self.err_yaw = 0.0
        self.err_pos = 0.0

        # action state
        self.action_state = 1 # start by fixing yaw

        self.awaiting_goal = True

class state_manager:
    def __init__(self, state_vars, params, avoidance, history):
        self.state_vars = state_vars
        self.params = params
        self.avoidance = avoidance
        self.history = history

    def ready(self):
        return self.state_vars.regions != None

    def move_to_next_state(self):

        goal = self.state_vars.goal_pos
        pos = self.state_vars.position

        self.state_vars.desired_yaw = math.atan2(goal.y - pos.y, goal.x - pos.x)
        self.state_vars.err_yaw = self.normalize_angle(self.state_vars.desired_yaw - self.state_vars.yaw)
        self.state_vars.err_pos = math.sqrt(pow(goal.y - pos.y, 2) + pow(goal.x - pos.x, 2))

        # if near goal
        if self.state_vars.err_pos <= self.params.dist_precision:
            self.set_action_state(3) # done
            self.state_vars.awaiting_goal = True
            return
            
        # if path is obstructed
        if self.params.avoidance_enabled and self.avoidance.is_obstructed():
            self.set_action_state(0) # avoid obstacle
            return

        # if yaw error below tolerance threshold, straight ahead, else fix yaw
        if math.fabs(self.state_vars.err_yaw) > self.params.yaw_precision and not self.is_oscillating():
            self.set_action_state(1) # fix yaw
        else:
            self.set_action_state(2) # go straight ahead
    
    def is_oscillating(self):
        rospy.loginfo(self.history)
        history_window = self.history.get()
        fix_yaw_count = 0
        last_ang_z = 1
        num_oscillations = 0
        for iteration in history_window[::-1]: #reverse iterator
            if iteration.action_state == 1: # is fix yaw iteration
                fix_yaw_count += 1
                if fix_yaw_count > 1 and iteration.ang_z * last_ang_z < 0:
                    num_oscillations += 1
                last_ang_z = iteration.ang_z

        oscillating = num_oscillations >= self.params.yaw_oscillation_tolerance
        if oscillating:
            rospy.loginfo(
                '-------------------------------------------------------------------------------------Will not fix yaw to avoid further oscillation. Detected %d oscillations-----------------------------------------------------------------------',
                num_oscillations
            )
        return oscillating

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