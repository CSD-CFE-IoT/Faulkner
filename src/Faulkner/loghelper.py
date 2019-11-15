#! /usr/bin/env python

def logheader(controller_name, state_vars):
    return '[%s],[(%f,%f)->(%f,%f)]:' % (
        controller_name,
        state_vars.position.x,
        state_vars.position.y,
        state_vars.goal_pos.x,
        state_vars.goal_pos.y
    )