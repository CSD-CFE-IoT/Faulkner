#! /usr/bin/env python

class iteration:
    def __init__(self, action_state, lin_x, ang_z):
        self.action_state = action_state
        self.lin_x = lin_x
        self.ang_z = ang_z

class history:
    def __init__(self,size_max):
        self.max = size_max
        self.data = []
        self.cur = 0

    def append(self, x):
        if len(self.data) < self.max:
            self.data.append(x)
        else:
            self.data[self.cur] = x
            self.cur=(self.cur+1) % self.max

    def get(self):
       return self.data[self.cur:]+self.data[:self.cur]

    def getLastN(self, n):
        if n <= self.cur:
            return self.data[self.cur-n:self.cur]
        return self.data[len(self.data)-(n-self.cur):] + self.data[:self.cur]