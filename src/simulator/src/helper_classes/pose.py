#!/usr/bin/python

class Pose:

    # Initializing variables with default value
    def __init__(self, pos_x=0, pos_y=0, theta=0):

        self.x = pos_x
        self.y = pos_y

        self.theta = theta

    def getPosition(self):

        return (self.x, self.y)