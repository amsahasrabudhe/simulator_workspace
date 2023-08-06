#!/usr/bin/python3

class Pose:

    # Initializing variables with default value
    def __init__(self, pos_x=0, pos_y=0, heading=0):

        self.x = pos_x
        self.y = pos_y

        self.heading = heading

    def __eq__(self, other):

        if self.x == other.x and self.y == other.y:
            return True
        return False

    def __add__(self, other):

        return Pose(self.x+other.x, self.y+other.y, self.heading+other.heading)

    def __sub__(self, other):

        return Pose(self.x-other.x, self.y-other.y, self.heading-other.heading)

    def multiply(self, factor):

        return Pose(self.x*factor, self.y*factor, self.heading*factor)

    def divide(self, factor):

        return Pose(self.x/factor, self.y/factor, self.heading/factor)

    def getPosition(self):

        return (self.x, self.y)