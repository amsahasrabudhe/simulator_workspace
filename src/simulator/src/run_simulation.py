#!/usr/bin/python3

from simulator import *

if __name__ == "__main__":

    rospy.init_node("simulator")
    simulator = Simulator()

    rospy.spin()
