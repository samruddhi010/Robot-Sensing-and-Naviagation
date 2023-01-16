#!/usr/bin/env python3


import sys
import rospy
from pa1.srv import *

def add_two_floats_client(x, y):
    rospy.wait_for_service('add_two_floats')
    try:
        add_two_floats = rospy.ServiceProxy('add_two_floats', AddTwofloats)
        resp1 = add_two_floats(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_floats_client(x, y)))
