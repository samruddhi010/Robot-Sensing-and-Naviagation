#!/usr/bin/env python3

from pa1.srv import AddTwofloats,AddTwofloatsResponse
import rospy

def handle_add_two_floatss(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwofloatsResponse(req.a + req.b)

def add_two_floats_server():
    rospy.init_node('add_two_floats_server')
    s = rospy.Service('add_two_floats', AddTwofloats, handle_add_two_floats)
    print("Ready to add two floats.")
    rospy.spin()

if __name__ == "__main__":
    add_two_floats_server()
