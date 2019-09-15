#!/usr/bin/env python

import sys
import rospy

from pluto_node_handler.srv import *

def send_msg_client(msg):
    rospy.wait_for_service('pluto_node_handler_server')
    try:
        add_two_ints = rospy.ServiceProxy('start', String)
        resp1 = start("start something")#(x, y)
        return resp1.response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "usage: %s str "%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        msg = sys.argv[1]
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s"%(msg)
    print "RCV: %s"%(send_msg_client(msg))
	
	
