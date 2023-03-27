#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " received message: %s", data.data)
    if data.data =='exit':
        rospy.signal_shutdown(-1)

def listener():
    print("Listener Start")
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chat", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
