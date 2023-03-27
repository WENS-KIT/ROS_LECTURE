#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chat', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    while not rospy.is_shutdown():
        message = input("Enter a message: ")
        rospy.loginfo("Sending message: %s", message)
        pub.publish(message)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
