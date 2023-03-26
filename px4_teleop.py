#!/usr/bin/env python

import rospy
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# Initialize the node and set the rate.
rospy.init_node('offboard_node')
rate = rospy.Rate(20)

# Set the message types.
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
setpoint_position_local = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

# Wait for the services to be available.
rospy.wait_for_service('/mavros/set_mode')
rospy.wait_for_service('/mavros/cmd/arming')

# Set the mode to OFFBOARD.
set_mode(0, 'OFFBOARD')

# Arm the vehicle.
arming(True)

# Set the setpoint message.
setpoint = PoseStamped()
setpoint.pose.position.x = 1.0
setpoint.pose.position.y = 0.0
setpoint.pose.position.z = 1.0

# Set the message header.
setpoint.header.stamp = rospy.Time.now()
setpoint.header.frame_id = 'map'

# Send the setpoint message.
while not rospy.is_shutdown():
    setpoint_position_local.publish(setpoint)
    rate.sleep()
