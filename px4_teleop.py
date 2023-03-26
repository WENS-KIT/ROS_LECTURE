import rospy
import keyboard
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


class PXController :
    def __init__(self):
        rospy.init_node('px4_teleop')

        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback=self.pose_cb)

        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.current_state = State()
        self.current_pose = PoseStamped()

        self.rate = rospy.Rate(20)
        self.pose = PoseStamped()

        self.pose.pose = self.current_pose.pose

    def state_cb(self, msg):
        self.current_state = msg
    
    def pose_cb(self, msg):
        self.current_pose = msg

    def connect(self):
        while(not rospy.is_shutdown() and not self.current_state.connected):
            rospy.loginfo_throttle(60, "Trying to connect MAVROS...")
            self.rate.sleep()
        rospy.loginfo("Connection successful")
        
    def takeoff(self, height):
        last_req = rospy.Time.now()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        self.pose.pose = self.current_pose.pose
        self.pose.pose.position.z = height
        
        rospy.loginfo("Takeoff... (h=%.2lf)", height)

        while not rospy.is_shutdown():
            if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(1.0)):
                if(self.set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")
                
                last_req = rospy.Time.now()
            else:
                if(not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(1.0)):
                    if(self.arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")
                
                    last_req = rospy.Time.now()


            if self.current_state.mode == "OFFBOARD" and self.current_state.armed :
                if height + height/10 >= self.current_pose.pose.position.z >= height - height/10:
                    break

            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

        rospy.loginfo("Takeoff successful")

    def land(self):
        land_set_mode = SetModeRequest()
        land_set_mode.custom_mode = 'AUTO.LAND'

        last_req = rospy.Time.now()

        rospy.loginfo("Land...")
        while not rospy.is_shutdown():

            if(self.current_state.mode != "AUTO.LAND" and (rospy.Time.now() - last_req) > rospy.Duration(1.0)):               
                self.set_mode_client.call(land_set_mode)
                last_req = rospy.Time.now()
                
            if self.current_state.mode == "AUTO.LAND" :
                rospy.loginfo("AUTO.LAND enabled")
                break;
                

            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()
    

    def run(self):
        rospy.loginfo("PX4 Teleop Node Start")

        self.connect()
        self.takeoff(2.0)

        rospy.loginfo("Keyboard Controll enabled")

        while not rospy.is_shutdown():

            if keyboard.is_pressed('w'):
                rospy.loginfo("Move Front")
                self.pose.pose.position.x += 0.5
            elif keyboard.is_pressed('x'):
                rospy.loginfo("Move Back")
                self.pose.pose.position.x -= 0.5
            elif keyboard.is_pressed('a'):
                rospy.loginfo("Move Left")
                self.pose.pose.position.y += 0.5
            elif keyboard.is_pressed('d'):
                rospy.loginfo("Move Right")
                self.pose.pose.position.y -= 0.5
            elif keyboard.is_pressed('s'):
                rospy.loginfo("Stop")
                self.pose.pose.position = self.current_pose.pose.position

            elif keyboard.is_pressed('u'):
                rospy.loginfo("Move Up")
                self.pose.pose.position.z += 0.5
            elif keyboard.is_pressed('j'):
                rospy.loginfo("Move Down")
                self.pose.pose.position.z -= 0.5

            elif keyboard.is_pressed('l'):
                self.land()
                break

            elif keyboard.is_pressed('q'):
                rospy.loginfo("Emergence Quit")
                rospy.signal_shutdown(-1)
                break

            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

if __name__ == "__main__":

    controller = PXController()
    controller.run()