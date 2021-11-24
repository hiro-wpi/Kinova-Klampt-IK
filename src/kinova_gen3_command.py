#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray, Float64


class KinovaGen3Command():

    def __init__(self):
        # ROS
        # subscribe to IK solution
        self.ik_sub = rospy.Subscriber("kinova_gen3_ik/result_config", 
                                        Float64MultiArray, self.ik_callback)
        self.vel_ik_sub = rospy.Subscriber("kinova_gen3_vel_ik/result_config", 
                                        Float64MultiArray, self.vel_ik_callback)
        # publish to Kinova API
        # This will differ depending on the platform you are using Kinova
        # 7 Joints have different topics (Gazebo)
        j1_pub = rospy.Publisher("joint_1_position_controller/command", 
                                  Float64, queue_size=1)
        j2_pub = rospy.Publisher("joint_2_position_controller/command", 
                                  Float64, queue_size=1)
        j3_pub = rospy.Publisher("joint_3_position_controller/command", 
                                  Float64, queue_size=1)
        j4_pub = rospy.Publisher("joint_4_position_controller/command", 
                                  Float64, queue_size=1)
        j5_pub = rospy.Publisher("joint_5_position_controller/command", 
                                  Float64, queue_size=1)
        j6_pub = rospy.Publisher("joint_6_position_controller/command", 
                                  Float64, queue_size=1)
        j7_pub = rospy.Publisher("joint_7_position_controller/command", 
                                  Float64, queue_size=1)
        self.joint_pubs = [j1_pub, j2_pub, j3_pub, j4_pub, j5_pub, j6_pub, j7_pub]
        # initialize message
        self.joint_messages = 7 * [Float64()]


    def ik_callback(self, data):
        # Get result
        res, angles = data.data[7], data.data[0:7]
        self.convert_and_pub(angles)
        
    def vel_ik_callback(self, data):
        # Get result
        res, angles = data.data[7], data.data[0:7]
        self.convert_and_pub(angles)
    
    def convert_and_pub(self, angles):
        # Convert angles array to 7 Float64 
        # and publish them seperately
        for i in range(7):
            self.joint_messages[i].data = angles[i]
            self.joint_pubs[i].publish(self.joint_messages[i])


if __name__ == "__main__":
    # Launch ros node class
    rospy.init_node('kinova_gen3_command')
    kinova_gen3_command = KinovaGen3Command()

    # Set rate and run
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        r.sleep()
