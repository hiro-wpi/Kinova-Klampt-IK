#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension
from kinova_gen3_ik import KinovaGen3IK


class KinovaGen3IKNode():

    def __init__(self):
        # ROS
        # subscribe to keyboard input
        self.point_sub = rospy.Subscriber("kinova_gen3_ik/point_ik", 
                                          Point, self.point_callback)
        self.fixed_sub = rospy.Subscriber("kinova_gen3_ik/fixed_ik", 
                                          Pose, self.fixed_callback)
        self.ini_sub = rospy.Subscriber("kinova_gen3_ik/set_init_angles", 
                                        Float64MultiArray, self.init_callback)
        self.link_sub = rospy.Subscriber("kinova_gen3_ik/set_ik_link", 
                                         String, self.link_callback)
        # publish the control command
        self.res_pub = rospy.Publisher("kinova_gen3_ik/result_config", 
                                       Float64MultiArray, queue_size=1)
        # initialize message
        self.joint_message = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.label = "Kinova IK Joint Angles"
        dim.size = 8
        dim.stride = 1
        self.joint_message.layout.dim = [dim]

        # IK model
        self.kinova_ik = KinovaGen3IK()


    def point_callback(self, data):
        # Get target
        target_pos = [data.x, data.y, data.z]
        # Solve IK
        res, angles = self.kinova_ik.solve_ik(target_pos)
        # Publish result
        res_num = 1 if res else 0
        data = angles[:]
        data.append(res_num)
        self.joint_message.data = data

        self.res_pub.publish(self.joint_message)
        
    def fixed_callback(self, data):
        # Get target
        target_pos = [data.position.x, data.position.y, data.position.z]
        target_quat = [data.orientation.w, data.orientation.x, data.orientation.y,
                       data.orientation.z]
        # Solve IK
        res, angles = self.kinova_ik.solve_ik(target_pos, target_quat)
        # Publish result
        res_num = 1 if res else 0
        data = angles[:]
        data.append(res_num)
        self.joint_message.data = data

        self.res_pub.publish(self.joint_message)

    
    def init_callback(self, data):
        # Get angles
        ini_angles = data.data
        # Initialize a robot configuration
        self.kinova_ik.set_angles(ini_angles)

    def link_callback(self, data):
        # Set link name
        self.kinova_ik.set_link(data.data)


if __name__ == "__main__":
    # Launch ros node class
    rospy.init_node('kinova_gen3_ik_node')
    kinova_gen3_ik_node = KinovaGen3IKNode()

    # Set rate and run
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        r.sleep()
