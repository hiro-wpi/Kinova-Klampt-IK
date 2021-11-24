#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension
from kinova_gen3_ik import KinovaGen3IK
import numpy as np


class KinovaGen3VelIKNode():

    def __init__(self):
        # ROS
        # subscribe to keyboard input
        self.link_sub = rospy.Subscriber("kinova_gen3_vel_ik/set_ik_link", 
                                         String, self.link_callback)
        self.pose_sub = rospy.Subscriber("kinova_gen3_vel_ik/set_current_pose", 
                                         Pose, self.pose_callback)
        self.twist_sub = rospy.Subscriber("kinova_gen3_vel_ik/cmd_vel",
                                          Twist, self.vel_callback)
        # publish the control command
        self.res_pub = rospy.Publisher("kinova_gen3_vel_ik/result_config", 
                                       Float64MultiArray, queue_size=1)
        # initialize joint message
        self.joint_message = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.label = "Kinova IK Joint Angles"
        dim.size = 8
        dim.stride = 1
        self.joint_message.layout.dim = [dim]

        # IK model
        self.kinova_ik = KinovaGen3IK()


    def pose_callback(self, data):
        # Set current pose
        curr_pos = [data.position.x, data.position.y, data.position.z]
        curr_quat = [data.orientation.w, data.orientation.x, data.orientation.y,
                     data.orientation.z]
        self.kinova_ik.set_link_pose(curr_pos, curr_quat)

    def vel_callback(self, data):
        # Get velocities
        lin_vel = np.array([data.linear.x, data.linear.y, data.linear.z])
        ang_vel = np.array([data.angular.z, data.angular.y, data.angular.x])

        # Compute new pose
        t = 1.0/30
        curr_rotation, curr_position = self.kinova_ik.get_link_pose()
        new_position = (np.array(curr_position) + t*lin_vel).tolist()
        new_rotation = (np.array(curr_rotation) + t*ang_vel).tolist()
        
        # Solve IK
        res, angles = self.kinova_ik.solve_ik(new_position, new_rotation)
        # Publish result
        res_num = 1 if res else 0
        data = angles[:]
        data.append(res_num)
        self.joint_message.data = data

        self.res_pub.publish(self.joint_message)


    def link_callback(self, data):
        # Set link name
        self.kinova_ik.set_link(data.data)


if __name__ == "__main__":
    # Launch ros node class
    rospy.init_node('kinova_gen3_vel_ik_node')
    kinova_gen3_vel_ik_node = KinovaGen3VelIKNode()

    # Set rate and run
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        r.sleep()
