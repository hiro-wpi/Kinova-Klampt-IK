#!/usr/bin/env python

import time
from os.path import dirname, abspath
import numpy as np

from klampt.model import ik, coordinates
from klampt.math import so3
from klampt import IKObjective, IKSolver, WorldModel, vis


class KinovaGen3IK():
    def __init__(self):
        # Klampt 
        # world
        self.world = WorldModel()
        coordinates.setWorldModel(self.world)
        parent_path = dirname(dirname(abspath(__file__)))
        self.world.loadElement(parent_path+"/robots/kinova_with_robotiq_85.rob")
        # robot
        self.robot = self.world.robot(0)
        # predefined variables
        self.link = self.robot.link("Tool_Frame")
        self.home_angles = [0, 0.4, np.pi, -np.pi+1.4, 0, -1, np.pi/2]
        self.robot.setConfig(self.joint_angles_to_config(self.home_angles))
        # visualize
        vis.add("robot", self.robot)
        coordinates.setWorldModel(self.world)
        vis.add("coordinates", coordinates.manager())


    def set_link(self, link):
        '''Set the IK target link'''
        if link == None or link == "end_effector":
            self.link = self.robot.link("Tool_Frame")
        elif link == "camera":
            self.link = self.robot.link("Camera_Link")
        elif type(link) is int or type(link) is str:
           self.link = self.robot.link(link)

    def set_angles(self, angles):
        '''Set current robot configuration'''
        if len(angles) == 7:
            self.robot.setConfig(self.joint_angles_to_config(angles))
        elif len(angles) == self.robot.numLinks():
            self.robot.setConfig(angles)

    def set_home_angles(self, home):
        '''Set robot home configuration'''
        if len(home) == 7:
            self.home_angles = home
        elif len(home) == self.robot.numLinks():
            self.home_angles = self.config_to_joint_angles(home)
        elif len(home) == 3:
            res, config = self.solve_ik(home)
            if res:
                self.home_angles = self.config_to_joint_angles(config)
            else:
                print("Cannot find a corresponding configuration")

    def set_link_pose(self, pos, rot):
        '''Set current robot pose'''
        res, config = self.solve_ik(pos, rot)


    def get_link(self):
        '''Get the IK target link name and index'''
        return self.link.getIndex(), self.link.getName()

    def get_angles(self):
        '''Get current robot configuration'''
        config = self.robot.getConfig()
        return self.wrap_to_pi(self.config_to_joint_angles(config))

    def get_home_angles(self):
        '''Get robot home configuration''' 
        return self.home_angles

    def get_link_pose(self, link=None, rot_type="rpy"):
        '''Get the pose of a link''' 
        if link == None:
            link = self.link
        R, t = link.getTransform()
        if rot_type == "rpy":
            R = so3.rpy(R)
        elif rot_type == "quaternion":
            R = so3.quaternion(R)
        return R, t


    def solve_ik(self, world_pos, world_rot=None, link=None, local_pos=(0,0,0),
                 max_iteration=100, tolerance=1e-3):
        '''Solve IK given a target position, 
           optionally a link and a local position
        '''
        # Initialization
        self.set_link(link)

        # Set target
        # point constraint
        if world_rot == None:
            obj = ik.objective(self.link, local=local_pos, world=world_pos)
        # fixed constraint
        else:
            # rotation angles zyx / rpy
            if len(world_rot) == 3:
                R = so3.from_rpy(world_rot)
            # quaternion [w, x, y, z]
            elif len(world_rot) == 4:
                R = so3.from_quaternion(world_rot)
            # rotation matrix
            elif len(world_rot) == 9: 
                R = world_rot
            obj = ik.objective(self.link, R=R, t=world_pos)
        solver = IKSolver(self.robot)
        solver.add(obj)

        # Solve IK
        solver.setMaxIters(max_iteration)
        solver.setTolerance(tolerance)
        res = solver.solve() 
        # itr = s.lastSolveIters()

        # Convert to angles
        result_config = self.robot.getConfig()
        angles = self.wrap_to_pi(self.config_to_joint_angles(result_config))

        return res, angles


    def visualize(self):
        '''Visualize robot with current configuration'''
        vis.show()
        while vis.shown():
            vis.lock()
            coordinates.updateFromWorld()
            vis.unlock()
            time.sleep(0.05)

    def joint_angles_to_config(self, joint_angles):
        '''Convert 7 joint angles to Klampt robot config'''
        config = self.robot.getConfig()
        config[1:8] = joint_angles[0:7]
        return config

    def config_to_joint_angles(self, config):
        '''Convert Klampt robot config to 7 joint angles'''
        return config[1:8]

    def wrap_to_pi(self, angles):
        '''Wrap a list of angles to (-pi, pi)'''
        array = np.array(angles)
        array = array - 2*np.pi * np.floor((array+np.pi)/(2*np.pi))
        return array.tolist()


# Test class
if __name__ == "__main__":
    kinova_ik = KinovaGen3IK()

    # point ik
    print(kinova_ik.solve_ik([0.4, 0.2, 0.3])) # without rotation
    kinova_ik.visualize()
    # fixed ik
    print(kinova_ik.solve_ik([0.4, 0.2, 0.3], 
                             [0, np.sqrt(2)/2, np.sqrt(2)/2, 0])) #quaternion
    print(kinova_ik.solve_ik([0.4, 0.2, 0.3], 
                             [np.pi, 0, np.pi/2])) #zyx
    kinova_ik.visualize()
