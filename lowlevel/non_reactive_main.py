#!/usr/bin/python3

"""
Description: Given a high level plan in the form of "sas_plan", execute each action in order. Connect to Optitrack
topics and the stretch robot to send commands.

Author: David Gundana

"""

import time, socket, json, pickle
import sys
import os
import threading
import math
import numpy as np
import time
import rospy
import re
from geometry_msgs.msg import PoseStamped
from generate_waypoints import generate_full_traj
import robot_class
from environment import getScene
import stretch_body.robot
import tf2_ros
import tf
from helperFuncs import global2robot, robot2Global, feedbackLin, limitCmds, euler_from_quaternion
from gofromto import gofromto
from grip import grip
from move import move

class runRobot:
    def __init__(self,envJSON,plan,robot):
        self.envJSON = envJSON
        self.plan = plan
        self.trackableObjects = ['table','pot','bowl','ladle']
        self.tableDim = [1.52,.76]
        self.pos = (np.size(self.trackableObjects) + 1) * [[]]
        self.totalArmDepth = 0.5
        self.totalArmHeight = 1.098
        self.defaultArmDepth = 0.49
        self.defaultArmHeight = 0
        self.maxV = .15
        self.distToGoal = .1
        self.wheel2Center = .17
        self.bloat_factor = .1
        self.stdev_scale = 0.75
        self.arm_offset = [0.02+0.08, 0.13, 0]  # will need to figure out what this actually is
        self.n_iter = 1000
        self.closeEnough = .1
        self.closeAngEnough = .05
        self.robot = robot

    def startExec(self):
        print('Starting Optitrack Thread')
        rospy.init_node('object_listener')
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(10.0)
        reachedFinal = 0
        isAligned = 0
        waypoints = []
        indWay = 0
        prop2Activate = self.plan
        ind = 0
        foundWaypoints = 0
        while not rospy.is_shutdown():
            try:
                stretch = tfBuffer.lookup_transform("world", 'stretch/base_link', rospy.Time())
                objects = (np.size(self.trackableObjects)) * [[]]
                for i in range(np.size(self.trackableObjects)):
                    subTopic = "{}/base_link".format(self.trackableObjects[i])
                    try:
                        objects[i] = tfBuffer.lookup_transform("world", subTopic, rospy.Time(),rospy.Duration(0.01))
                    except:
                        objects[i] = None
                        pass
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
            # Calculate PosVec for Stretch
            self.trackBody(stretch,0)
            for i in range(np.size(objects)):
                if objects[i] is not None:
                    self.trackBody(objects[i],i+1)

            #Send commands
            currentCommand = re.split(' ',prop2Activate[ind])

            if 'gofromto' in currentCommand[0]:
                foundWaypoints, isAligned, retractInit, foundW, ind, waypoints, indWay = \
                    gofromto(self,currentCommand, foundWaypoints,isAligned, ind, waypoints,indWay)
            if 'grip' in currentCommand[0]:
                retractInit, foundW, waypoints, ind = \
                    grip(self,retractInit, foundW, currentCommand, waypoints,ind)
            if 'do_liquid' in currentCommand[0]:
                print('No action for: {} ...'.format(currentCommand[0]))
                ind += 1
            if 'move' in currentCommand[0]:
                foundWaypoints, waypoints,indWay = move(self,currentCommand,foundWaypoints,waypoints, indWay)
            time.sleep(.2)


    def trackBody(self,data,index):
        # +z is +x. +x is +y +y is +z
        posVec = [-data.transform.translation.z,
                           data.transform.translation.x,
                           -data.transform.translation.y]
        roll_x, pitch_y, yaw_z = euler_from_quaternion(-data.transform.rotation.z, data.transform.rotation.x,
                                                          data.transform.rotation.y, data.transform.rotation.w)
        self.pos[index] = posVec + [roll_x, pitch_y, -yaw_z+np.pi/2]

    def getWaypoints(self,current_pos, tableCoords, posOfI, retract):
        visObjects = [i for i, x in enumerate(self.pos) if x][1:]
        objectPos = np.asarray([self.pos[i] for i in visObjects])
        visObjects = [x - 1 for x in visObjects]
        objectTemp = [self.trackableObjects[i] for i in visObjects]
        scene, outputObjects, outputAxis = getScene(objectTemp, objectPos, self.envJSON)
        tableLoc = self.pos[self.trackableObjects.index('table')+1][0:2] +\
                   [self.pos[self.trackableObjects.index('table')+1][-1]]
        # print(current_pos, posOfI, tableCoords,tableLoc[0:2], tableLoc[-1])
        current_z = self.robot.lift.status['pos']
        currentArmExtension = self.robot.arm.status['pos']
        EE_obj = []
        waypoints = generate_full_traj(self, outputObjects, current_pos, current_z, currentArmExtension, retract, EE_obj,
                                       posOfI, tableCoords, tableLoc[0:2],tableLoc[-1],
        self.bloat_factor,self.stdev_scale, self.arm_offset, self.n_iter)
        return waypoints

    def getTableCoords(self):
        self.tableDim[0] += 1
        self.tableDim[1] += 1
        tableLoc = self.pos[self.trackableObjects.index('table')+1][0:2] +\
                   [self.pos[self.trackableObjects.index('table')+1][-1]]
        tableH = np.sqrt((self.tableDim[0]/2)**2 + (self.tableDim[1]/2)**2)
        pt1 = [-self.tableDim[0]/2, -self.tableDim[1]/2]
        pt2 = [-self.tableDim[0]/2, self.tableDim[1]/2]
        pt3 = [self.tableDim[0]/2, self.tableDim[1]/2]
        pt4 = [self.tableDim[0]/2, -self.tableDim[1]/2]

        pt1 = robot2Global(tableLoc,pt1,'tup')
        pt2 = robot2Global(tableLoc,pt2,'tup')
        pt3 = robot2Global(tableLoc,pt3,'tup')
        pt4 = robot2Global(tableLoc,pt4,'tup')

        tableCoords =[pt1,pt2,pt3,pt4]
        return tableCoords

if __name__ == "__main__":
    # Load in the environment file if it is not an argument
    if '-plan' in sys.argv:
        planFile = sys.argv[sys.argv.index('-plan') + 1]
    else:
        # Default environment file
        planFile = 'sas_plan'

    # Load in the high level planner file if it is not an argument
    if '-env' in sys.argv:
        envFile = sys.argv[sys.argv.index('-env') + 1]
    else:
        # Default environment file
        envFile = 'environment.json'

    # Load in the environment file
    filePath = os.path.normpath(os.getcwd() + os.sep + os.pardir)
    envJSONPath = os.path.join(filePath, 'highlevel_planner',envFile)
    planPath = os.path.join(filePath, 'highlevel_planner',planFile)
    f = open(envJSONPath)
    envJSON = json.load(f)
    with open(planPath) as f:
        plan_nonreactive = f.readlines()

    # Initialize the stretch robot
    robot = stretch_body.robot.Robot()
    robot.startup()
    robot.end_of_arm.move_to('wrist_yaw', 0)
    robot.end_of_arm.move_to('stretch_gripper', 25)
    robot.lift.move_to(1)
    robot.arm.move_to(0)
    robot.push_command()
    time.sleep(5)
    # robot.stow()

    # Initialize the robot class which defines parameters
    r = runRobot(envJSON,plan_nonreactive,robot)

    # This is in a try except so that ctrl-c sends a final kill command to the robot and closes all threads.
    try:
        r.startExec()
    except rospy.ROSInterruptException as e:
        print(e)
        pass
    finally:
        # Stop the robot
        print('Stopping robot')
        r.robot.base.set_velocity(v_m=0, w_r=0)
        r.robot.push_command()
        robot.end_of_arm.move_to('stretch_gripper', 25)

        time.sleep(2)
        r.robot.base.set_velocity(v_m=0, w_r=0)
        r.robot.push_command()
        print('robot stopped')

    robot.base.set_velocity(v_m=0, w_r=0)
    robot.push_command()
    print('Complete')