#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May  9 09:34:43 2022

@author: cornell
"""


import numpy as np
import networkx as nx

import json
import matplotlib.pyplot as plt


import sys
import os
import subprocess

filePath = os.path.normpath(os.getcwd() + os.sep + os.pardir)
if(filePath not in sys.path):
    sys.path.append(filePath)
    sys.path.append(filePath + os.sep + 'lowlevel')
    sys.path.append('/home/cornell/Tools/PythonRobotics')

from lowlevel import generate_waypoints
from lowlevel import environment
from lowlevel import robot_class
import encode_pddl_inputs



envJSONPath = 'environment.json'
global_user_defined_initial_conditions = []

def getTableCoords(tableDim,tableLoc):
        pt1 = [tableLoc[0]-tableDim[0]/2, tableLoc[1]-tableDim[1]/2]
        pt2 = [tableLoc[0]-tableDim[0]/2, tableLoc[1]+tableDim[1]/2]
        pt3 = [tableLoc[0]+tableDim[0]/2, tableLoc[1]+tableDim[1]/2]
        pt4 = [tableLoc[0]+tableDim[0]/2, tableLoc[1]-tableDim[1]/2]

        tableCoords =[pt1,pt2,pt3,pt4]
        return tableCoords



def is_path_available(objects, objectPos, current_pose, target_xy, EE_obj, show_animation=True):

    f = open(envJSONPath)
    envJSON = json.load(f)

    scene, objects_mesh, outputAxis = environment.getScene(objects,objectPos,envJSON)

    stretch = robot_class.robot()

    tableDim = [1.52, 0.76]
    tableLoc = objectPos[0,0:2]
    # tableLoc = (1, 1)
    table_coords = getTableCoords(tableDim,tableLoc)
    table_origin = tableLoc
    table_theta = 0


    # for roadmap
    bloat_factor = 0.1
    # for sampling
    stdev_scale = 0.75
    arm_offset = [0.02+0.08, -0.13, 0]   # measured from center of base
    # arm_offset = [0.1, 0, 0]
    n_iter = 1000

    current_z = 0. # gripper's current z position in global coordinates
    currentArmExtension = 0. # scalar value of how much the arm is extended from stowed position
    retract = False # set False for now, set true if robot needs to retract arm

    waypoints=[]
    path_clear = True
    try:
        waypoints = generate_waypoints.generate_full_traj(stretch, objects_mesh,current_pose, current_z, currentArmExtension, \
                                                          retract, EE_obj, target_xy, \
                                                          table_coords, table_origin, table_theta, \
                                                          bloat_factor, stdev_scale, arm_offset, n_iter)
    except:
        print('no path')
        path_clear = False
    # print(waypoints)


    # show_animation = True
    if show_animation:
        # table
        plt.figure(3)
        xt = [x[0] for x in table_coords]
        yt = [x[1] for x in table_coords]

        xt.append(xt[0]), yt.append(yt[0])
        plt.plot(xt,yt, 'b')

        plt.plot(objectPos[1][0],objectPos[1][1],'ro',label='pot')
        plt.plot(objectPos[2][0],objectPos[2][1],'bo',label='bowl')
        plt.plot(objectPos[3][0],objectPos[3][1],'go',label='laddle')
        counter = 0

        arm_pose = waypoints[-1]
        # trajectory
        for wp in waypoints:
            counter += 1
            if wp[:2] != current_pose[:2]:
                if np.isnan(wp[2]):   # base waypoints
                    plt.plot(wp[0],wp[1],'g*')
                else:
                    armx = arm_pose[0]+(wp[4] + stretch.defaultArmDepth)*np.cos(arm_pose[2])
                    army = arm_pose[1]+(wp[4] + stretch.defaultArmDepth)*np.sin(arm_pose[2])
                    plt.plot(armx, army,'*', label=str(counter))

        # current xy
        plt.plot(current_pose[0],current_pose[1], 'r.', label='current')
        plt.plot(target_xy[0],target_xy[1], 'c.', label='goal')
        # plt.plot(q_goal[0],q_goal[1], 'g*', label='goal base')

        # objectPos = np.array([[1, 1, 1, 0,0,0],[0, 1, 1.4, np.pi,0,0],[2, 0.5, 1.2, 0,0,0],[0, 0, 1.25, np.pi,0,0]])

        plt.axis("equal")
        plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
        plt.tight_layout()
        plt.show()

    return path_clear


class base_obj():
    def __init__(self, name, pose, can_carry=False, has_liquid=False, able_liquid=False, \
                                              has_push=False, able_push=False):
        self.name = name
        self.pose = np.copy(pose)
        self.can_carry = can_carry
        self.has_liquid = has_liquid
        self.able_liquid = able_liquid
        self.has_push = has_push
        self.able_push = able_push



class env_state():
    def __init__(self, objects, at=None):
        self.objects = objects
        self.at = at

    def get_objects(self):
        # breakpoint()
        objects = [o.name  for o in self.objects.values()]

        objectPos = [o.pose  for o in self.objects.values()]

        objectPos = np.array(objectPos)

        return objects, objectPos

    def get_target(self, to):
        return self.objects[to].pose[0:3]

    def get_current_pose(self):
        return self.objects[self.at].pose[0:2]


    def set_current_pose(self, obj, new_obj):
        if('idle' not in obj):
            self.objects[obj].pose = self.objects[new_obj].pose

        self.at = new_obj


def run_pddl():
    with open('environment.json', 'r') as env_file:
        environment = json.load(env_file)
        objects = environment["objects"]
        with open('domain.pddl', 'w') as dmn_file:
            encode_pddl_inputs.writeDomainFile(dmn_file, objects)
        with open('problem.pddl', 'w') as prblm_file:
            # add it to the global list, and then just call the function
            # if(user_defined_initial_conditions):
            #     global_user_defined_initial_conditions.append(user_defined_initial_conditions)
            #     encode_pddl_inputs.user_defined_initial_conditions = global_user_defined_initial_conditions

            encode_pddl_inputs.writeProblemFile(prblm_file, objects)

    print('running \"./runit.sh\" to solve.')
    subprocess.call("./runit.sh", shell=True)



def find_valid_hl_solution(name_pose_dict):
    # G = nx.DiGraph(name='ConnectivityGraph')

    current_pose = (0, 0)

    idle  = base_obj('idle', name_pose_dict['idle'])
    table = base_obj('table', name_pose_dict['table'])
    pot   = base_obj('pot', name_pose_dict['pot'],has_liquid=True, able_liquid=True)
    bowl  = base_obj('bowl', name_pose_dict['bowl'], able_liquid=True)
    ladle = base_obj('ladle', name_pose_dict['ladle'], can_carry=True, able_liquid=True)

    # G.add_node(state)

    successful_plan = True
    while (True):
        state = env_state({'table': table,'pot': pot,'bowl':bowl,'ladle':ladle}, 'idle')
        run_pddl()
        # here is the solution
        planFile = 'sas_plan'
        with open(planFile) as f:
            plan_nonreactive = f.readlines()

        for i in range( len(plan_nonreactive) ):

            cmds = plan_nonreactive[i].split()

            if('gofromto' in cmds[0]  or 'move' in cmds[0]):
                go_from = cmds[1]
                go_to = cmds[3].strip(')')


                objects, objectPos = state.get_objects()
                # objects = ['table','pot','bowl','ladle']
                # objectPos = np.array([[1,   1,   1,    0,     0, 0],  \
                #                       [0.5, 1,   1.4,  np.pi, 0, 0], \
                #                       [1.5, 1.2, 1.2,  0,     0, 0], \
                #                       [0.7, 0.7, 1.25, np.pi, 0, 0]])
                # # for our purposes now, it just need to be away from the table
                # current_pose = (1.1, 0) # (x,y,?)
                # position of object (x,y,z)
                target_xy = state.get_target(go_to) #(1.2, 0.75, 1.0)

                EE_obj = [] # set as empty [] if gripper not holding any object, else EE_obj = [<object's tip x>, <object's tip y>, <object's tip z>, <largest radius of the object from tip]

                print('new action:')
                print(objects)
                print(objectPos)
                print(current_pose)
                print(target_xy)
                print(' ')

                bVal = is_path_available(objects, objectPos, current_pose, target_xy, EE_obj, show_animation=False)

                if(bVal == False):
                    print(f'need to add man_disable {go_from} {go_to}')
                    global_user_defined_initial_conditions.append(f'man_disable {go_from} {go_to}')
                    encode_pddl_inputs.user_defined_initial_conditions = global_user_defined_initial_conditions

                    # G.add_edge()
                    successful_plan = False
                    break
                else:
                    state.set_current_pose(go_from, go_to)
                    if('move' in cmds[0]):
                        state.set_current_pose(go_from, go_to)

        if(successful_plan):
            print('successful plan')
            break

if __name__ == "__main__":
    objects = {'idle': [0,   0,   0,    0,     0, 0], 'table': [1,   1,   0.75,    0,     0, 0], \
                'pot': [0.5, 1,   1.0,  np.pi, 0, 0], 'bowl': [1.5, 1.2, 0.9,  0,     0, 0], \
                'ladle': [0.7, 0.7, 0.8, np.pi, 0, 0] }
    find_valid_hl_solution(objects)


