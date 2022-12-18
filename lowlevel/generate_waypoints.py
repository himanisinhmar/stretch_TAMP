#!/usr/bin/env python

"""
Description: Given its current position, find a series of waypoints for the robot to execute to reach a given target on the table. The trajectory is a move-base-then-arm scheme.
Each waypoint is 5d [x y theta z d]

For the waypoints that just move the base: [x y NaN z_stow d_stow]
For the waypoints that just move the arm: [targetx targety theta z d]

Author: Amy Fang
"""

import sys
import os
import numpy as np
import math
import numpy.random
import scipy.stats as ss
import shapely.geometry as geometry
import matplotlib.pyplot as plt
import givenSamplePoint_findTraj
from helperFuncs import robot2Global

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                 "/../../PythonRobotics/")

from PathPlanning.VisibilityRoadMap import visibility_road_map as vrm

def findAngle(base_xy, target_xy):
    '''
    find the angle between two points (rad), where the angle is taken relative to positive x-axis
    '''
     
    delta_x = target_xy[0] - base_xy[0]
    delta_y = target_xy[1] - base_xy[1]
    theta = math.atan2(delta_y, delta_x)

    # vecTemp = [target_xy[0] - base_xy[0],target_xy[1] - base_xy[1]]
    # vecTemp = vecTemp/np. linalg. norm(vecTemp)
    # dotProd = np.dot(vecTemp,np.array([1,0]))
    # theta = np.arccos(dotProd)

    return theta

def rotate(origin, point, theta):

    px, py = point
    ox, oy = origin

    qx = ox + math.cos(theta) * (px - ox) - math.sin(theta) * (py - oy)
    qy = oy + math.sin(theta) * (px - ox) + math.cos(theta) * (py - oy)

    return qx, qy

def rotate_round(origin, point, theta):

    px, py = point
    ox, oy = origin

    qx = ox + math.cos(theta) * (px - ox) - math.sin(theta) * (py - oy)
    qy = oy + math.sin(theta) * (px - ox) + math.cos(theta) * (py - oy)

    return round(qx, 2), round(qy, 2)

def findWorkspaceCoords(table_bloat, table_coords, table_origin, table_theta):
    '''
    table_bloat: how much to bloat table by (to determine what area to sample from) 1x2 np array
    current_xy: current base position
    goal: target position in table to reach

    process - make table axis-aligned, bloat, then rotate
    '''
    table_rot = [rotate_round(table_origin, pt, -table_theta) for pt in table_coords]

    outer_coords = []
    minX, minY = min(table_rot, key=lambda coord: (coord[0], coord[1]))
    maxX, maxY = max(table_rot, key=lambda coord: (coord[0], coord[1]))

    table_rot = np.array(table_rot)
    minX = min(table_rot[:,0])
    minY = min(table_rot[:,1])
    maxX = max(table_rot[:,0])
    maxY = max(table_rot[:,1])


    # minX, minY = round(minX,10), round(minY,10)
    # maxX, maxY = round(maxX,10), round(maxY,10)

    for coord in table_rot:
        if (coord == [minX,minY]).all():
            outer_coord = coord - table_bloat
        elif (coord == [maxX,maxY]).all():
            outer_coord = coord + table_bloat
        elif (coord == [minX, maxY]).all():
            outer_coord = coord + np.array([-table_bloat, table_bloat])
        elif (coord == [maxX, minY]).all():
            outer_coord = coord + np.array([table_bloat, -table_bloat])
        try:
            outer_coords.append(tuple(outer_coord))
        except:
            print(coord)
            outer_coords.append(tuple(outer_coord))

    # workspace min max
    minXw, minYw = min(outer_coords, key=lambda coord: (coord[0], coord[1]))
    maxXw, maxYw = max(outer_coords, key=lambda coord: (coord[0], coord[1]))
    outer_coordArr = np.array(outer_coords)
    minXw = min(outer_coordArr[:,0])
    minYw = min(outer_coordArr[:,1])
    maxXw = max(outer_coordArr[:,0])
    maxYw = max(outer_coordArr[:,1])

    outer_rot = [rotate(table_origin, pt, table_theta) for pt in outer_coords]
    

    # # # table 
    # plt.figure(3)
    # xt = [x[0] for x in table_coords]
    # yt = [x[1] for x in table_coords]

    # xt.append(xt[0]), yt.append(yt[0])
    # plt.plot(xt,yt, 'b')

    # xw = [x[0] for x in outer_rot]
    # yw = [x[1] for x in outer_rot]

    # xw.append(xw[0]), yw.append(yw[0])
    # plt.plot(xw,yw, 'r')

    # plt.axis("equal")

    # plt.show()
    # asd

    return outer_rot, maxXw-minXw, maxYw-minYw
    
def findFreeSpace(workspace_coords, table_coords):
    '''
    create polygon of free space to sample from
    '''

    exterior = workspace_coords + [workspace_coords[0]]
    interior = table_coords + [table_coords[0]][::-1]


    freespace = geometry.polygon.Polygon(exterior, interior)
    return freespace


def sampleBasePose(freespace, current_xy, target_xy, stdev):
    ''' sample based on gaussian distribution

    outputs [x y theta]
    '''

    stdev_x, stdev_y = stdev
    sampleXY = [1000, 1000]
    
    # sample until the point is in free space
    while not freespace.contains(geometry.Point(sampleXY)):
        sampleX = np.random.normal(current_xy[0], stdev_x)
        sampleY = np.random.normal(current_xy[1], stdev_y)
        sampleXY = [sampleX, sampleY]

    # calculate theta according to line connecting x y and position of the target within table
    if target_xy[0]-sampleXY[0] == 0:
        theta = 0
    else:
        theta = findAngle(sampleXY, target_xy)

    return [sampleX, sampleY, theta+np.pi/2]

def calculateStDev(workspace_coords, length, width, stdev_scale):
    '''
    returns 1x2 of st dev for x and y 
    stdev_scale: ratio of standard deviation relative to workspace area (ex: 0.25 means one stdev is 1/4 of the workspace area)

    '''
     # workspace min max
    # minX, minY = min(workspace_coords, key=lambda coord: (coord[0], coord[1]))
    # maxX, maxY = max(workspace_coords, key=lambda coord: (coord[0], coord[1]))
    
    stdev = [stdev_scale*length, stdev_scale*width]

    return stdev

def findNextBasePose(stretch, objects_mesh, workspace_coords, work_length, work_width, table_coords, current_xy, current_z, currentArmExtension, retract, EE_obj, target_xy, stdev_scale, arm_offset, n_iter, show_animation = False):
    '''
    stdev_scale: ratio of standard deviation relative to workspace area (ex: 0.25 means one stdev is 1/4 of the workspace area)
    offset_xy: pose of arm wrt robot frame

    waypoints: n x 5 array of base and arm poses
    '''

    stdev = calculateStDev(workspace_coords, work_length, work_width, stdev_scale)
    freespace = findFreeSpace(workspace_coords, table_coords)

    collision_free = False
    counter = 1
    stepSize_z = 0.05

    """ NEW INPUTS ADDED FOR find_z_d function """
    # radius_dummy = 0.2;
    # length_dummy = 0.8
    # objectHolding = False;
    # # for stowed position
    # currentArmExtension = 0 # REPLACE with real time DATA
    # current_z = stretch.defaultArmHeight # REPLACE with real time DATA

    # retract = False 
    # # currentArmExtension = 1.6876803062867025 # REPLACE with real time DATA
    # # current_z = 2.2 # REPLACE with real time DATA

    current_theta = findAngle(current_xy, target_xy) + np.pi/2
    samplePose = [current_xy[0],current_xy[1],current_theta]
    # samplePose_gripper = [samplePose[0] + arm_offset[0],samplePose[1] + arm_offset[1]]
    samplePose_gripper = robot2Global(samplePose, arm_offset[0:2],'lis')
    samplePose_gripper.append(findAngle(samplePose_gripper,target_xy))
    print('sample pose gripper: {}'.format(samplePose_gripper))

    # find global coordinates of gripper and the object it is holding
    EE_loc = find_gripper_loc(samplePose_gripper,stretch,currentArmExtension,current_z)
    # EE_obj = find_dummy_objCoord(samplePose_gripper,radius_dummy,length_dummy,stretch,objectHolding,EE_loc) # REPLACE with real time DATA

    if retract:
        print("samplePose_gripper passed to find_z_d = ",samplePose_gripper)
        print("goal passed to find_z_d = ",target_xy)
        print("gripper's tip xyz coords passed to find_z_d = ",EE_loc)
        print("robot base pose passed to find_z_d = ",samplePose)
        collision_free,waypoints = givenSamplePoint_findTraj.find_z_d(target_xy,samplePose_gripper,samplePose,stepSize_z,stretch,objects_mesh,EE_obj,EE_loc, retract)
    else:
        # first try to reach it at its current position
        print("samplePose_gripper passed to find_z_d = ",samplePose_gripper)
        print("goal passed to find_z_d = ",target_xy)
        print("gripper's tip xyz coords passed to find_z_d = ",EE_loc)
        print("robot base pose passed to find_z_d = ",samplePose)
        collision_free, waypoints = givenSamplePoint_findTraj.find_z_d(target_xy,samplePose_gripper,samplePose,stepSize_z,stretch,objects_mesh, EE_obj, EE_loc, retract)
        while (not collision_free and counter <= n_iter):
            samplePose = sampleBasePose(freespace, current_xy, target_xy, stdev)
            # samplePose_gripper = [samplePose[0] + arm_offset[0],samplePose[1] + arm_offset[1]]
            # samplePose_gripper.append(findAngle(samplePose_gripper,target_xy))
            samplePose_gripper = robot2Global(samplePose, arm_offset[0:2],'lis')
            samplePose_gripper.append(findAngle(samplePose_gripper, target_xy))

            # EE_obj = find_dummy_objCoord(samplePose_gripper,radius_dummy,length_dummy,stretch,objectHolding) # REPLACE with real time DATA
            EE_loc = find_gripper_loc(samplePose_gripper,stretch,currentArmExtension,current_z)

            # CHECK FOR COLLISION CHECKING FUNCTION GOES HERE ---------------------
            #[x y theta z d]
            print("samplePose_gripper passed to find_z_d = ",samplePose_gripper)
            print("goal passed to find_z_d = ",target_xy)
            print("gripper's tip xyz coords passed to find_z_d = ",EE_loc)
            print("robot base pose passed to find_z_d = ",samplePose)
            collision_free,waypoints = givenSamplePoint_findTraj.find_z_d(target_xy,samplePose_gripper,samplePose,stepSize_z,stretch,objects_mesh,EE_obj,EE_loc, retract)
            counter += 1


    # outside of both while loops - check if solution was found
    if not collision_free:
        raise ValueError('No solution found! Either no solution exists, or try increasing stdev_scale / n_iter')

    return samplePose, waypoints

def find_dummy_objCoord(samplePose_gripper,radius,length,stretch,objectHolding,EE_loc):
    if objectHolding:
        x_obj = samplePose_gripper[0] + (stretch.defaultArmDepth + 0.1)*np.cos(samplePose_gripper[2])
        y_obj = samplePose_gripper[1] + (stretch.defaultArmDepth + 0.1)*np.sin(samplePose_gripper[2])
        z_obj = EE_loc[2] - length
        EE_obj = [x_obj,y_obj,z_obj,radius]
    else:
        EE_obj = []

    return EE_obj

def find_gripper_loc(samplePose_gripper,stretch,currentArmExtension,current_z):
    EE_loc = [samplePose_gripper[0]+(stretch.defaultArmDepth+currentArmExtension)*np.cos(samplePose_gripper[2]),
                samplePose_gripper[1]+(stretch.defaultArmDepth+currentArmExtension)*np.sin(samplePose_gripper[2]),
                current_z]
    return EE_loc

def createRoadMap(q_start, q_goal, obstacle_list, expand_distance, show_animation = False):
    ''' create visibility roadmap

    obstacle_list: list of obstacles with a list of coordinates in cw direction
    expand_distance: how much to bloat obstacle by

    outputs path as a list of x coords and a list of y coords
    '''
    obstacles = []
    for obs in obstacle_list:
        obs_x = [p[0] for p in obs]
        obs_y = [p[1] for p in obs]
        obstacles.append(vrm.ObstaclePolygon(obs_x, obs_y))
    
    rx, ry = vrm.VisibilityRoadMap(expand_distance, do_plot=True).planning(q_start[0], q_start[1], q_goal[0], q_goal[1], obstacles)


    if show_animation: 
        plt.axis("equal")
        # start and goalpoints
        plt.plot(q_start[0], q_start[1], "or", label='start')
        plt.plot(q_goal[0], q_goal[1], "og", label='goal')
        for ob in obstacles:
            ob.plot()

        plt.plot(rx, ry, "--c", label='path')

        plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
        plt.tight_layout()
        
        plt.pause(0.1)
        

    return rx, ry

def generate_full_traj(stretch, objects_mesh,current_pose, current_z, currentArmExtension, retract, EE_obj, target_xy, table_coords, table_origin, table_theta, bloat_factor, stdev_scale, arm_offset, n_iter):
    ''' outputs full series of waypoints to reach target point on table as list of lists, 
    where each nested list is the state of the robot [x y theta z d]

    for the waypoints that just move the base: [x y NaN z_stow d_stow]
    for the waypoints that just move the arm: [targetx targety theta z d]
    
    current_z: gripper's current z position in global coordinates
    currentArmExtension: scalar value of how much the arm is extended from stowed position
    retract: set False for now, set true if robot needs to retract arm
    EE_obj: set as empty [] if gripper not holding any object, else EE_obj = [<object's tip x>, <object's tip y>, <object's tip z>, <largest radius of the object from tip]

    '''

    expand_distance = bloat_factor/2

    workspace_coords, length, width = findWorkspaceCoords(bloat_factor, table_coords, table_origin, table_theta)
    q_goal, arm_wp = findNextBasePose(stretch, objects_mesh, workspace_coords, length, width, [table_coords], current_pose, current_z, currentArmExtension, retract, EE_obj, target_xy, stdev_scale, arm_offset, n_iter)
    roadmap_x, roadmap_y = createRoadMap(current_pose, q_goal, [table_coords], expand_distance)

    full_traj = []
    arm_stowed = [0,0]      # will actually to find this value
    for i in range(len(roadmap_x)):
        state = [roadmap_x[i], roadmap_y[i], float('NaN')]
        state.extend(arm_stowed)
        full_traj.append(state)

    for i in range(len(arm_wp)):
        full_traj.append(arm_wp[i])

    return full_traj


def main():
    '''
    for testing
    '''
    show_animation = True

    # table coordinates
    coords = [(0, 0), (0, 4), (8, 4), (8,0)]
    current_pose = (0.2, 4.2)
    target_xy = (4,3)


    bloat_factor = 0.5
    stdev_scale = 0.25
    xy_offset = [0.1, 0.1, 0]  # will need to figure out what this actually is
    n_iter = 1000

    workspace_coords = findWorkspaceCoords(bloat_factor, coords)

    # for testing the sampling
    samples = []
    for i in range(1000):
        s, arm_wp = findNextBasePose(workspace_coords, coords, current_pose, target_xy, stdev_scale, xy_offset, n_iter, show_animation= show_animation)
        samples.append(s)

    # start and goal position (for now, just picking the last sample)
    q_goal = samples[-1]

    expand_distance = bloat_factor/2  # have the edges of the roadmap go down the middle of the corridor

    roadmap_x, roadmap_y = createRoadMap(current_pose, q_goal, coords, expand_distance, show_animation = show_animation)
    print(roadmap_x)
    print(roadmap_y)


    # --------------------- PLOTTING ---------------------

    if show_animation:
    # table 
        plt.figure(3)
        xt = [x[0] for x in coords]
        yt = [x[1] for x in coords]

        xt.append(xt[0]), yt.append(yt[0])
        plt.plot(xt,yt, 'b')

        # workspace 
        xw = [x[0] for x in workspace_coords]
        yw = [x[1] for x in workspace_coords]

        xw.append(xw[0]), yw.append(yw[0])

        plt.plot(xw,yw, 'm')
        
        # samples
        xs = [x[0] for x in samples]
        ys = [x[1] for x in samples]

        plt.plot(xs,ys, 'y.', label='samples')

        # current xy
        plt.plot(current_pose[0],current_pose[1], 'r*', label='current')
        plt.plot(target_xy[0],target_xy[1], 'c*', label='goal')
        plt.plot(q_goal[0],q_goal[1], 'g*', label='goal base')

        plt.axis("equal")
        plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
        plt.tight_layout()
        plt.show()



if __name__ == '__main__':
    # main()

    # table coordinates
    coords = [(0, 0), (0, 4), (8, 4), (8,0)]
    # coords_obs = [(-0.4,3), (-0.4, 4.2), (2, 4.2), (2,3)]
    current_pose = (0.2, 4.2)
    target_xy = (4,3)


    bloat_factor = 0.5
    stdev_scale = 0.25
    arm_offset = [0.1, 0.1, 0]  # will need to figure out what this actually is
    n_iter = 1000

    waypoints = generate_full_traj(current_pose, target_xy, coords, bloat_factor, stdev_scale, arm_offset, n_iter)
    print(waypoints)



