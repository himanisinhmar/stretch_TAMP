import environment
import givenSamplePoint_findTraj
import generate_waypoints
import math
import numpy as np
import os
import json
import scipy #need the graphics engine from scipy to show objects
import robot_class
import matplotlib.pyplot as plt

objects = ['table','pot','bowl','ladle']
objectPos = np.array([[1, 1, 1, 0,0,0],[0, 1, 1.4, np.pi,0,0],[2, 0.5, 1.2, 0,0,0],[0, 0, 1.25, np.pi,0,0]])
filePath = os.path.normpath(os.getcwd() + os.sep + os.pardir)
envJSONPath = os.path.join(filePath, 'highlevel_planner','environment.json')
f = open(envJSONPath)
envJSON = json.load(f)

scene, objects_mesh, outputAxis = environment.getScene(objects,objectPos,envJSON);

stretch = robot_class.robot();
stepSize_z = 0.1;

# ### Scenario 1: move to the bowl
# pose = np.array([1.2,-1.5,np.pi/2]);
# goal = np.array([2,0.5,1.2]);

# ### Scenario 2: move to the pot
# # pose = np.array([-0.2,-1.5,np.pi/2]);
# # goal = np.array([0,1,1.2]);

# ### find theta:
# vecTemp = goal[0:2] - pose[0:2]
# vecTemp = vecTemp/np. linalg. norm(vecTemp)
# dotProd = np.dot(vecTemp,np.array([1,0]))
# theta = np. arccos(dotProd)
# pose[2] = theta

# goodPoint,traj = givenSamplePoint_findTraj.find_z_d(goal,pose,stepSize_z,stretch,objects_mesh);

# print("Is sampled point a goodpoint: ",goodPoint)
# print("trajectory: ",traj)

def getTableCoords(tableDim,tableLoc):
        pt1 = [tableLoc[0]-tableDim[0]/2, tableLoc[1]-tableDim[1]/2]
        pt2 = [tableLoc[0]-tableDim[0]/2, tableLoc[1]+tableDim[1]/2]
        pt3 = [tableLoc[0]+tableDim[0]/2, tableLoc[1]+tableDim[1]/2]
        pt4 = [tableLoc[0]+tableDim[0]/2, tableLoc[1]-tableDim[1]/2]

        tableCoords =[pt1,pt2,pt3,pt4]
        return tableCoords

def rotate(origin, point, theta):

    px, py = point
    ox, oy = origin

    qx = ox + math.cos(theta) * (px - ox) - math.sin(theta) * (py - oy)
    qy = oy + math.sin(theta) * (px - ox) + math.cos(theta) * (py - oy)

    return qx, qy



#########
tableDim = (4,3)
tableLoc = (1,1)
table_coords = getTableCoords(tableDim,tableLoc)
table_origin = tableLoc
table_theta = 0
print(table_coords)
# #rotate table
# table_coords = [(0, 0), (0, 4), (8, 4), (8,0)]

# #rotate table
# table_origin = (0.2, -0.4)
# table_theta = -np.pi/4
# # table_coords = [rotate(xy, theta_table) for xy in table_coords]
# table_coords = [generate_waypoints.rotate(table_origin, pt, table_theta) for pt in table_coords]

# cuurentPose = (1, -1, 2.0344439357957027, 1.4, 1.4360679774997898)

# coords_obs = [(-0.4,3), (-0.4, 4.2), (2, 4.2), (2,3)]
current_pose = (0, -1, 0)

# position of object (x,y,z)
target_xy = (0, 1, 1.4)

# for roadmap
bloat_factor = 0.5
# for sampling
stdev_scale = 0.75
arm_offset = [-0.052, 0.013, 0]   # measured from center of base
# arm_offset = [0.1, 0, 0]
n_iter = 1000

# for checking retraction
# current_pose = (5.720954979290964, 0.7444603659236217, -2.944127486320978)

waypoints = generate_waypoints.generate_full_traj(stretch, objects_mesh,current_pose, target_xy, table_coords, table_origin, table_theta, bloat_factor, stdev_scale, arm_offset, n_iter)
print(waypoints)

# plt.close()

show_animation = True
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

    # arm_pose = waypoints[-1]
    # trajectory
    for wp in waypoints:
        counter += 1
        if wp[:2] != current_pose[:2]:
            if np.isnan(wp[2]):   # base waypoints
                plt.plot(wp[0],wp[1],'g*')
            else:
                armx = arm_pose[0]+(wp[4] + stretch.defaultArmDepth)*math.cos(arm_pose[2])
                army = arm_pose[1]+(wp[4] + stretch.defaultArmDepth)*math.sin(arm_pose[2])
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


# // "W" : 1.52,
# // "D" : 0.76,
# // "H" : 0.2
