#!/usr/bin/env python3

"""
Description: Given a high level plan in the form of "sas_plan", execute each action in order. Connect to Optitrack
topics and the stretch robot to send commands.

Author: David Gundana

"""

import trimesh
import numpy as np
import os
import json
import scipy #need the graphics engine from scipy to show objects

def getScene(objects,objectPos,envJSON):
    """
    Given:
            a list of object names, object positions, and environment object descriptions
            objects: 1xN list of object names
            objectPos: Nx6 poisiton vector of each object [x, y, z, yaw, pitch, roll]
    :return:
            scene of objects and all objects
    """

    # Create a scene for visualization
    scene = trimesh.Scene()
    allObjects = envJSON['objects']
    origin, xaxis, yaxis, zaxis = [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]
    outputObjects = []
    outputAxis = []
    for i in range(np.size(objects)):
        objInfo = allObjects[objects[i]]
        try:
            typeOfObj = objInfo['type']
        except Exception as e:
            print('Missing type of object')
            continue
        # Create a mesh depending on the object type
        if typeOfObj == 'box':
            item = trimesh.creation.box(extents=[objInfo['dimensions']['W'], objInfo['dimensions']['D'],
                                                objInfo['dimensions']['H']])
            # Not really needed but could be good for visualization
            item.visual.face_colors = [0.5, 0.5, 0.5, 0.5]
        elif typeOfObj == 'annulus':
            item = trimesh.creation.annulus(r_min=objInfo['dimensions']['r_in'],r_max=objInfo['dimensions']['r_out'],
                                            height=objInfo['dimensions']['H'])
            # Not really needed but could be good for visualization
            item.visual.face_colors = [0, 1., 0, 0.5]
        # Create an axis for visualization purposes
        axis = trimesh.creation.axis(origin_color=[1., 0, 0])
        # Translate the object by its position in the environment
        translation = objectPos[i,0:3]
        # Translate the box and the axis by the position
        item.apply_translation(translation)
        axis.apply_translation(translation)
        Rx = trimesh.transformations.rotation_matrix(objectPos[i,3], xaxis,point=item.centroid)
        Ry = trimesh.transformations.rotation_matrix(objectPos[i,4], yaxis,point=item.centroid)
        Rz = trimesh.transformations.rotation_matrix(objectPos[i,5], zaxis,point=item.centroid)
        R = trimesh.transformations.concatenate_matrices(Rx, Ry, Rz)
        item.apply_transform(R)
        axis.apply_transform(R)
        # Add the object to the scene
        scene.add_geometry(item)
        scene.add_geometry(axis)

        outputObjects.append(item)
        outputAxis.append(axis)
    # scene.show()

    return scene, outputObjects, outputAxis

if __name__ == "__main__":
    # These are the objects in the scene
    objects = ['table','pot','bowl','ladle']

    #position of the objects
    objectPos = np.array([[1, 1, 1, 0,0,0],[0, 1, 1.4, np.pi,0,0],[2, 0.5, 1.2, 0,0,0],[0, 0, 1.25, np.pi,0,0]])

    #Environemnt file with objects
    filePath = os.path.normpath(os.getcwd() + os.sep + os.pardir)
    envJSONPath = os.path.join(filePath, 'highlevel_planner','environment.json')
    f = open(envJSONPath)
    envJSON = json.load(f)
    getScene(objects,objectPos,envJSON)