#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May  9 17:34:20 2022

@author: cornell
"""

from pytransform3d.urdf import UrdfTransformManager


tm = UrdfTransformManager()

urdffile = 'stretch.urdf'
with open(urdffile) as f:
    urdf_str = f.read()

tm.load_urdf(urdf_str)

# tm.set_joint('base_link', angle)
tm.set_joint('joint_lift', 0.4) # this is the z value
tm.set_joint('joint_arm_l0', 0.0) # the sum of these 4 are the d value
tm.set_joint('joint_arm_l1', 0.13)
tm.set_joint('joint_arm_l2', 0.0)
tm.set_joint('joint_arm_l3', 0.12)

ee2object = tm.get_transform("base_link", "link_grasp_center")
print(ee2object)