import trimesh
import numpy as np
import math
import matplotlib.pyplot as plt

"""
Description: Given the 3D goal point, 2D location point of robot's base 
and the theta direction of approach, this functions returns if the goal point is reachable without collisions.
If reachable, it also outputs the required arm extension depth, d and height of the arm, z.
this function is assuming that z = 0 at the base of the robot

EE_obj gives info of the object that end effector is holding currently
EE_obj contains current coordinates of new end effector tip along with largest radius bounding the object at tip
EE_obj = [<tip_x>,<tip_y>,<tip_z>,<radius>]
	
EE_loc contains current x,y,z coordinates of gripper

ASSUMING THAT EE_obj[2] IS ALWAYS LESS THAN EE_loc[2]

Author: Himani Sinhmar
"""

def find_z_d(goal, pose, basePose,stepSize_z, stretch, objects_mesh, EE_obj, EE_loc, retract):
	# breakpoint()
	if retract:
		traj = find_z_d_retract(goal, pose,basePose, stepSize_z, stretch, objects_mesh, EE_obj, EE_loc)
		if not traj:
			goodPoint = False
		else:
			goodPoint = True
		# return goodPoint, traj
	else:
		if not EE_obj:
			EE_obj = EE_loc
			EE_obj.append(0)
		# 	goodPoint,traj = find_z_d_EE_notExtended(goal,pose,basePose,stepSize_z,stretch,objects_mesh, EE_obj, EE_loc)
		# else:
		# 	goodPoint,traj = find_z_d_EE_extended(goal, pose, basePose,stepSize_z, stretch, objects_mesh, EE_obj, EE_loc)

		goodPoint,traj = find_z_d_EE_extended(goal, pose, basePose,stepSize_z, stretch, objects_mesh, EE_obj, EE_loc)
	# traj.append(pose)

	return goodPoint,traj

def find_rays(EE_obj,theta,final_d,EE_loc,z_Check,stretch):
	if EE_obj:
		# check collision with 3 end points 
		rayOrigin_ee = np.array([[EE_obj[0] + EE_obj[3]*np.cos(theta),EE_obj[1] + EE_obj[3]*np.cos(theta),EE_obj[2]],
			[EE_obj[0] + EE_obj[3]*np.cos(theta + np.pi/2),EE_obj[1] + EE_obj[3]*np.cos(theta + np.pi/2),EE_obj[2]], 
			[EE_obj[0] + EE_obj[3]*np.cos(theta + 3*np.pi/2),EE_obj[1] + EE_obj[3]*np.cos(theta + 3*np.pi/2),EE_obj[2]]])
		rayOrigin_ee = rayOrigin_ee.reshape(3,3)
		### find direction vector for the rays
		rayDirection_ee = np.array([[np.cos(theta),np.sin(theta),0],[np.cos(theta),np.sin(theta),0],[np.cos(theta),np.sin(theta),0]])
		rayDirection_ee = rayDirection_ee.reshape(3,3);
	else:
		# rayOrigin_ee = np.array([robotBaseLoc2D[0]+(stretch.defaultArmDepth+final_d)*np.cos(theta), robotBaseLoc2D[1]+(stretch.defaultArmDepth+final_d)*np.sin(theta), z_Check])
		# rayOrigin_ee = rayOrigin_ee.reshape(1,3);
		# ### find direction vector for the rays
		# rayDirection_ee = np.array([np.cos(theta),np.sin(theta),0]);
		# rayDirection_ee = rayDirection_ee.reshape(1,3);
		rayOrigin_ee = np.array([EE_loc[0]+(final_d)*np.cos(theta), EE_loc[1]+(final_d)*np.sin(theta), z_Check])
		rayOrigin_ee = rayOrigin_ee.reshape(1,3);
		### find direction vector for the rays
		rayDirection_ee = np.array([np.cos(theta),np.sin(theta),0]);
		rayDirection_ee = rayDirection_ee.reshape(1,3);

	return rayOrigin_ee,rayDirection_ee

def checkCollision_ray_Scene(ray_origins,ray_directions,objects_mesh):
	# col = False;
	colDist = 1000000.00;
	#  check if colliding with any of the object
	for i in range(np.size(objects_mesh)):
		mesh = objects_mesh[i];
		# run the mesh- ray query
		index_triangle, index_ray, locations = mesh.ray.intersects_id(ray_origins, ray_directions, 
					return_locations=True, multiple_hits=True)
		x1 = ray_origins[0][0]; y1 = ray_origins[0][1]; z1 = ray_origins[0][2];
		# check if the intersecting location is within the extended arm reach
		for h in range(len(locations)):
			x2 = locations[h][0]; y2 = locations[h][1]; z2 = locations[h][2];
			dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2);
			if dist < colDist:
				colDist = dist;
	return colDist;

def checkCollision_ray_Scene_EEextended(ray_origins,ray_directions,objects_mesh):
	# col = False;
	colDist = 1000000.00;
	#  check if colliding with any of the object
	for i in range(np.size(objects_mesh)):
		mesh = objects_mesh[i];
		# run the mesh- ray query
		index_triangle, index_ray, locations = mesh.ray.intersects_id(ray_origins, ray_directions, 
					return_locations=True, multiple_hits=True)
		# check if the intersecting location is within the extended arm reach
		for h in range(len(locations)):
			x1 = ray_origins[index_ray[h]][0]; y1 = ray_origins[index_ray[h]][1]; z1 = ray_origins[index_ray[h]][2];
			x2 = locations[h][0]; y2 = locations[h][1]; z2 = locations[h][2];
			dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2);
			if dist < colDist:
				colDist = dist;
	return colDist;

def find_z_d_retract(goal, pose, basePose,tepSize_z, stretch, objects_mesh, EE_obj, EE_loc):

	"""move the arm to the top and then set extension to be zero.
	"""
	# breakpoint()
	traj = []
	z_Check = stretch.totalArmHeight;
	if EE_obj:
		EE_obj[2] += z_Check - EE_loc[2]
		# z_change = EE_loc[2] - EE_obj[2] # difference in z position of object and gripper
		# EE_obj[2] = z_Check - z_change
	else:
		EE_obj = [EE_loc[0],EE_loc[1],z_Check,0]

	theta = pose[2]

	ray_origins,ray_directions = find_rays(EE_obj,theta,0,pose[0:1],z_Check,stretch)
	colDist = checkCollision_ray_Scene_EEextended(ray_origins,ray_directions,objects_mesh)
	if colDist < stretch.totalArmDepth:
		traj = []
	else:
		final_d = 0
		traj.append([basePose[0],basePose[1],basePose[2],z_Check,final_d])
	return traj

def find_z_d_EE_extended(goal, pose, basePose, stepSize_z, stretch, objects_mesh, EE_obj, EE_loc):
 
	"""
	EE_obj gives info of the object that end effector is holding currently
	EE_obj contains current coordinates of new end effector tip along with largest radius bounding the object at tip
	EE_obj = [<tip_x>,<tip_y>,<tip_z>,<radius>]
 	
 	EE_loc contains current x,y,z coordinates of gripper

 	ASSUMING THAT EE_obj[2] IS ALWAYS LESS THAN EE_loc[2]

	defaultArmHeight: coordinate in z axis of the arm in global frame
	defaultArmDepth: positive scalar value of the arm extension
	"""

	totalArmDepth = stretch.totalArmDepth;
	defaultArmDepth = stretch.defaultArmDepth;
	defaultArmHeight = stretch.defaultArmHeight;
	totalArmHeight = stretch.totalArmHeight;

	# robotBaseLoc2D = pose[0:2];
	theta = pose[2];
	# z_Check is the arm height
	
	
	reachedGoal = False;
	### flag for checking if z-space is explored entirely
	done = False
	### goodPoint if robot's arm able to reach in the vicnity (within closeEnough_3D) of the goal
	closeEnoughGoal_3D = 1e-3;

	### returns boolean goodPoint and the 5D traj ("waypoints")
	goodPoint = False;
	traj = [];
	# breakpoint()
	### initialize arm height and extension variables, initally arm depth is stowed (at default depth)
	d_Check = defaultArmDepth;
	### setting z the same height as goal point
	print("EE_obj initial = ",EE_obj)
	if goal[2] <= totalArmHeight:
		z_change = EE_loc[2] - EE_obj[2]
		EE_obj[2] = goal[2];
		z_Check =  EE_obj[2] + z_change
		print("difference in length of obj and arm = ",z_change)
		print("EE_obj after moving to goal = ",EE_obj)
		print("z_Check after moving to goal = ",z_Check)
	else:
		print("goal z is higher than the total arm height")
		return goodPoint,traj

	### variable to store how much total arm extension is required from default position
	final_d = 0; # initially arm is not extended

	### find rays originating at current position of the end effector
	rayOrigin_ee,rayDirection_ee = find_rays(EE_obj,theta,final_d,EE_loc,z_Check,stretch)
	# breakpoint()
	while not done:
		### at current z, compute distance of current position of the end effector to the goal in xy plane
		distToGoal_2d = math.sqrt((EE_obj[0]-goal[0])**2 + (EE_obj[1]-goal[1])**2);
		if distToGoal_2d <= totalArmDepth:
			d_extension_required = distToGoal_2d;
		else:
			break;

		### check if the 3D goal is accessible at current position
		checkGoalDist_3D = math.sqrt((EE_obj[0]-goal[0])**2 + (EE_obj[1]-goal[1])**2 + (EE_obj[2]-goal[2])**2);
		# if (checkGoalDist_3D <= closeEnoughGoal_3D) and (not col):
		if checkGoalDist_3D <= closeEnoughGoal_3D:
			reachedGoal = True;
			goodPoint = True;
			traj.append([basePose[0],basePose[1],basePose[2],z_Check,final_d]);
			break;
		
		### compute how much to extend arm by checking where the rays originating from current position of the end effector would hit with the closest object
		colDist = checkCollision_ray_Scene_EEextended(rayOrigin_ee,rayDirection_ee,objects_mesh);
		# print(colDist)

		if colDist > d_extension_required:
			d_Check = d_extension_required;
			### update total arm extension required
			final_d += d_Check; 

			EE_obj[0] += d_Check*np.cos(theta)
			EE_obj[1] += d_Check*np.sin(theta)

			traj.append([basePose[0],basePose[1],basePose[2],z_Check,final_d]);
			if EE_obj[2] == goal[2]:
				done = True
				goodPoint = True;
				reachedGoal = True;
				break
			else:
				# z_change = EE_obj[2] - goal[2]
				# z_Check =  EE_loc[2] - z_change
				EE_obj[2] = goal[2]
				z_Check = z_change + EE_obj[2]
		else:
			d_Check = colDist;
			### update total arm extension required
			final_d += d_Check; 
			### move the arm by stepSize_z
			EE_obj[2] += stepSize_z;
			# z_Check += stepSize_z;
			z_Check = z_change + EE_obj[2]
			EE_obj[0] += d_Check*np.cos(theta)
			EE_obj[1] += d_Check*np.sin(theta)

		print("EE_obj current = ",EE_obj)
		print("z_Check current = ",z_Check)
		if z_Check < defaultArmHeight:
			done = True;
			break;

		# check collision with 3 end points 
		rayOrigin_ee,rayDirection_ee = find_rays(EE_obj,theta,final_d,EE_loc,z_Check,stretch)

	if not goodPoint:
		traj = [];
	return goodPoint,traj

def find_z_d_EE_notExtended(goal,pose,basePose,stepSize_z,stretch,objects_mesh, EE_obj, EE_loc):
	### defaultArmHeight: coordinate in z axis of the arm in global frame
	### defaultArmDepth: positive scalar value of the arm extension

	totalArmDepth = stretch.totalArmDepth;
	defaultArmDepth = stretch.defaultArmDepth;
	defaultArmHeight = stretch.defaultArmHeight;
	totalArmHeight = stretch.totalArmHeight;

	# robotBaseLoc2D = pose[0:2];
	theta = pose[2];

	reachedGoal = False;
	### flag for checking if z-space is explored entirely
	done = False
	### goodPoint if robot's arm able to reach in the vicnity (within closeEnough_3D) of the goal
	closeEnoughGoal_3D = 1e-2;

	### returns boolean goodPoint and the 5D traj ("waypoints")
	goodPoint = False;
	traj = [];

	### initally both z and d are stowed
	d_Check = defaultArmDepth;
	### setting z the same height as goal point
	if goal[2] <= totalArmHeight:
		z_Check = goal[2];
	else:
		z_Check = defaultArmHeight; 
	### variable to store how much total arm extension is required from default position
	final_d = 0; # initially arm is not extended

	### find ray originating at current position of the end effector
	rayOrigin_ee,rayDirection_ee = find_rays(EE_obj,theta,final_d,EE_loc,z_Check,stretch)

	while not done:
		### at current z, compute distance of current position of the end effector to the goal in xy plane
		distToGoal_2d = math.sqrt((rayOrigin_ee[0][0]-goal[0])**2 + (rayOrigin_ee[0][1]-goal[1])**2);
		# breakpoint()
		if distToGoal_2d <= totalArmDepth:
			d_extension_required = distToGoal_2d;
		else:
			break;

		### check if the 3D goal is accessible at current position
		checkGoalDist_3D = math.sqrt((rayOrigin_ee[0][0]-goal[0])**2 + (rayOrigin_ee[0][1]-goal[1])**2 + (rayOrigin_ee[0][2]-goal[2])**2);
		# if (checkGoalDist_3D <= closeEnoughGoal_3D) and (not col):
		if checkGoalDist_3D <= closeEnoughGoal_3D:
			reachedGoal = True;
			goodPoint = True;
			traj.append([basePose[0],basePose[1],basePose[2],z_Check,final_d]);
			break;
		
		### compute how much to extend arm by checking where the ray originating from current position of the end effector would hit with the closest object
		colDist = checkCollision_ray_Scene(rayOrigin_ee,rayDirection_ee,objects_mesh);
		# print(colDist)

		if colDist > d_extension_required:
			d_Check = d_extension_required;
			### update total arm extension required
			final_d += d_Check; 
			traj.append([basePose[0],basePose[1],basePose[2],z_Check,final_d]);
			if z_Check == goal[2]:
				done = True
				goodPoint = True;
				reachedGoal = True;
				break
			else:
				z_Check = goal[2];
			# z_Check = goal[2];
		else:
			d_Check = colDist;
			### update total arm extension required
			final_d += d_Check; 
			### move the arm up by stepSize_z
			z_Check += stepSize_z;

		if z_Check > totalArmHeight:
			done = True;
			break;

		### compute new end effector location 
		rayOrigin_ee,rayDirection_ee = find_rays(EE_obj,theta,final_d,EE_loc,z_Check,stretch)

	if not goodPoint:
		traj = [];
	return goodPoint,traj