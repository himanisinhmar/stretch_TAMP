import numpy as np
import math

def global2robot(pose, xyG):
    # pose - position of the robot
    # xyG - 2d point in the global coordinate system

    # output xyR - 2d point in the robot frame
    T_GR = np.array(
        [[np.cos(pose[2]), -np.sin(pose[2]), pose[0]], [np.sin(pose[2]), np.cos(pose[2]), pose[1]], [0, 0, 1]])
    xyR = np.matmul(np.linalg.inv(T_GR), np.hstack((xyG, 1)))

    return xyR[0:2]

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians

def feedbackLin(theta,vx,vy):
    epsilon = 0.05
    R = np.array([[np.cos(theta),np.sin(theta)],
                [-np.sin(theta),np.cos(theta)]])
    dirVel = np.array([[vx],[vy]])
    Rot = np.array([[1,0],[0,1/epsilon]])
    commands = np.matmul(np.matmul(Rot, R),dirVel)

    return commands[0], commands[1]


def limitCmds(fwdVel, angVel, wheel2Center, maxV):
    wheelVel1 = fwdVel + wheel2Center * angVel  # v = R * omega
    wheelVel2 = fwdVel - wheel2Center * angVel

    wheelVels = [wheelVel1, wheelVel2]
    wheelVelmax = max(abs(wheelVel1), abs(wheelVel2))
    # check if saturated
    if wheelVelmax > maxV:
        wheelVels = [x * maxV / wheelVelmax for x in wheelVels]
    # parse out forward and ang velocities
    cmdV = sum(wheelVels) / 2
    cmdW = (wheelVels[0] - wheelVels[1]) / (2 * wheel2Center)

    # if cmdW != 0:
    #     r = cmdV/cmdW
    # else:
    #     r = 0
    # vr = cmdW*(r + wheel2Center/2)
    # vl = cmdW*(r - wheel2Center/2)
    return cmdV, cmdW

def robot2Global(pose,xyR,type):
    T_RG = np.array(
        [[np.cos(pose[2]), -np.sin(pose[2]), pose[0]], [np.sin(pose[2]), np.cos(pose[2]), pose[1]], [0, 0, 1]])
    xyG =np.matmul(T_RG,np.hstack((xyR,1)))
    if type == 'lis':
        return [xyG[0],xyG[1]]
    else:
        return (xyG[0],xyG[1])