import numpy as np
import findTraj
import time
import math
from helperFuncs import global2robot, robot2Global, feedbackLin, limitCmds

def gofromto(robInfo, currentCommand, foundWaypoints, isAligned,ind,waypoints, indWay):
    # Go to a point
    retractInit, foundW = 1, 0
    tableCoords = robInfo.getTableCoords()
    # Find point from the label of the object
    indOfPoint = next((idx, obj) for idx, obj in
                      enumerate(robInfo.trackableObjects) if obj in currentCommand[-1])
    posOfI = robInfo.pos[indOfPoint[0] + 1][0:3]
    if not foundWaypoints:
        current_pos = robInfo.pos[0][0:2] + [robInfo.pos[0][-1]]

        if len(posOfI) == 0:
            print('{} is not in the environment'.format(indOfPoint[1]))
        else:
            waypoints = robInfo.getWaypoints(current_pos, tableCoords, posOfI, False)
            foundWaypoints = 1
            print(waypoints)
            indWay = 0

    goalPoint = waypoints[indWay]
    smartTraj = 0
    if smartTraj:
        goalPoint = waypoints[indWay]
        goalPoint = [.5, -1, 160 * np.pi / 180]
        errorX = goalPoint[0] - robInfo.pos[0][0]
        errorY = goalPoint[1] - robInfo.pos[0][1]
        theta = robInfo.pos[0][-1]
        thetaGoal = goalPoint[2]
        rho, cmdV, cmdW = findTraj.calc_control_command(errorX, errorY, theta, thetaGoal)
        # cmdV, cmdW = robInfo.limitCmds(v, w)
        print('-------------------------------')
        print('Action: Navigating')

        print('posX: {}, posY: {}, posTheta: {}'.format(round(robInfo.pos[0][0], 2),
                                                        round(robInfo.pos[0][1], 2),
                                                        round(robInfo.pos[0][-1], 2)))
        print('Command V: {}, command W: {}, dist2goal {}'.format(round(cmdV, 2),
                                                                  round(cmdW, 2)
                                                                  , round(rho, 2)))
        print('Waypoint: {}'.format(goalPoint))
        print('-------------------------------\n')
        robInfo.robot.base.set_velocity(v_m=cmdV, w_r=cmdW)
        robInfo.robot.push_command()
    else:
        if math.isnan(goalPoint[2]):
            # navigating to point
            errorX = robInfo.pos[0][0] - goalPoint[0]
            errorY = robInfo.pos[0][1] - goalPoint[1]
            distToGoal = np.sqrt(errorX ** 2 + errorY ** 2)
            if distToGoal < robInfo.closeEnough:
                indWay += 1
                robInfo.robot.base.set_velocity(v_m=0, w_r=0)
                robInfo.robot.push_command()
                print('Going to next point...')
            v, omega = feedbackLin(robInfo.pos[0][-1], errorX, errorY)
            cmdV, cmdW = limitCmds(v, omega, robInfo.wheel2Center, robInfo.maxV)
            print('-------------------------------')
            print('Action: Navigating')

            print('posX: {}, posY: {}, posTheta: {}'.format(round(robInfo.pos[0][0], 2),
                                                            round(robInfo.pos[0][1], 2),
                                                            round(robInfo.pos[0][-1], 2)))
            print('Command V: {}, command W: {}, dist to goal {}'.format(round(cmdV[0], 2),
                                                                         round(cmdW[0], 2)
                                                                         , round(distToGoal, 2)))
            print('Waypoint: {}'.format(goalPoint[0:3]))
            print('-------------------------------\n')

            robInfo.robot.base.set_velocity(v_m=-cmdV[0], w_r=-cmdW[0])
            robInfo.robot.push_command()
        else:
            if not isAligned:
                direc = -1
                if goalPoint[2] - robInfo.pos[0][-1] > 0:
                    direc = 1
                diff = np.abs(robInfo.pos[0][-1] - goalPoint[2])

                print('-------------------------------')
                print('Action: Aligning')

                print('posX: {}, posY: {}, posTheta: {}'.format(round(robInfo.pos[0][0], 2),
                                                                round(robInfo.pos[0][1], 2),
                                                                round(robInfo.pos[0][-1], 2)))
                print('angle difference {}'.format(round(diff, 2)))
                print('Waypoint: {}'.format(goalPoint[3:]))
                print('-------------------------------\n')

                if diff > robInfo.closeAngEnough:
                    vr = .2 * direc
                    robInfo.robot.base.set_rotational_velocity(vr)
                    robInfo.robot.push_command()
                else:
                    robInfo.robot.base.set_rotational_velocity(0)
                    robInfo.robot.push_command()
                    indOfPoint = next((idx, obj) for idx, obj in
                                      enumerate(robInfo.trackableObjects) if obj in currentCommand[-1])
                    xyR = global2robot([robInfo.pos[0][0], robInfo.pos[0][1], robInfo.pos[0][-1]], posOfI[0:2])
                    robInfo.robot.base.translate_by(xyR[0] - robInfo.arm_offset[0])
                    robInfo.robot.end_of_arm.move_to('stretch_gripper', 50)
                    robInfo.robot.push_command()
                    time.sleep(1)
                    newDiff = np.abs(xyR[0] - robInfo.arm_offset[0])
                    print('-------------------------------')
                    print('Action: Fixing Error in robot x')

                    print('posX: {}, posY: {}, posTheta: {}'.format(round(robInfo.pos[0][0], 2),
                                                                    round(robInfo.pos[0][1], 2),
                                                                    round(robInfo.pos[0][-1], 2)))
                    print('x difference {}'.format(round(newDiff, 2)))
                    print('-------------------------------\n')
                    if newDiff < .1:
                        print('Finished Aligning')
                        print('Getting new z and d')
                        current_pos = robInfo.pos[0][0:2] + [robInfo.pos[0][-1]]
                        waypointsNew = robInfo.getWaypoints(current_pos, tableCoords, posOfI, False)
                        waypointsNew = waypointsNew[-1]
                        goalPoint[4] = waypointsNew[4]
                        isAligned = 1
            else:
                armHeight = robInfo.robot.lift.status['pos']
                diff = np.abs(armHeight - goalPoint[3])

                if diff > 0.01:
                    print('-------------------------------')
                    print('Action: Matching Height')

                    print('posX: {}, posY: {}, posTheta: {}, armHeight: {}'.format(round(robInfo.pos[0][0], 2),
                                                                                   round(robInfo.pos[0][1], 2),
                                                                                   round(robInfo.pos[0][-1], 2),
                                                                                   round(armHeight, 2)))
                    print('height difference {}'.format(round(diff, 2)))
                    print('Waypoint: {}'.format(goalPoint[0:4]))
                    print('-------------------------------\n')
                    robInfo.robot.lift.move_to(x_m=goalPoint[3])
                    robInfo.robot.push_command()
                    # Should be changed to an actual check
                    time.sleep(3)
                else:
                    armExtend = robInfo.robot.arm.status['pos']
                    diff = armExtend - goalPoint[4]
                    print('-------------------------------')
                    print('Action: Extending')

                    print('posX: {}, posY: {}, posTheta: {}, Arm Depth: {}'.format(round(robInfo.pos[0][0], 2),
                                                                                   round(robInfo.pos[0][1], 2),
                                                                                   round(robInfo.pos[0][-1], 2),
                                                                                   round(armHeight, 2)))
                    print('extend difference {}'.format(round(diff, 2)))
                    print('Waypoint: {}'.format(goalPoint[4:]))
                    print('-------------------------------\n')
                    robInfo.robot.arm.move_to(goalPoint[-1])
                    robInfo.robot.push_command()
                    time.sleep(3)
                    if diff < 0.01:
                        indWay += 1
                        if indWay >= np.size(waypoints, 0):
                            current_pos = robInfo.pos[0][0:2] + [robInfo.pos[0][-1]]
                            gripper_posR = robInfo.arm_offset[0:2]
                            gripper_posR[1] = robInfo.arm_offset[1] - robInfo.defaultArmDepth - robInfo.robot.arm.status['pos']
                            objPos = global2robot(current_pos, posOfI[0:2])
                            dDist = gripper_posR[1] - objPos[1]
                            print('gripper pose: {}, ladlepos: {}'.format(gripper_posR, objPos))
                            robInfo.robot.arm.move_by(dDist)
                            robInfo.robot.push_command()
                            time.sleep(2)
                            errorX = robInfo.pos[0][0] - goalPoint[0]
                            errorY = robInfo.pos[0][1] - goalPoint[1]
                            distToGoal = np.sqrt(errorX ** 2 + errorY ** 2)
                            thetaError = robInfo.pos[0][-1] - goalPoint[2]
                            zError = robInfo.robot.lift.status['pos'] - goalPoint[3]
                            dError = robInfo.robot.arm.status['pos'] - goalPoint[4]
                            # print('errorX: {}, errorY: {}, errorDist: {}, thetaError: {}, zError: {}, dError: {}'
                            #       .format(errorX, errorY, distToGoal, thetaError, zError, dError))
                            # print('robot pos: {}'.format(robInfo.pos[0]))
                            # print('ladle pos: {}'.format(posOfI))
                            # print('waypoint: {}'.format(waypoints))
                            # print('table pos: {}'.format(robInfo.pos[1]))
                            print('gofromto Command complete')
                            retractInit = 0
                            foundWaypoints = 0
                            foundW = 1
                            ind += 1

    return foundWaypoints, isAligned, retractInit, foundW, ind, waypoints, indWay