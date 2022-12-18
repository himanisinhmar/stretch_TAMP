import math
import numpy as np
from helperFuncs import feedbackLin, limitCmds

def move(robInfo,currentCommand, foundWaypoints, waypoints,indWay):
    indOfPoint = next((idx, obj) for idx, obj in
                      enumerate(robInfo.trackableObjects) if obj in currentCommand[-1])
    posOfI = robInfo.pos[indOfPoint[0] + 1][0:3]
    tableCoords = robInfo.getTableCoords()

    if not foundWaypoints:
        current_pos = robInfo.pos[0][0:2] + [robInfo.pos[0][-1]]

        if len(posOfI) == 0:
            print('{} is not in the environment'.format(indOfPoint[1]))
        else:
            posOfI[-1] = .95
            print(posOfI[-1])
            waypoints = robInfo.getWaypoints(current_pos, tableCoords, posOfI, False)
            foundWaypoints = 1
            print(waypoints)
            indWay = 0

    goalPoint = waypoints[indWay]
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
        print('ready to align')

    return foundWaypoints, waypoints, indWay