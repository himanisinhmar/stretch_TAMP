import numpy as np
import time
def grip(robInfo, retractInit,foundW,currentCommand, waypoints,ind):
    if not retractInit:
        gripPower = robInfo.robot.end_of_arm.status['stretch_gripper']['pos']
        diff = np.abs(gripPower + 4)
        if diff > 0.5:
            robInfo.robot.end_of_arm.move_to('stretch_gripper', -50)
            robInfo.robot.push_command()
            print('attempting grip')
            time.sleep(2)
        else:
            print('gripped')
            retractInit = 1
    if retractInit and foundW:
        print('attempting to find retraction path')
        current_pos = robInfo.pos[0][0:2] + [robInfo.pos[0][-1]]
        tableCoords = robInfo.getTableCoords()
        indOfPoint = next((idx, obj) for idx, obj in
                          enumerate(robInfo.trackableObjects) if obj in currentCommand[-1])
        posOfI = robInfo.pos[indOfPoint[0] + 1][0:3]
        waypoints = robInfo.getWaypoints(current_pos, tableCoords, posOfI, True)
        print(waypoints)
        foundW = 0
    elif retractInit:
        goalPoint = waypoints[-1]
        armHeight = robInfo.robot.lift.status['pos']
        diff = np.abs(armHeight - goalPoint[3])
        if diff > 0.01:
            print('-------------------------------')
            print('Action: Matching Height for retraction')
    
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
            print('Action: Matching extension for retraction')

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
                ind += 1
                print('Grip successful (maybe)')

    return retractInit, foundW, waypoints,ind