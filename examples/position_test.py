import serial
import time
import robopy.base.model as model
import numpy as np
from faceStar_test import *

block_size = 3.0


def position_test():
    blueprint = np.array([
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
    ])

    robot = model.Inchworm(base=np.matrix([[1, 0, 0, 0.5],
                                               [0, 1, 0, 0.5],
                                               [0, 0, 1, 1],
                                               [0, 0, 0, 1]]), blueprint=blueprint, port='/dev/cu.usbmodem14101')



    time.sleep(2)

    q0 = np.array([0, np.pi/2, 0, 0])
    base_pos = np.array([0., 0., 1.])



    angles = []
    q1 = [0., 90, 0, 0]
    # q1 = [0, 70, -110, -20]
    q1 = [0, 90, -90, -90]
    q1 = np.array([0, 1.47977127e+00, -2.03224729e+00, -1.01750194e+00]) * 180 / np.pi

    qUp = np.array([0, 1.47977127e+00, -2.03224729e+00, -1.01750194e+00]) * 180 / np.pi  # up
    qOver = np.array([0, 1.05228649e+00, -1.46107186e+00, -1.16119259e+00]) * 180 / np.pi  # over
    qDown = np.array([0, 4.50414252e+01, -9.00828504e+01, -4.49116859e+01]) #down
    # q2 = [0, np.pi/2, 0, 0] #180 degrees
    # q3 = [0, np.pi/3, 0, 0] #90 degrees
    # # qFow = np.array([0, 65, 36, 60]) #90 degrees
    # qFow = np.array([0, 65, -124, -30]) # supposed to be:
    # qFow = qFow / 180.0 * np.pi
    # qBack = np.array([0, 49, -91, -48]) #90 degrees
    # qBack = qBack / 180.0 * np.pi

    # angles.append(qBack)
    # angles.append(qFow)
    #
    # angles.append(q3)
    # angles.append(q2)
    # angles.append(q1)
    angles.append(qDown)
    angles.append(qOver)
    angles.append(qUp)


    while len(angles) > 0:
        curr_angle = angles.pop()
        print("Current Angle: {}".format(curr_angle))
        robot.send_to_robot(curr_angle, 2.0)
        robot.map_angles_to_robot(curr_angle)
        # robot.plot(np.asmatrix(curr_angle), unit="deg")

#         time.sleep(2.0)
#
# def send_angles(q):
#     qTemp = np.array([q[0] + np.pi, q[1] + (np.pi / 2), q[2] + np.pi, q[3] + np.pi])
#     qTemp = qTemp * 180.0 / np.pi  # convert to degrees
#     print("Final Angles: {}".format(qTemp[1:]))
#     targetAngles = str(int(qTemp[1])).zfill(3) + str(int(qTemp[2])).zfill(3) + str(int(qTemp[3])).zfill(3)
#     return targetAngles


if __name__ == '__main__':
    position_test()