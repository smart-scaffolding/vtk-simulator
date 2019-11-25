import serial
import time
import robopy.base.model as model
import numpy as np
from faceStar_test import *

block_size = 3.0


def arm_animation():
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

    robot = model.Puma560_TEST(base=np.matrix([[1, 0, 0, 0.5],
                                               [0, 1, 0, 1.5],
                                               [0, 0, 1, 1],
                                               [0, 0, 0, 1]]), blueprint=blueprint)


    ser = serial.Serial(port='/dev/cu.usbmodem14201', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS, timeout=3.0)

    time.sleep(2)

    q0 = np.array([0, np.pi/2, 0, 0])
    base_pos = np.array([0., 0., 1.])



    angles = []
    q1 = [0., 0, 0, 0]
    q2 = [0, np.pi/2, 0, 0] #180 degrees
    q3 = [0, np.pi/3, 0, 0] #90 degrees
    # qFow = np.array([0, 65, 36, 60]) #90 degrees
    qFow = np.array([0, 65, -124, -30]) # supposed to be:
    qFow = qFow / 180.0 * np.pi
    qBack = np.array([0, 49, -91, -48]) #90 degrees
    qBack = qBack / 180.0 * np.pi

    # angles.append(qBack)
    # angles.append(qFow)
    #
    # angles.append(q3)
    # angles.append(q2)
    # angles.append(q1)



    while len(angles) > 0:
        curr_angle = angles.pop()
        print("Current Angle: {}".format(curr_angle))
        targetAngles = send_angles(curr_angle)
        ser.write(targetAngles)

        # robot.plot(np.asmatrix([0, 7, -13, -85]), unit="deg") # stretched out
        # robot.plot(np.asmatrix([0, 50, -99, -40]), unit="deg")  # about one step between
        robot.plot(np.asmatrix(curr_angle), unit="deg")

        time.sleep(2.0)

def send_angles(q):
    qTemp = np.array([q[0] + np.pi, q[1] + (np.pi / 2), q[2] + np.pi, q[3] + np.pi])
    qTemp = qTemp * 180.0 / np.pi  # convert to degrees
    print("Final Angles: {}".format(qTemp[1:]))
    targetAngles = str(int(qTemp[1])).zfill(3) + str(int(qTemp[2])).zfill(3) + str(int(qTemp[3])).zfill(3)
    return targetAngles


if __name__ == '__main__':
    arm_animation()