from serial import Serial, PARITY_NONE, STOPBITS_ONE, EIGHTBITS
import threading
import numpy as np
import time


accuracy = 1e-7
threshold = 1
animate = False
delay = 0.5
num_steps = 30 #ol
use_serial = True
baud=9600
port='/dev/cu.usbmodem14201'


robot_serial = Serial(port=port, baudrate=baud, parity=PARITY_NONE,
                                    stopbits=STOPBITS_ONE, bytesize=EIGHTBITS, timeout=3.0)


def read_angles_from_robot(read_serial):
    print("Ready to read angles\n")
    while True:
        if read_serial.in_waiting > 0:
            line = read_serial.readline()
            print(f"[ROBOT]: {line.decode()}")

def send_angles_to_robot(angles):
    num_angles = len(angles)
    for index, angle in enumerate(angles):
        target_angles = map_angles_to_robot(angle, index, num_angles)
        robot_serial.write(target_angles)
        time.sleep(delay)


def map_angles_to_robot(q, index, num_angles, open_gripper=None):
    qTemp = q
    # qTemp = np.array([q[0], 90 - q[1], q[2] * -1, q[3] * -1])
    # print("Final Angles: {}".format(qTemp[1:]))

    gripper = "0100"
    if open_gripper:
        gripper = "00" + open_gripper

    targetAngles = f'{qTemp[0]:4.2f} '.zfill(8) + f'{qTemp[1]:4.2f} '.zfill(8) + f'{qTemp[2]:4.2f} '.zfill(8) + \
                   gripper + '\n'

    print("-" * 40)
    print(f"[SENDING]: {targetAngles}")
    print(f"Angle: {index+1} / {num_angles}")
    print("-" * 40)
    return str.encode(targetAngles)

if __name__ == '__main__':
    reading = threading.Thread(target=read_angles_from_robot, args=(robot_serial,))
    reading.start()

    time.sleep(3)
    """
    NOTE: Angles should already be mapped to Arduino space, do not use the immediate output from move_robot_test
    """

    angles_to_send = [
        [29.71, 83.71, 66.53]
    ]
    send_angles_to_robot(angles_to_send)
    time.sleep(2)

    while True:
        angles_to_send = [
            # [5.21, 116.44, 58.30],
            [29.71, 83.71, 66.53],
            [31.71, 85.71, 68.53],
            [33.71, 87.71, 70.53],
            [31.71, 85.71, 68.53],
            # [44.96, 90.08, 44.91],
        ]
        send_angles_to_robot(angles_to_send)