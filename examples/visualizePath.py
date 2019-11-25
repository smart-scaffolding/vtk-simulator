import robopy.base.model as model
import numpy as np
import robopy.base.transforms as tr
import math
from robopy.base.FaceStar import *

def main():
    blueprint = np.array([
        [[1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 1, 1]],
        [[1, 0, 0], [1, 0, 0], [1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [1, 1, 0], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [1, 0, 0], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 1, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 1, 1], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
    ])

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

    num_steps = 10

    startFace = BlockFace(3, 1, 0, 'top')
    endFace = BlockFace(6, 1, 2, 'left')


    ik_motion, path = follow_path(robot, num_steps, offset=0.5, startFace=startFace, endFace=endFace, blueprint=blueprint)


    robot = model.Puma560_TEST(base=np.matrix([[1, 0, 0, 0.5],
                                               [0, 1, 0, 1.5],
                                               [0, 0, 1, 1],
                                               [0, 0, 0, 1]]), blueprint=blueprint)

    robot.animate(stances=ik_motion, frame_rate=30, unit='deg', num_steps=num_steps * 3, display_path=False, obstacles=[[num_steps*9, (5, 0, 0)], [num_steps*15, (6, 0, 0)]], path=path)



def follow_path(robot, num_steps, offset, startFace, endFace, blueprint):
    # path = [(3.5, 0.5, 1), (2.5, 0.5, 1), (3.5, 0.5, 1), (2.5, 0.5, 1), (3.5, 0.5, 1), (2.5, 0.5, 1), (3.5, 0.5, 1), (2.5, 0.5, 1)]
    # path = [(3.5, 1.5, 1), (1.5, 0.5, 1),(4.5, 0.5, 1), (2.5, 0.5, 1),(5.5, 0.5, 1), (3.5, 0.5, 1),(6.5, 0.5, 1), (4.5, 0.5, 1) ]
    # path = [(3.49, 1.49, 1), (1.49, 1.49, 1), (4.49, 1.49, 1), (2.49, 1.49, 1),(5.49, 1.49, 2),(4.5, 1.5, 1),(6.5, 1.5, 3),(5.5, 1.5, 2),
    #         (7.49, 1.49, 4),(6.5, 1.5, 3),(7.5, 2.5, 4), (7.5, 1.5, 4), (6.5, 2.5, 4), (7.5, 2.5, 4), (5.5, 2.5, 4),(6.5, 2.5, 4)]

    armReach = [2.38, 1.58]
    # armReach = [3, 3]



    faceStarPlanner = FaceStar(startFace, endFace, blueprint, armReach)
    path = faceStarPlanner.get_path()

    global_path = []
    global_path.append((num_steps, path))

    filtered_path = []
    for index, point in enumerate(path):
        filtered_path.append(point)
        if index == 0:
            filtered_path.append((1, 1, 0, "top"))
        else:
            filtered_path.append(path[index-1])

    path = filtered_path

    return path

def update_path(save_path, new_motion_1, new_motion_2, new_motion_3):
    if save_path is None:
        return np.concatenate((new_motion_1, new_motion_2, new_motion_3))
    return np.concatenate((save_path, new_motion_1, new_motion_2, new_motion_3))

def add_offset(ee_pos, direction, offset):
    if direction == "top":
        ee_pos[2] = float(ee_pos[2]) + offset

    if direction == "bottom":
        ee_pos[2] = float(ee_pos[2]) - offset

    if direction == "front":
        ee_pos[1] = float(ee_pos[1]) - offset

    if direction == "back":
        ee_pos[1] = float(ee_pos[1]) + offset

    if direction == "left":
        ee_pos[0] = float(ee_pos[0]) - offset

    if direction == "right":
        ee_pos[0] = float(ee_pos[0]) + offset


    return ee_pos

if __name__ == '__main__':
    main()
