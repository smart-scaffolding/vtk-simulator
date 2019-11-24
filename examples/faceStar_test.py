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
    endFace = BlockFace(7, 1, 3, 'top')


    ik_motion, path = follow_path(robot, num_steps, offset=0.5, startFace=startFace, endFace=endFace, blueprint=blueprint)


    robot = model.Puma560_TEST(base=np.matrix([[1, 0, 0, 0.5],
                                               [0, 1, 0, 1.5],
                                               [0, 0, 1, 1],
                                               [0, 0, 0, 1]]), blueprint=blueprint)

    robot.animate(stances=ik_motion, frame_rate=30, unit='deg', num_steps=num_steps * 3, display_path=False, obstacles=[[num_steps*9, (5, 0, 0)], [num_steps*15, (6, 0, 0)]], path=path)

def move_to_point(point, robot, num_steps, previous_angles=None, flip_angles=False):
    # print(point)
    ik_angles = robot.ikineConstrained(point, flipped=flip_angles) * 180 / np.pi ## converted to degrees
    # print(ik_angles)

    if previous_angles is None:
        previous_angles = [1] * robot.length

    else:
        print("Previous Angles: {}".format(previous_angles))
    forward_1 = np.transpose(np.asmatrix(np.linspace(float(previous_angles[0]), ik_angles[0], num_steps)))
    forward_2 = np.transpose(np.asmatrix(np.linspace(float(previous_angles[1]), ik_angles[1], num_steps)))
    forward_3 = np.transpose(np.asmatrix(np.linspace(float(previous_angles[2]), ik_angles[2], num_steps)))
    forward_4 = np.transpose(np.asmatrix(np.linspace(float(previous_angles[3]), ik_angles[3], num_steps)))
    ik_test = np.concatenate((forward_1, forward_2, forward_3, forward_4), axis=1)
    # if flip_angles:
    #     ik_test = np.concatenate((forward_1, forward_4, forward_3, forward_2), axis=1)
    robot.update_angles(ik_test[-1].flatten().tolist()[0], unit="deg")
    return ik_test

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
            filtered_path.append(point)
        else:
            filtered_path.append(path[index-1])

    path = filtered_path
    save_path = None
    for index, item in enumerate(path):
        # move_ee_up = robot.end_effector_position().flatten().tolist()[0]
        direction = item[-1]
        point = list(item[0:3])
        point[0] = item[0] + 0.5
        point[1] = item[1] + 0.5
        point[2] = item[2] + 1
        move_ee_up = np.copy(path[index-1])
        # print(move_ee_up)
        # TODO: FIX TO ACCEPT ANY ORIENTATION, NOT JUST +Z
        move_ee_up[2] = float(move_ee_up[2]) + offset


        if index == 0:
            move_up = list(point)
            # TODO: FIX TO ACCEPT ANY ORIENTATION, NOT JUST +Z
            move_up[2] = move_up[2] + offset
            previous_angles_1 = move_to_point(move_up, robot, num_steps)

            previous_angles_2 = move_to_point(point, robot, num_steps, previous_angles_1[-1].flatten().tolist()[0])
            previous_angles_3 = move_to_point(point, robot, num_steps, previous_angles_2[-1].flatten().tolist()[0])




        else:
            ee_pos = robot.end_effector_position()
            initial_angles = previous_angles_3[-1].flatten().tolist()[0]
            ee_pos = ee_pos.tolist()[0]
            ee_pos[0] = math.floor(ee_pos[0]) + 0.5
            ee_pos[2] = round(ee_pos[2])
            if (index) % 2 == 0:
                new_base = tr.trotz(0, unit="deg", xyz=ee_pos)
                temp = initial_angles[1]
                initial_angles[1] = 180 / 2 + initial_angles[3]
                initial_angles[3] = temp - 180 / 2
                flip_angles = False

                # global_path.append((num_steps * index, path[index:]))
            else:
                new_base = tr.trotz(180, unit="deg", xyz=ee_pos)
                temp = initial_angles[1]
                initial_angles[1] = 180 / 2 + initial_angles[3]
                initial_angles[3] = temp - 180 / 2
                flip_angles = True



            robot.base = new_base
            robot.update_angles(initial_angles, unit="deg")






            print("Initial Angles: {}".format(initial_angles))
            ee_pos = robot.end_effector_position(np.asarray(initial_angles) * np.pi/180)


            # ee_pos = robot.end_effector_position()
            ee_up = np.copy(ee_pos).tolist()[0]
            ee_up[0] = math.floor(ee_up[0]) + 0.5
            ee_up[2] = round(ee_up[2])
            print("\t\t\tGoing to point: {}\t EE Pos: {}\tRounded Pos: {}".format(point, ee_pos, ee_up))
            # TODO: FIX TO ACCEPT ANY ORIENTATION, NOT JUST +Z
            ee_up[2] = ee_up[2] + offset

            previous_angles_1 = move_to_point(ee_up, robot, num_steps, initial_angles, flip_angles=flip_angles)

            stop_above = np.copy(point)
            # TODO: FIX TO ACCEPT ANY ORIENTATION, NOT JUST +Z
            stop_above[2] = stop_above[2] + offset
            previous_angles_2 = move_to_point(stop_above, robot, num_steps, previous_angles_1[-1].flatten().tolist()[0], flip_angles=flip_angles)

            previous_angles_3 = move_to_point(point, robot, num_steps, previous_angles_2[-1].flatten().tolist()[0], flip_angles=flip_angles)
            # robot.update_angles(previous_angles_3[-1].flatten().tolist()[0], unit="rad")
        save_path = update_path(save_path, previous_angles_1, previous_angles_2, previous_angles_3)

    return save_path, global_path

def update_path(save_path, new_motion_1, new_motion_2, new_motion_3):
    if save_path is None:
        return np.concatenate((new_motion_1, new_motion_2, new_motion_3))
    return np.concatenate((save_path, new_motion_1, new_motion_2, new_motion_3))

if __name__ == '__main__':
    main()
