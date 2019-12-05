import robopy.base.model as model
import numpy as np
import robopy.base.transforms as tr
import math
from robopy.base.FaceStar import *
from robopy.base.common import create_point_from_homogeneous_transform, flip_base, round_end_effector_position
import time
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
        [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
    ])

    # robot = model.Puma560_TEST(base=np.matrix([[1, 0, 0, 0.5],
    #                                           [0, 1, 0, 1.5],
    #                                           [0, 0, 1, 1],
    #                                           [0, 0, 0, 1]]), blueprint=blueprint)
    robot = model.Puma560_TEST(base=np.matrix([[1, 0, 0, 0.5],
                                               [0, 1, 0, 0.5],
                                               [0, 0, 1, 1.],
                                               [0, 0, 0, 1]]), blueprint=blueprint)

    num_steps = 40

    startFace = BlockFace(1, 1, 0, 'top')
    endFace = BlockFace(5, 1, 3, 'top')
    # endFace = BlockFace(4, 1, 2, 'top')
    # endFace = BlockFace(5, 0, 0, 'top')
    # endFace = BlockFace(3, 2, 3, "left")

    # (2, 1, 0, 'top'), (3, 1, 0, 'top'), (4, 1, 0, 'top'), (5, 1, 1, 'top'), (6, 1, 2, 'top'), (6, 2, 3, 'top')]
    ik_motion, path, directions = follow_path(robot, num_steps, offset=1, startFace=startFace, endFace=endFace, blueprint=blueprint)


    robot = model.Puma560_TEST(base=np.matrix([[1, 0, 0, 0.5],
                                               [0, 1, 0, 0.5],
                                               [0, 0, 1, 1.],
                                               [0, 0, 0, 1]]), blueprint=blueprint)

    robot_orientation = ["top"] * (len(path[0][1:][0]))
    print("Robot Orientation Original: {}".format(robot_orientation))
    print("Path Length: {}".format(len(path[0][1:][0])))
    print("Num Steps: {}".format(num_steps))
    modified_path = path[0][1:][0]
    for index, position in enumerate(modified_path):
        direction = position[-1]
        if index == 0:
            robot_orientation[index] = "top"
        else:
            previous_direction = modified_path[index - 1][-1]
            if previous_direction == direction:
                robot_orientation[index] = direction
            else:
                robot_orientation[index] = previous_direction

    print("Robot Orientation: {}".format(robot_orientation))

    robot.animate(stances=ik_motion, frame_rate=30, unit='deg', num_steps=num_steps * 3, display_path=False, obstacles=[[num_steps*9, (5, 0, 0)], [num_steps*15, (6, 0, 0)]], path=path, directions=directions, orientation=robot_orientation)

def move_to_point(direction, point, robot, num_steps, previous_angles=None, flip_angles=False):
    # print(point)
    try:
        ik_angles = robot.ikineConstrained(direction, point, flipped=flip_angles) * 180 / np.pi ## converted to degrees
        # print(ik_angles)

    except ValueError as e:
        print("\n\n")
        print("\t\t" + "*"*50)
        print("\t\tUNABLE TO REACH POINT")
        print("\t\t" + "*"*50)
        print("\t\tRobot Base: {}".format(np.transpose(create_point_from_homogeneous_transform(robot.base))))
        print("\t\tRobot EE Pos: {}".format(robot.end_effector_position()))
        print("\t\tRobot Angles: {}".format(robot.get_current_joint_config(unit="deg")))
        print("\t\tGoing to Point: {} \t Face: {}".format(point, direction))
        print("\t\t" + "*"*50)
        print("\n\n")
        time.sleep(0.1)
        raise e


    if previous_angles is None:
        previous_angles = [1] * robot.length

    # else:
    #     print("Previous Angles: {}".format(previous_angles))
    forward_1 = np.transpose(np.asmatrix(np.linspace(float(previous_angles[0]), ik_angles[0], num_steps)))
    forward_2 = np.transpose(np.asmatrix(np.linspace(float(previous_angles[1]), ik_angles[1], num_steps)))
    forward_3 = np.transpose(np.asmatrix(np.linspace(float(previous_angles[2]), ik_angles[2], num_steps)))
    forward_4 = np.transpose(np.asmatrix(np.linspace(float(previous_angles[3]), ik_angles[3], num_steps)))

    ik_test = np.concatenate((forward_1, forward_2, forward_3, forward_4), axis=1)
    # if flip_angles:
    #     ik_test = np.concatenate((forward_1, forward_4, forward_3, forward_2), axis=1)
    angle_update = ik_test[-1].flatten().tolist()[0]
    # angle_update = angle_update[0:3]
    robot.update_angles(angle_update, unit="deg")
    return ik_test

def follow_path(robot, num_steps, offset, startFace, endFace, blueprint):
    # path = [(3.5, 0.5, 1), (2.5, 0.5, 1), (3.5, 0.5, 1), (2.5, 0.5, 1), (3.5, 0.5, 1), (2.5, 0.5, 1), (3.5, 0.5, 1), (2.5, 0.5, 1)]
    # path = [(3.5, 1.5, 1), (1.5, 0.5, 1),(4.5, 0.5, 1), (2.5, 0.5, 1),(5.5, 0.5, 1), (3.5, 0.5, 1),(6.5, 0.5, 1), (4.5, 0.5, 1) ]
    # path = [(3.49, 1.49, 1), (1.49, 1.49, 1), (4.49, 1.49, 1), (2.49, 1.49, 1),(5.49, 1.49, 2),(4.5, 1.5, 1),(6.5, 1.5, 3),(5.5, 1.5, 2),
    #         (7.49, 1.49, 4),(6.5, 1.5, 3),(7.5, 2.5, 4), (7.5, 1.5, 4), (6.5, 2.5, 4), (7.5, 2.5, 4), (5.5, 2.5, 4),(6.5, 2.5, 4)]

    # path = [(2.49, 2.49, 1.3, "top"), (0.49, 2.49, 1.3, "top"),(2, 2.49, 2.5, "left")]
    path = [(1, 2, 0, "top"), (0, 2, 0, "top"), (3, 2, 3, "left"), (3, 2, 2, "left"), (3, 2, 5, "left")  ]
    # armReach = [2.38, 1.58]

    # armReach = [1.5, 1.5]

    armReach = [1.5, 1.5]



    # faceStarPlanner = FaceStar(startFace, endFace, blueprint, armReach)
    # path = faceStarPlanner.get_path()
    # #
    global_path = []
    global_path.append((num_steps, path))

    # filtered_path = []
    # for index, point in enumerate(path):
    #     filtered_path.append(point)
    #     if index == 0:
    #         filtered_path.append((0, 1, 0, "top"))
    #     else:
    #         filtered_path.append(path[index-1])
    #
    # filtered_path.append((6, 2, 3, "top"))
    # filtered_path.append((6, 1, 2, "top"))
    #
    # path = filtered_path

    global_direction = []


    save_path = None
    previous_point = None
    # previous_direction = "top"
    for index, item in enumerate(path):
        # move_ee_up = robot.end_effector_position().flatten().tolist()[0]

        direction = item[-1]
        if index == 0:
            global_direction.append((0, "top"))
            previous_direction = "top"
        else:
            global_direction.append((num_steps*index, path[index-1][-1]))
            previous_direction = path[index-1][-1]
            # previous_direction = direction

        point = list(item[0:3])
        if direction == "top" or direction =="bottom":
            point[0] = item[0] + 0.5
            point[1] = item[1] + 0.5
            point[2] = item[2] + 1
        if direction == "left" or direction =="right":
            point[0] = item[0] - 1.37
            point[1] = item[1] + 0.5
            point[2] = item[2] - 0.87
        if direction == "front" or direction =="back":
            point[0] = item[0] + 0.5
            point[1] = item[1] + 1
            point[2] = item[2] + 0.5
        # move_ee_up = np.copy(path[index-1])
        # print(move_ee_up)
        # TODO: FIX TO ACCEPT ANY ORIENTATION, NOT JUST +Z

        # move_ee_up[2] = add_offset(float(move_ee_up[2]) + offset
        # move_ee_up = add_offset(move_ee_up, direction, offset)

        print("\n\nPOINT: {}   DIRECTION: {}    PREVIOUS_DIRECTION: {}".format(point, direction, path[index-1][-1]))
        if index == 0:
            move_up = list(point)
            # TODO: FIX TO ACCEPT ANY ORIENTATION, NOT JUST +Z
            # move_up[2] = move_up[2] + offset

            move_up = add_offset(move_up, previous_direction, offset)


            previous_angles_1 = move_to_point(direction, move_up, robot, num_steps)

            previous_angles_2 = move_to_point(direction, point, robot, num_steps, previous_angles_1[-1].flatten().tolist()[0])
            previous_angles_3 = move_to_point(direction, point, robot, num_steps, previous_angles_2[-1].flatten().tolist()[0])




        else:
            ee_pos = robot.end_effector_position()
            initial_angles = previous_angles_3[-1].flatten().tolist()[0]

            ee_pos = round_end_effector_position(ee_pos.tolist()[0], direction, previous_point)

            if (index) % 2 == 0:


                # new_base = tr.trotz(0, unit="deg", xyz=ee_pos)
                if int(index) == 4:
                    new_base = flip_base(ee_pos, direction, 0)
                    new_base = flip_base(ee_pos, "left", -90)
                    #
                    new_pos = create_point_from_homogeneous_transform(new_base)
                    #
                    # new_base = new_base * flip_base(ee_pos, "left", 0)
                    # #
                    # ee_pos[0] = ee_pos[0] + 0.5
                    # new_base = new_base * flip_base(ee_pos, "top", 180)

                    new_base[0:3, 3] = new_pos
                    new_base[0, 3] = new_base[0, 3] + 0.5
                    new_base[2, 3] = new_base[2, 3] + 0.5
                    # ee_pos = robot.end_effector_position()
                    # ee_pos = ee_pos.tolist()[0]
                    # ee_pos[0] = math.floor(ee_pos[0])+1
                    # ee_pos[2] = round(ee_pos[2])-0.5
                    # new_base = flip_base(ee_pos, "left", 0)
                    robot.base = new_base
                    # direction = "top"
                    # previous_direction = "top"
                    # print("ROBOT BASE: {} END_EFFECTOR_POS: {}".format(robot.base, robot.end_effector_position()))
                    previous_direction = "left"
                elif int(index) == 6:
                    new_base = flip_base(ee_pos, "left", 0)
                    new_base = flip_base(ee_pos, "left", -90)
                    #
                    new_pos = create_point_from_homogeneous_transform(new_base)
                    #
                    # new_base = new_base * flip_base(ee_pos, "left", 0)
                    # #
                    # ee_pos[0] = ee_pos[0] + 0.5
                    new_base = new_base * flip_base(ee_pos, "top", 0)

                    new_base[0:3, 3] = new_pos
                    new_base[0, 3] = new_base[0, 3] + 0.5
                    new_base[2, 3] = new_base[2, 3] + 0.5
                    # ee_pos = robot.end_effector_position()
                    # ee_pos = ee_pos.tolist()[0]
                    # ee_pos[0] = math.floor(ee_pos[0])+1
                    # ee_pos[2] = round(ee_pos[2])-0.5
                    # new_base = flip_base(ee_pos, "left", 0)
                    robot.base = new_base
                    previous_direction = "left"
                    # direction = "top"
                    # previous_direction = "top"
                    # print("ROBOT BASE: {}
                # if index - 2 >= 0 and path[index - 1][-1] == direction and path[index - 2][-1] != direction:
                #     rotation = 90
                # else:
                #     rotation = 0
                else:
                    new_base = flip_base(ee_pos, direction, 0)

                    temp = initial_angles[1]
                    initial_angles[1] = 180 / 2 + initial_angles[3]
                    initial_angles[3] = temp - 180 / 2
                    flip_angles = False

                # global_path.append((num_steps * index, path[index:]))
            else:
                # new_base = tr.trotz(180, unit="deg", xyz=ee_pos)


                # if index - 2 >= 0 and path[index - 1][-1] == direction and path[index - 2][-1] != direction:
                #
                #     rotation = 180
                #
                #
                # else:
                #     rotation = 180
                new_base = flip_base(ee_pos, direction, 180)
                if index == 3:
                    # ee_pos = robot.end_effector_position()
                    # ee_pos = ee_pos.tolist()[0]
                    # ee_pos[0] = math.floor(ee_pos[0])+1
                    # ee_pos[2] = round(ee_pos[2])-0.5
                    # new_base = flip_base(ee_pos, "left", 90)

                    new_base = flip_base(ee_pos, "left", -90)

                    robot.base = new_base
                    new_pos = create_point_from_homogeneous_transform(new_base)
                    # self.base=new_base

                    # ee_pos = self.end_effector_position(new_base[0:3, 3])
                    # ee_pos = ee_pos.tolist()[0]
                    # ee_pos[0] = ee_pos[0] + 0.5
                    # ee_pos[2] = round(ee_pos[2])
                    new_base = new_base * flip_base(ee_pos, "top", 180)
                    #
                    new_base[0:3, 3] = new_pos

                    new_base[0, 3] = new_base[0, 3] + 0.5
                    new_base[2, 3] = new_base[2, 3] + 0.5
                    # self.base = new_base

                    # new_base = self.flip_base(ee_pos, "left", 90)
                    robot.base = new_base


                if int(index) == 5:
                    # new_base = flip_base(ee_pos, "left", 0)
                    new_base = flip_base(ee_pos, "left", -90)
                    new_pos = create_point_from_homogeneous_transform(new_base)
                    #
                    # new_base = new_base * flip_base(ee_pos, "left", 0)
                    # #
                    # ee_pos[0] = ee_pos[0] + 0.5
                    new_base = new_base * flip_base(ee_pos, "top", 180)

                    new_base[0:3, 3] = new_pos
                    new_base[0, 3] = new_base[0, 3] + 0.5
                    new_base[2, 3] = new_base[2, 3] + 0.5
                    # ee_pos = robot.end_effector_position()
                    # ee_pos = ee_pos.tolist()[0]
                    # ee_pos[0] = math.floor(ee_pos[0])+1
                    # ee_pos[2] = round(ee_pos[2])-0.5
                    # new_base = flip_base(ee_pos, "left", 0)
                    robot.base = new_base
                    # direction = "top"
                    # previous_direction = "top"
                    # print("ROBOT BASE: {} END_EFFECTOR_POS: {}".format(robot.base, robot.end_effector_position()))

                temp = initial_angles[1]
                initial_angles[1] = 180 / 2 + initial_angles[3]
                initial_angles[3] = temp - 180 / 2
                flip_angles = True



            robot.base = new_base
            robot.update_angles(initial_angles, unit="deg")






            # print("Initial Angles: {}".format(initial_angles))
            ee_pos = robot.end_effector_position(np.asarray(initial_angles) * np.pi/180)


            # # ee_pos = robot.end_effector_position()
            # if index == 3:
            #     ee_up = np.copy(ee_pos).tolist()[0]
            #     ee_up = round_end_effector_position(ee_up, direction)
            #     print("EEEEEEEEEEEE POSITION: {}      GOING TO: {}".format(ee_up, point))
            # else:
            ee_up = np.copy(ee_pos).tolist()[0]
            ee_up = round_end_effector_position(ee_up, direction, previous_point)
            print("Going to point: {}\t EE Pos: {}\tRounded Pos: {}".format(point, ee_pos, ee_up))



            # TODO: FIX TO ACCEPT ANY ORIENTATION, NOT JUST +Z
            ee_up = add_offset(ee_up, previous_direction, offset)
            # ee_up[2] = ee_up[2] + offset

            stop_above = np.copy(point)


            stop_above = add_offset(stop_above, direction, offset)

            if direction == path[index - 1][-1]:
                direction = "top"
                previous_direction = "top"

            previous_angles_1 = move_to_point(previous_direction, ee_up, robot, num_steps, initial_angles, flip_angles=flip_angles)




            previous_angles_2 = move_to_point(direction, stop_above, robot, num_steps, previous_angles_1[-1].flatten().tolist()[0], flip_angles=flip_angles)
            #
            # if index == 3:
            #     direction="top"
            #
            # if index == 4:
            #     direction="top"


            previous_angles_3 = move_to_point(direction, point, robot, num_steps, previous_angles_2[-1].flatten().tolist()[0], flip_angles=flip_angles)
            # robot.update_angles(previous_angles_3[-1].flatten().tolist()[0], unit="rad")

        save_path = update_path(save_path, previous_angles_1, previous_angles_2, previous_angles_3)

        # previous_point = point
    return save_path, global_path, global_direction

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
