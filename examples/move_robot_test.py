import robopy.base.model as model
import numpy as np
import robopy.base.transforms as tr
import math
from robopy.base.FaceStar import *
from robopy.base.common import create_point_from_homogeneous_transform, flip_base, round_end_effector_position
from robopy.base.graphics import AnimationUpdate
import time

accuracy = 1e-7
threshold = 1
# num_way_points = 2
use_face_star = False
animate = False
move_both_end_effectors=True
use_serial = True


robot_ee_starting_point = (2.5, 0.5, 1)
def main():

    #playground
    blueprint = np.array([
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
    ])


    robot = model.Inchworm(base=np.matrix([[1, 0, 0, 0.5],
                                               [0, 1, 0, 0.5],
                                               [0, 0, 1, 1.],
                                               [0, 0, 0, 1]]), blueprint=blueprint)

    # robot.update_angles(np.array([0, 0, 0, -90]), unit="deg")
    num_steps = 20

    # robot.update_angles(np.array([0, 1.08030020e+00,  -2.16060041e+00, -4.89677758e-01])*180/np.pi)
    # startFace = BlockFace(1, 0, 0, 'top')
    startFace = BlockFace(1, 1, 0, 'top')
    # endFace = BlockFace(3, 0, 0, 'top')
    # endFace = BlockFace(5, 1, 3, 'top')
    # endFace = BlockFace(5, 0, 0, 'top')
    # endFace = BlockFace(3, 2, 6, "top")
    endFace = BlockFace(5, 1, 3, "top")
    # endFace= BlockFace(3, 2, 5, "left")
    # endFace = BlockFace(5, 0, 0, 'top')



    ik_motion, path, directions, animation_update = follow_path(robot, num_steps, offset=1,
                                                                         startFace=startFace,
                                                                    endFace=endFace, blueprint=blueprint,

                                                                         )

    port = None
    if use_serial:
        port='/dev/cu.usbmodem14201'

    robot = model.Inchworm(base=np.matrix([[1, 0, 0, 0.5],
                                               [0, 1, 0, 0.5],
                                               [0, 0, 1, 1.],
                                               [0, 0, 0, 1]]), blueprint=blueprint, port=port)

    # robot.update_angles(np.array([0, 0, 0, -90]), unit="deg")
    print("Path Length: {}".format(len(path[0][1:][0])))
    print("Path: {}".format(path))
    print("Num Steps: {}".format(num_steps))

    robot_orientation = ["top"] * (len(path[0][1:][0]))
    if use_face_star:
        robot_orientation = ["top"] * (len(path[0][1:][0])) * 2
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


    print("Final Angle: {}".format(ik_motion[-1]))
    flip_angles=True
    for index, angle in enumerate(ik_motion):
        # print(f"Angle: {angle}")

        angle = angle.tolist()[0]
        # robot.map_angles_to_robot(angle)
        if index % (num_steps) == 0:
            print("\nFIRST POINT REACHED")

        if index % (num_steps * 2) == 0:
            print("\n SECOND STEP REACHED")
        if index % (num_steps*3) == 0:
            print("\n THIRD STEP REACHED")
            flip_angles = True if flip_angles == False else False
            print("\n\nIndex: {}  New Flipping Angle: {}".format(index, flip_angles))

        if flip_angles:
            temp = angle[1]
            angle[1] = 180 / 2 + angle[3]
            angle[3] = temp - 180 / 2

        if use_serial:
            robot.send_to_robot(angle, delay=0.2)
        # robot.plot(angle, unit="deg")


    if animate:

        robot.animate(stances=ik_motion, frame_rate=30, unit='deg', num_steps=num_steps*3, orientation=robot_orientation,
                  showPath=True, showPlacedBlock=True, update=animation_update)

def move_to_point(direction, point, robot, num_steps, previous_angles=None, flip_angles=False, accuracy=accuracy):
    # print(point)
    try:
        ik_angles = robot.ikineConstrained(direction, point, flipped=flip_angles, accuracy=accuracy) * 180 / np.pi ## converted to degrees
        # print(ik_angles)

    except ValueError as e:
        if accuracy >= threshold:


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
        else:
            print("IK Accuracy Too High, Lowering Value To: {}".format(accuracy))
            return move_to_point(direction, point, robot, num_steps, previous_angles, flip_angles, accuracy * 10)


    if previous_angles is None:
        previous_angles = [1.61095456e-15,  6.18966422e+01, -1.23793284e+02, -2.80564688e+01]

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

def follow_path(robot, num_steps, offset, startFace, endFace, blueprint, secondPosition=None):
    # path = [(3.5, 0.5, 1), (2.5, 0.5, 1), (3.5, 0.5, 1), (2.5, 0.5, 1), (3.5, 0.5, 1), (2.5, 0.5, 1), (3.5, 0.5, 1), (2.5, 0.5, 1)]
    # path = [(3.5, 1.5, 1), (1.5, 0.5, 1),(4.5, 0.5, 1), (2.5, 0.5, 1),(5.5, 0.5, 1), (3.5, 0.5, 1),(6.5, 0.5, 1), (4.5, 0.5, 1) ]
    # path = [(3.49, 1.49, 1), (1.49, 1.49, 1), (4.49, 1.49, 1), (2.49, 1.49, 1),(5.49, 1.49, 2),(4.5, 1.5, 1),(6.5, 1.5, 3),(5.5, 1.5, 2),
    #         (7.49, 1.49, 4),(6.5, 1.5, 3),(7.5, 2.5, 4), (7.5, 1.5, 4), (6.5, 2.5, 4), (7.5, 2.5, 4), (5.5, 2.5, 4),(6.5, 2.5, 4)]

    # path = [(2.49, 2.49, 1.3, "top"), (0.49, 2.49, 1.3, "top"),(2, 2.49, 2.5, "left")]
    if not use_face_star:
        # path = [(1, 2, 0, "top"), (0, 2, 0, "top"), (3, 2, 3, "left"), (3, 2, 2, "left"), (3, 2, 5, "left"), (3, 2, 4, "left") ]
        # path = [(3, 0, 0, "top")]
        path = [(3, 0, 0, "top"), (1, 0, 0, "top"), (4, 0, 0, "top")]
    armReach = [2.38, 1.58]

    # armReach = [1.5, 1.5]

    # armReach = [1.5, 1.5]


    if use_face_star:

        faceStarPlanner = FaceStar(startFace, endFace, blueprint, armReach)
        path = faceStarPlanner.get_path()
    # #
    global_path = []
    global_path.append((num_steps, path))

    if use_face_star and move_both_end_effectors:
        filtered_path = []
        for index, point in enumerate(path):
            filtered_path.append(point)
            if index == 0:
                if secondPosition == 0:
                    filtered_path.append((0, 0, 0, "top"))
                else:
                    filtered_path.append((0, 1, 0, "top"))
            else:
                filtered_path.append(path[index-1])

        path = filtered_path

    if not move_both_end_effectors:
        path.pop(0)

    global_direction = []

    update_animation = []

    save_path = None
    previous_point = None
    back_foot_pos = None
    # previous_direction = "top"
    for index, item in enumerate(path):
        print(path)

        # move_ee_up = robot.end_effector_position().flatten().tolist()[0]

        direction = item[-1]
        print(direction)
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
            point[0] = item[0] - 1.37 #-1.37
            point[1] = item[1] + 0.5
            point[2] = item[2] - 0.87 #-.87
        if direction == "front" or direction =="back":
            point[0] = item[0] + 0.5
            point[1] = item[1] + 1
            point[2] = item[2] + 0.5

        print("\t\t\n\nINDEX: {}".format(index))
        print("\nPOINT: {}   DIRECTION: {}    PREVIOUS_DIRECTION: {}".format(point, direction, path[index-1][-1]))


        if index == 0:
            ee_up = list(robot_ee_starting_point)
            # TODO: FIX TO ACCEPT ANY ORIENTATION, NOT JUST +Z
            # move_up[2] = move_up[2] + offset
            previous_angles_1, previous_angles_2, previous_angles_3 = None, None, None

            ee_up = add_offset(ee_up, previous_direction, offset)

            # divorce_wall = np.linspace(robot.end_effector_position().tolist()[0], move_up, num_way_points)

            # for waypoint in divorce_wall:
            #     if previous_angles_1 is not None:
            #         previous_angles_1 = np.vstack([previous_angles_1,
            #                                        move_to_point(direction, waypoint, robot, num_steps,
            #                                                      previous_angles_1[-1].flatten().tolist()[0])])
            #     else:
            previous_angles_1 = move_to_point(direction, ee_up, robot, num_steps)


            # for waypoint in range(num_steps):
            #     if previous_angles_2 is not None:
            #         previous_angles_2 = np.vstack([previous_angles_2,
            #                                        move_to_point(direction, point, robot, num_steps,
            #                                                      previous_angles_2[-1].flatten().tolist()[0])])
            #     else:
            stop_above = np.copy(point)
            stop_above = add_offset(stop_above, direction, offset)
            previous_angles_2 = move_to_point(direction, stop_above, robot, num_steps,
                                                      previous_angles_1[-1].flatten().tolist()[0])


            # for waypoint in range(num_steps):
            #     if previous_angles_3 is not None:
            #         previous_angles_3 = np.vstack([previous_angles_3,
            #                                        move_to_point(direction, point, robot, num_steps,
            #                                                      previous_angles_2[-1].flatten().tolist()[0])])
            #     else:
            previous_angles_3 = move_to_point(direction, point, robot, num_steps,
                                                      previous_angles_2[-1].flatten().tolist()[0])





        else:
            # previous_angles_3 = [1.61095456e-15, 6.18966422e+01, -1.23793284e+02, -2.80564688e+01]
            ee_pos = robot.end_effector_position()
            initial_angles = previous_angles_3[-1].flatten().tolist()[0]
            #
            ee_pos = round_end_effector_position(ee_pos.tolist()[0], direction, previous_point)
            # ee_pos = np.copy(previous_point)

            if (index) % 2 == 0:

                new_base = flip_base(ee_pos, previous_direction, 0)

                temp = initial_angles[1]
                initial_angles[1] = 180 / 2 + initial_angles[3]
                initial_angles[3] = temp - 180 / 2
                flip_angles = False

                # global_path.append((num_steps * index, path[index:]))
            else:
                new_base = flip_base(ee_pos, previous_direction, 180)


                temp = initial_angles[1]
                initial_angles[1] = 180 / 2 + initial_angles[3]
                initial_angles[3] = temp - 180 / 2
                flip_angles = True



            robot.base = new_base
            robot.update_angles(initial_angles, unit="deg")






            # print("Initial Angles: {}".format(initial_angles))
            # ee_pos = robot.end_effector_position()
            print("Previous Point: {}".format(previous_point))
            ee_pos = back_foot_pos


            # ee_up = np.copy(ee_pos).tolist()[0]
            ee_up = np.copy(ee_pos)
            # ee_up = round_end_effector_position(ee_up, direction, previous_point)
            print("Going to point: {}\t EE Pos: {}\tRounded Pos: {}".format(point, ee_pos, ee_up))
            print("\tPrevious Direction: {}".format(previous_direction))


            # TODO: FIX TO ACCEPT ANY ORIENTATION, NOT JUST +Z
            ee_up = add_offset(ee_up, previous_direction, offset, previous_point, index=index)

            stop_above = np.copy(point)


            stop_above = add_offset(stop_above, direction, offset)

            print("\t\t\tOffsets: \n\t\t\tEE Up: {}\n\t\t\tStop Above: {}".format(ee_up, stop_above))

            if direction == path[index - 1][-1]:
                direction = "top"
                previous_direction = "top"


            previous_angles_1, previous_angles_2, previous_angles_3 = None, None, None

            # divorce_wall = np.linspace(ee_pos, ee_up, num_way_points)

            # for waypoint in divorce_wall:
            #     if previous_angles_1 is not None:
            #         previous_angles_1 = np.vstack([previous_angles_1, move_to_point(previous_direction, waypoint, robot, num_steps, previous_angles_1[-1].flatten().tolist()[0], flip_angles=flip_angles)])
            #     else:
            previous_angles_1 = move_to_point(previous_direction, ee_up, robot, num_steps, initial_angles, flip_angles=flip_angles)

            # stop_above_path = np.linspace(ee_up, stop_above, num_way_points)

            # for waypoint in stop_above_path:
            #     if previous_angles_2 is not None:
            #         previous_angles_2 = np.vstack([previous_angles_2,
            #                                            move_to_point(direction, waypoint, robot, num_steps, previous_angles_2[-1].flatten().tolist()[0], flip_angles=flip_angles)])
            #     else:
            previous_angles_2 = move_to_point(direction, stop_above, robot, num_steps, previous_angles_1[-1].flatten().tolist()[0], flip_angles=flip_angles)

            # reach_point = np.linspace(stop_above, point, num_way_points)
            #
            # for waypoint in reach_point:
            #     if previous_angles_3 is not None:
            #         previous_angles_3 = np.vstack([previous_angles_3,
            #                                            move_to_point(direction, waypoint, robot, num_steps, previous_angles_3[-1].flatten().tolist()[0], flip_angles=flip_angles)])
            #     else:
            previous_angles_3 = move_to_point(direction, point, robot, num_steps, previous_angles_2[-1].flatten().tolist()[0], flip_angles=flip_angles)

            # previous_angles_3 = move_to_point(direction, point, robot, num_steps, previous_angles_2[-1].flatten().tolist()[0], flip_angles=flip_angles)
            # robot.update_angles(previous_angles_3[-1].flatten().tolist()[0], unit="rad")

        save_path = update_path(save_path, previous_angles_1, previous_angles_2, previous_angles_3)

        previous_point = point
        back_foot_pos = create_point_from_homogeneous_transform(robot.base).flatten().tolist()[0]
        print("BACK FOOT: {}".format(back_foot_pos))

        if path[index-1][-1] == path[index][-1]:
            direction = path[index][-1]
        else:
            direction = path[index-1][-1]

        # if index == 11 and secondPosition == 0:
        #     update_animation.append(
        #         AnimationUpdate(robot=robot, robot_base=robot.base, direction=direction, path=path[index:], index=index,
        #                         trajectory=[ee_up, stop_above, point], placedObstacle=True, obstacle=[6, 0, 0]))
        # else:
        update_animation.append(AnimationUpdate(robot=robot, robot_base=robot.base, direction=direction, path=path[index:], index=index, trajectory=[ee_up, stop_above, point]))

    update_animation.append(
        AnimationUpdate(robot=robot, robot_base=robot.base, direction=direction, path=path[index:], trajectory=[],
                        index=index-1, placedObstacle=True, obstacle=[6, 0, 0]))
    return save_path, global_path, global_direction, update_animation

def update_path(save_path, new_motion_1, new_motion_2, new_motion_3):
    if save_path is None:
        return np.concatenate((new_motion_1, new_motion_2, new_motion_3))
    return np.concatenate((save_path, new_motion_1, new_motion_2, new_motion_3))

def update_path_single(save_path, new_motion_1):
    return np.concatenate((save_path, new_motion_1))

def add_offset(ee_pos, direction, offset, previous_point=None, index=None):
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
        # ee_pos[2] = ee_pos[2] - 0.5
        # if previous_point is not None:
        #     print("BACK FOOT POS: {}".format(previous_point))
        #     ee_pos[2] = previous_point[2][0]
            # ee_pos[1] = previous_point[1]
        # if index == 3:
        #     ee_pos[2] = 1
        # if index == 4:
        #     ee_pos[2] = 1
        # if index == 5:
        #     print("\n\nIndex 5")
        #     print(previous_point)
        #     print(ee_pos)
        # if index == 6:
        #     ee_pos[2] = 1
    if direction == "right":
        ee_pos[0] = float(ee_pos[0]) + offset


    return ee_pos



if __name__ == '__main__':
    main()
