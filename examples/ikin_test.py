import robopy.base.model as model
import numpy as np
import robopy.base.transforms as tr

def main():
    blueprint = np.array([
        [[1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 1, 1]],
        [[1, 0, 0], [1, 0, 0], [1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [1, 1, 0], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [1, 0, 0], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[0, 0, 0], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
    ])

    robot = model.Puma560_TEST(base=np.matrix([[1, 0, 0, 0.5],
                                              [0, 1, 0, 0.5],
                                              [0, 0, 1, 1],
                                              [0, 0, 0, 1]]), blueprint=blueprint)

    num_steps = 60


    ik_motion = follow_path(robot, num_steps, offset=0.5)
    robot = model.Puma560_TEST(base=np.matrix([[1, 0, 0, 0.5],
                                               [0, 1, 0, 0.5],
                                               [0, 0, 1, 1],
                                               [0, 0, 0, 1]]), blueprint=blueprint)
    robot.animate(stances=ik_motion, frame_rate=30, unit='deg', num_steps=num_steps * 3, display_path=False, obstacles=[[num_steps*3, (4, 0, 0)], [num_steps*9, (5, 0, 0)], [num_steps*15, (6, 0, 0)]])

def move_to_point(point, robot, num_steps, previous_angles=None):
    print(point)
    ik_angles = robot.ikineConstrained(point) * 180 / np.pi
    print(ik_angles)

    if previous_angles is None:
        previous_angles = [1] * robot.length

    else:
        print("Previous Angles: {}".format(previous_angles))
    forward_1 = np.transpose(np.asmatrix(np.linspace(float(previous_angles[0]), ik_angles[0], num_steps)))
    forward_2 = np.transpose(np.asmatrix(np.linspace(float(previous_angles[1]), ik_angles[1], num_steps)))
    forward_3 = np.transpose(np.asmatrix(np.linspace(float(previous_angles[2]), ik_angles[2], num_steps)))
    forward_4 = np.transpose(np.asmatrix(np.linspace(float(previous_angles[3]), ik_angles[3], num_steps)))
    ik_test = np.concatenate((forward_1, forward_2, forward_3, forward_4), axis=1)
    robot.update_angles(ik_test[-1].flatten().tolist()[0], unit="rad")
    return ik_test

def follow_path(robot, num_steps, offset):
    # path = [(3.5, 0.5, 1), (2.5, 0.5, 1), (3.5, 0.5, 1), (2.5, 0.5, 1), (3.5, 0.5, 1), (2.5, 0.5, 1), (3.5, 0.5, 1), (2.5, 0.5, 1)]
    path = [(3.5, 1.5, 1), (1.5, 0.5, 1),(4.5, 0.5, 1), (2.5, 0.5, 1),(5.5, 0.5, 1), (3.5, 0.5, 1),(6.5, 0.5, 1), (4.5, 0.5, 1) ]
    # path = [(3.5, 0.5, 1), (1.5, 0.5, 1), (3.5, 0.5, 2), (3.5, 0.5, 1), (5.5, 0.5, 3), (4.5, 0.5, 2), (6.5, 0.5, 4),
    #         (5.5, 0.5, 2)]

    save_path = None
    for index, point in enumerate(path):
        # move_ee_up = robot.end_effector_position().flatten().tolist()[0]
        move_ee_up = np.copy(path[index-1])
        print(move_ee_up)
        move_ee_up[2] = float(move_ee_up[2]) + offset

        if index == 0:
            move_up = list(point)
            move_up[2] = move_up[2] + offset
            previous_angles_1 = move_to_point(move_up, robot, num_steps)

            previous_angles_2 = move_to_point(point, robot, num_steps, previous_angles_1[-1].flatten().tolist()[0])
            previous_angles_3 = move_to_point(point, robot, num_steps, previous_angles_2[-1].flatten().tolist()[0])
            # robot.update_angles(previous_angles_3[-1].flatten().tolist()[0])



        else:
            ee_pos = robot.end_effector_position()
            if (index) % 2 == 0:
                new_base = tr.trotz(0, unit="deg", xyz=ee_pos.tolist()[0])
            else:
                new_base = tr.trotz(180, unit="deg", xyz=ee_pos.tolist()[0])

            robot.base = new_base
            ee_pos = robot.end_effector_position()
            ee_up = np.copy(ee_pos).tolist()[0]
            ee_up[2] = ee_up[2] + offset
            previous_angles_1 = move_to_point(ee_up, robot, num_steps, previous_angles_3[-1].flatten().tolist()[0])

            stop_above = np.copy(point)
            stop_above[2] = stop_above[2] + 0.5
            previous_angles_2 = move_to_point(stop_above, robot, num_steps, previous_angles_1[-1].flatten().tolist()[0])

            previous_angles_3 = move_to_point(point, robot, num_steps, previous_angles_2[-1].flatten().tolist()[0])
            robot.update_angles(previous_angles_3[-1].flatten().tolist()[0], unit="rad")
        save_path = update_path(save_path, previous_angles_1, previous_angles_2, previous_angles_3)

    return save_path

def update_path(save_path, new_motion_1, new_motion_2, new_motion_3):
    if save_path is None:
        return np.concatenate((new_motion_1, new_motion_2, new_motion_3))
    return np.concatenate((save_path, new_motion_1, new_motion_2, new_motion_3))

if __name__ == '__main__':
    main()