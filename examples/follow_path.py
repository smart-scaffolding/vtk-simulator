import numpy as np

import logging
import os
from robopy.base.path_planner import PathPlanner
from math import pi
import time

block_size = 3.0


def arm_animation():
    logging.basicConfig(level=logging.DEBUG,
                       format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
                       datefmt='%m-%d %H:%M',
                       filename=os.getcwd() + "/app.log",
                       filemode='w')
    # define a Handler which writes INFO messages or higher to the sys.stderr
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    # set a format which is simpler for console use
    formatter = logging.Formatter('%(name)-12s: %(levelname)-8s %(message)s')
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)

    # logging.info('Jackdaws love my big sphinx of quartz.')


    q0 = np.array([3.14143349, 1.77881778, 2.11949991, 0.20802146])
    # q0 = np.array([0, -np.pi/2, 0.7, 0.8, 0.9, 0])
    base_pos = np.array([0.5, 0., 1.3])



    blueprint = np.array([
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 1, 1]],
            [[1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        ])

    building_implemented = np.array([
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 1, 1]],
        [[1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
    ])

    left_to_build = (blueprint == building_implemented)

    result = np.where(left_to_build == False)

    listOfCoordinates = list(zip(result[0], result[1], result[2]))

    print("{} BLOCKS REMAINING".format(len(listOfCoordinates)))

    start = (3, 0, 0)
    for index, cord in enumerate(listOfCoordinates):
        print("\tBlock {}: ({}, {}, {})".format(
            index, cord[0], cord[1], cord[2]))
        if index > 0:
            start = goal
        goal = (listOfCoordinates[index])


        # try:
        path_planner = PathPlanner(start, goal, blueprint)

        route = path_planner.get_path()
        color = path_planner.display_path()

        sample_path = []
        for i in route:
            sample_path.append([i[0] + 0.5, i[1]+0.5, i[2] + 1.0])

        print("SAMPLE PATH: {}".format(sample_path))
        # robot_behavior.follow_path(sample_path)
        # robot_behavior.place_block([goal[0] + 0.5, goal[1], goal[2] + 1])
        #
        # robot_behavior.show_behavior(building_implemented, color)
        # except Exception as e:
        #     print("Exception occured: {}".format(e))
        #     logging.error(e)
        #     continue

    # robot_behavior.show_behavior(building_implemented, color)

if __name__ == '__main__':
    arm_animation()
