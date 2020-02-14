import robopy.base.model as model
import numpy as np
import robopy.base.transforms as tr
import math

class FakeUpdate:
    def __init__(self, base):
        self.robot_base = base

def main():

    blueprint = np.array([
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
    ])

    base = np.matrix([[1, 0, 0, 0.5],
                                              [0, 1, 0, 0.5],
                                              [0, 0, 1, 1],
                                              [0, 0, 0, 1]])

    robot = model.Inchworm(base=base, blueprint=blueprint)
    # robot.plot(np.asmatrix([0, 7, -13, -85]), unit="deg") # stretched out
    q1 = np.array([0, 84.78,  -116.439, -58.29]) * np.pi / 180



    # q1 = np.array([0, 1.47977127e+00, -2.03224729e+00, -1.01750194e+00]) * 180 / np.pi #up
    # q1 = np.array([0,  1.05228649e+00, -1.46107186e+00, -1.16119259e+00]) * 180 / np.pi #over
    # q1 = np.array([0, 4.50414252e+01,  -9.00828504e+01, -4.49116859e+01]) #down

    robot.plot(np.asmatrix(q1), unit="deg", update=FakeUpdate(base)) # about one step between


if __name__ == '__main__':
    main()