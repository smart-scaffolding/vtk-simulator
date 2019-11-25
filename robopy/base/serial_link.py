# Created by: Aditya Dua
# 30 September 2017
from __future__ import print_function
from abc import ABC
import math
from math import pi
import numpy as np
import vtk
from . import transforms
from .graphics import VtkPipeline
from .graphics import axesCube
from .graphics import axesCubeFloor
from .graphics import vtk_named_colors
from .graphics import cubeForPath
from .graphics import MakeAxesActor
from .graphics import vtk_named_colors
import pkg_resources
from scipy.optimize import minimize
import robopy.base.transforms as tr
from .common import *
from scipy.spatial.transform import Rotation as R

class SerialLink:
    """
    SerialLink object class.
    """

    def __init__(self, links, name=None, base=None, tool=None, stl_files=None, q=None, colors=None, param=None, blueprint=None):
        """
        Creates a SerialLink object.
        :param links: a list of links that will constitute SerialLink object.
        :param name: name property of the object.
        :param base: base transform applied to the SerialLink object.
        :param stl_files: STL file names to associate with links. Only works for pre-implemented models in model module.
        :param q: initial angles for link joints.
        :param colors: colors of STL files.
        """
        self.pipeline = None
        self.links = links
        if q is None:
            self.q = np.matrix([0 for each in links])
        if base is None:
            self.base = np.asmatrix(np.eye(4, 4))
        else:
            assert (type(base) is np.matrix) and (base.shape == (4, 4))
            self.base = base
        if tool is None:
            self.tool = np.asmatrix(np.eye(4, 4))
        else:
            assert (type(tool) is np.matrix) and (tool.shape == (4, 4))
            self.tool = tool
        # Following arguments initialised by plot function and animate functions only
        if stl_files is None:
            # Default stick figure model code goes here
            pass
        else:
            self.stl_files = stl_files
        if name is None:
            self.name = ''
        else:
            self.name = name
        if colors is None:
            self.colors = vtk_named_colors(["Grey"] * len(stl_files))
        else:
            self.colors = colors
        if param is None:
            # If model deosn't pass params, then use these default ones
            self.param = {
                "cube_axes_x_bounds": np.matrix([[-1.5, 1.5]]),
                "cube_axes_y_bounds": np.matrix([[-1.5, 1.5]]),
                "cube_axes_z_bounds": np.matrix([[-1.5, 1.5]]),
                "floor_position": np.matrix([[0, 0, 0]])
            }
        else:
            self.param = param

        if blueprint is None:
            raise Exception("Please provide a blueprint")
        else:
            self.blueprint = blueprint

        # self.scale = .13

    def __iter__(self):
        return (each for each in self.links)

    def reset(self):
        """Resets the arm back to its resting state, i.e. q0

        :rtype: None
        """
        self.update_angles(self.q)

    def get_current_joint_config(self, unit='rad'):
        """Gets the current joint configuration from the links

        :returns: 1xN vector of current joint config
        :rtype: numpy.ndarray
        """
        q = np.zeros(self.length)
        for i, link in enumerate(self.links):
            q[i] = link.theta
        if unit == 'deg':
            q = q * 180 / pi
        return q

    def update_angles(self, new_angles, save=False, unit='rad'):
        """Updates all the link's angles

        :param new_angles: 1xN vector of new link angles
        :type new_angles: numpy.ndarray

        :param save: Flag that determines if the update is cached
        :param save: bool

        :rtype: None
        """

        ##THIS EXPECTS EVERYTHING TO BE IN DEGREES
        ##SETS LINK ANGLES TO RADIANS
        for link, new_theta in zip(self.links, new_angles):
            if unit == 'deg':
                new_theta = new_theta * np.pi / 180
            link.set_theta(new_theta)
        # self.update_link_positions()

        # if save:
        #     q = np.array([l.theta for l in self.links])
        #     self.qs = np.vstack((self.qs, q.copy()))
    def update_link_angle(self, link, new_angle, save=False):
        """Updates the given link's angle with the given angle

        :param link: The link you want to update
        :type link: int

        :param new_angle: The link's new angle
        :type new_angle: int

        :param save: Flag that determines if the update is cached
        :type save: bool

        :rtype: None
        """
        self.links[link].set_theta(new_angle)
        # self.update_link_positions()

        # Save each config for replay
        # if save:
        #     q = np.array([l.theta for l in self.links])
        #     self.qs = np.vstack((self.qs, q.copy()))

    # TODO: Acceleration over time seems like a weird way to update this
    def update_link_velocity(self, link, accel, time):
        """Updates the given link's velocity with the given
        acceleration over the given time

        :param link: The link you want to update
        :type link: int

        :param accel: The acceleration (Radians per second^2)
        :type accel: int

        :param time: The time (Seconds)
        :type time: int

        :rtype: None
        """
        self.links[link].update_velocity(accel, time)
        # self.update_link_positions()

    # def update_link_positions(self):
    #     """Walk through all the links and update their positions.
    #
    #     :rtype: None
    #     """
    #
    #     for i, link in enumerate(self.links):
    #         # Set link base position
    #         if i == 0:
    #             link.base_pos = utils.create_point_from_homogeneous_transform(
    #                 self.base)
    #         else:
    #             link.base_pos = self.links[i - 1].end_pos
    #
    #         # Set link end position
    #         if link.length == 0 and link.offset == 0:
    #             link.end_pos = link.base_pos
    #         else:
    #             # Compute FKine up to that link endpoint
    #             # to get the location in homogenous coords
    #             t = self.fkine(links=range(i + 1))
    #             # Then convert that to world space
    #             end_pos = utils.create_point_from_homogeneous_transform(t).T
    #             link.end_pos = end_pos.A1



    def end_effector_position(self, q=None, transform=False, num_links=None):
        """Return end effector position

        :param q: Config to compute the end effector position for a given
                  1xN q vector
        :type q: numpy.ndarray or None

        :returns: Position (x, y, z) of end effector
        :rtype: numpy.ndarray
        """
        if q is None:
            q = self.get_current_joint_config()

        if num_links==None:
            num_links=self.length

        t = self.fkine(stance=q, num_links=num_links)
        if transform:
            return t
        end_pos = create_point_from_homogeneous_transform(t).T
        return end_pos

    def end_effector_velocity(self):
        """Calculate the end effector velocity of the arm given
        its current angular velocities.

        :returns: Returns linear and angular velocity in each dimension
                  (vx, vy, vz, wx, wy, wz).
        :rtype: np.ndarray
        """
        q = np.array([link.theta for link in self.links])
        dq = np.array([link.velocity for link in self.links])

        velocity = self.jacob0(q) * np.asmatrix(dq).T
        return velocity.A1

    @property
    def length(self):
        """
        length property
        :return: int
        """
        # return len(self.links)
        return 4

    def fkine(self, stance, unit='rad', apply_stance=False, actor_list=None, timer=None, num_steps=0, num_links=4, directions=None):
        """
        Calculates forward kinematics for a list of joint angles.
        :param stance: stance is list of joint angles.
        :param unit: unit of input angles.
        :param apply_stance: If True, then applied tp actor_list.
        :param actor_list: Passed to apply transformations computed by fkine.
        :param timer: internal use only (for animation).
        :return: homogeneous transformation matrix.
        """

        flipped=False

        if type(stance) is np.ndarray:
            stance = np.asmatrix(stance)
        if unit == 'deg':
            stance = stance * pi / 180
        if timer is None:
            timer = 0
        if num_steps > 0 and timer % num_steps == 0:
            # if timer - num_steps == 0:
            ee_pos = self.end_effector_position()
            ee_pos = ee_pos.tolist()[0]
            ee_pos[0] = math.floor(ee_pos[0]) + 0.5
            ee_pos[2] = round(ee_pos[2])
            if (timer / num_steps) % 2 == 0:
                if directions == None:
                    new_base = tr.trotz(0, unit="deg", xyz=ee_pos)
                # else:
                #     for direction in directions:
                #         if timer >= 50:
                #             new_base = self.flip_base(ee_pos, "left", 90)
                #             break
                #         else:
                #             new_base = tr.trotz(0, unit="deg", xyz=ee_pos)

                flipped=False
            else:
                # new_base = tr.trotz(180, unit="deg", xyz=ee_pos)
                if directions == None:
                    new_base = tr.trotz(180, unit="deg", xyz=ee_pos)
                # else:
                #     for direction in directions:
                #         if timer >= 50:
                #             new_base = self.flip_base(ee_pos, "left", 180)
                #             break
                #         else:
                #             new_base = tr.trotz(180, unit="deg", xyz=ee_pos)
                flipped=True
            print("EE_POS: {}".format(ee_pos))
            # new_base = new_base + tr.trotz(-90, unit='deg')

            self.base = new_base
        if stance is None:
            t = self.links[0].transform_matrix
        else:
            t = self.base

            angles = stance[timer, 0]
            # if flipped:
            #     temp = angles[1]
            #     angles[1] = angles[3]
            #     angles[3] = temp
            # if flipped:
            #     angles = np.flip(angles)
            # if flipped:
            #     temp = angles[1]
            #     angles[1] = np.absolute(np.pi / 2 + angles[3])
            #     angles[3] = -1 * np.absolute((np.pi / 2 - temp))
            # if flipped:
            #     temp = angles[1]
            #     angles[1] = np.pi / 2 + angles[3]
            #     angles[3] = temp - np.pi / 2
            t = t * self.links[0].A(angles)
        # actor_list[0].SetOrigin(t[0:3, 3])
        if apply_stance:
            actor_list[0].SetUserMatrix(transforms.np2vtk(t))
            actor_list[0].SetScale(self.scale)
        prev_t = t
        for i in range(1, num_links, 1):
            # print("I: {}".format(i))
            # print("\tT: {}".format(t))
            if stance is None:
                t = t * self.links[i].transform_matrix
            else:
                angles = stance[timer, i]
                # if flipped:
                #     temp = angles[1]
                #     angles[1] = angles[3]
                #     angles[3] = temp

                # if flipped:
                #     angles = np.flip(angles)
                # if flipped:
                #     temp = angles[1]
                #     angles[1] = np.pi/2 - angles[3]
                #     angles[3] = np.pi/2 - temp
                # if flipped:
                #     temp = angles[1]
                #     angles[1] = np.pi / 2 + angles[3]
                #     angles[3] = temp - np.pi / 2
                t = t * self.links[i].A(angles)
            if apply_stance:
                # print(actor_list[i])


                # x = actor_list[i + 1].GetUserMatrix().GetElement(0, 3)
                # y = actor_list[i + 1].GetUserMatrix().GetElement(1, 3)
                # z = actor_list[i + 1].GetUserMatrix().GetElement(2, 3)
                # print(x, y, z)
                # actor_list[i].SetOrigin([x, y, z])

                # shift_pos = t * self.links[i+1].A(stance[timer, i+1])
                # actor_list[i].SetOrigin(shift_pos[0:3, 3])

                # actor_list[i].SetOrigin(t[0:3, 3])
                actor_list[i].SetUserMatrix(transforms.np2vtk(t))
                actor_list[i].SetScale(self.scale)

                # actor_list[i].SetPosition(t[0:3, 3])
                # actor_list[i].SetOrientation(t[0:3, 2])
                # print("Position of part: {}".format(actor_list[i].GetPosition()))
                # print("User Matrix: {}".format(actor_list[i].GetUserMatrix()))
                # print("Matrix: {}".format(actor_list[i].GetMatrix()))
            prev_t = t
            # print("I TEST: {}".format(i))
            # print("Link LengthsL {}".format(len(self.links)))
            # print("Stance {}".format(stance))

        t = t * self.tool
        # if apply_stance:
        #     print("LENGTH: {}".format(self.length))
        #     actor_list[self.length-1].SetUserMatrix(transforms.np2vtk(t))
        return t

    def ikineConstrained(self, direction, p, num_iterations=1000, alpha=0.1, prior_q=None, vertical=False, flipped=False):
        """Computes the inverse kinematics to find the correct joint
        configuration to reach a given point

        :param p: The point (x, y, z) to solve the inverse kinematics for
        :type p: numpy.ndarray

        :param num_iterations: The number of iterations to try before
                               giving up
        :type num_iterations: int

        :param alpha: The stepsize for the ikine solver (0.0 - 1.0)
        :type alpha: int

        :returns: 1xN vector of the joint configuration for given point p.
        :rtype: numpy.ndarray
        """
        # Check to make sure alpha is between 0 and 1
        if not (0.0 <= alpha <= 1.0):
            print("Invalid alpha. Defaulting to 0.1")
            alpha = 0.1
        q = self.get_current_joint_config()
        # q = self.get_current_joint_config()*np.pi/180
        print("\t\tCURRENT JOINT CONFIG: {}".format(q))
        print("\t\tGOING TO POINT IK: {}".format(p))
        if flipped:
            temp = q[1]
            q[1] = np.pi/2 + q[3]
            q[3] = temp - np.pi/2
        pTarget = np.copy(p)
        # TODO fix the hardcoded value
        pTarget[2] = pTarget[2] + 1.04775

        goal = create_homogeneous_transform_from_point(pTarget)
        for i in range(num_iterations):
            # Calculate position error of the end effector
            curr = self.fkine(q, num_links=self.length-1)
            # print("Current EE Pos: {}".format(curr[0:3, 3]))
            # self.update_angles(q)

            err = goal - curr

            # Convert error from homogeneous to xyz space
            err = create_point_from_homogeneous_transform(err)


            # Get the psudoinverse of the Jacobian
            J = self.jacob0(q)
            vel_J = J[0:3, :]

            # Increment q a tiny bit
            delta_q = np.linalg.pinv(vel_J) * err
            delta_q = np.squeeze(np.asarray(delta_q))
            q = q + (alpha * delta_q.flatten())

            q[-1] = q[1] - np.pi/2

            if abs(np.linalg.norm(err)) <= 1e-1:

                absolute = np.absolute(q[1])+ (pi - np.absolute(q[2]))
                q[-1] = -1*(1.57-(9.4248 - absolute - 2 * pi))

                if direction == "top":
                    q[-1] = q[-1]
                if direction == "bottom":
                    q[-1] = q[-1] + pi

                if direction == "left" or direction == "front":
                    q[-1] = q[-1] + pi/2

                if direction == "right" or direction == "back":
                    q[-1] = q[-1] - pi/2
                # print("Absolute: {}".format(absolute))
                # print("Absolute Degrees: {}".format(absolute * 180 / pi))
                # print("Angle Dif: {}".format(q[-1] * 180 / pi))

                return q
        raise ValueError("Could not find solution.")


    def jacob0(self, q=None):
        """Calculates the jacobian in the world frame by finding it in
        the tool frame and then converting to the world frame.

        :param q: (Optional) 1xN joint configuration to compute the jacobian on
        :type q: numpy.ndarray

        :returns: 6xN Jacobian in the world frame
        :rtype: numpy.matrix
        """

        # Get the tool frame jacobian
        J = self.jacobn(q)

        # Set up homogeneous transform matrix for the world
        eet = self.fkine(q)
        rotation = get_rotation_from_homogeneous_transform(eet)
        zeros = np.zeros((3, 3))
        a1 = np.hstack((rotation, zeros))
        a2 = np.hstack((zeros, rotation))

        # Convert to world frame
        J = np.vstack((a1, a2)) * J
        return J

    def jacobn(self, q=None):
        """Calculates the jacobian in the tool frame

        :param q: (Optional) 1xN joint configuration to compute the jacobian on
        :type q: 1xN numpy.ndarray

        :returns: 6xN Jacobian in the tool frame
        :rtype: numpy.matrix
        """
        J = np.zeros((6, self.length))
        U = self.tool
        I = range(self.length - 1, -1, -1)

        for i, link in zip(I, self.links[::-1]):
            if np.any(q):
                U = link.A(q[i]) * U
            else:
                U = link.transform_matrix * U

            d = np.array([-U[0, 0] * U[1, 3] + U[1, 0] * U[0, 3],
                          -U[0, 1] * U[1, 3] + U[1, 1] * U[0, 3],
                          -U[0, 2] * U[1, 3] + U[1, 2] * U[0, 3]])
            delta = U[2, 0:3]

            J[:, i] = np.vstack((d, delta)).flatten()
        return J

    def ikine(self, T, q0=None, unit='rad'):
        """
        Calculates inverse kinematics for homogeneous transformation matrix using numerical optimisation method.
        :param T: homogeneous transformation matrix.
        :param q0: initial list of joint angles for optimisation.
        :param unit: preferred unit for returned joint angles. Allowed values: 'rad' or 'deg'.
        :return: a list of 6 joint angles.
        """
        assert T.shape == (4, 4)
        bounds = [(link.qlim[0], link.qlim[1]) for link in self]
        reach = 0
        for link in self:
            reach += abs(link.a) + abs(link.d)
        omega = np.diag([1, 1, 1, 3 / reach])
        if q0 is None:
            q0 = np.asmatrix(np.zeros((1, self.length)))

        def objective(x):
            return (
                np.square(((np.linalg.lstsq(T, self.fkine(x))[0]) - np.asmatrix(np.eye(4, 4))) * omega)).sum()

        sol = minimize(objective, x0=q0, bounds=bounds)
        if unit == 'deg':
            return np.asmatrix(sol.x * 180 / pi)
        else:
            return np.asmatrix(sol.x)


    def plot(self, stance, unit='rad'):
        """
        Plots the SerialLink object in a desired stance.
        :param stance: list of joint angles for SerialLink object.
        :param unit: unit of input angles.
        :return: null.
        """

        assert type(stance) is np.matrix

        if unit == 'deg':
            stance = stance * np.pi / 180

        self.pipeline = VtkPipeline()
        self.pipeline.reader_list, self.pipeline.actor_list, self.pipeline.mapper_list = self.__setup_pipeline_objs()

        # print("LENGHT OF ACTOR LIST: {}".format(len(self.pipeline.actor_list)))
        self.fkine(stance, apply_stance=True, actor_list=self.pipeline.actor_list)

        self.update_angles(stance.tolist()[0])

        # print(self.get_current_joint_config(unit='deg'))
        # print(self.end_effector_position())

        # for i in self.links:
        #     i.display(unit='deg')
        # self.pipeline.add_actor(axesCube(self.pipeline.ren))
        cube_axes = axesCubeFloor(self.pipeline.ren,
                                  self.param.get("cube_axes_x_bounds"),
                                  self.param.get("cube_axes_y_bounds"),
                                  self.param.get("cube_axes_z_bounds"),
                                  self.param.get("floor_position"))

        self.pipeline.add_actor(cube_axes)

        for i, each in enumerate(self.pipeline.actor_list):
            each.SetScale(self.scale)
        self.__setup_structure_display()
        self.pipeline.add_actor(self._display_path())

        xyzLabels = ['X', 'Y', 'Z']
        scale = [1.0, 1.0, 1.0]
        axes = MakeAxesActor(scale, xyzLabels)

        om2 = vtk.vtkOrientationMarkerWidget()
        om2.SetOrientationMarker(axes)
        # Position lower right in the viewport.
        om2.SetViewport(0.8, 0, 1.0, 0.2)
        om2.SetInteractor(self.pipeline.iren)
        om2.EnabledOn()

        self.pipeline.render()

    def __setup_pipeline_objs(self):
        """
        Internal function to initialise vtk objects.
        :return: reader_list, actor_list, mapper_list
        """
        print("STL FILES: {}".format(self.stl_files))
        reader_list = [0] * len(self.stl_files)
        actor_list = [0] * len(self.stl_files)
        # print("Actor List: {}".format(actor_list))

        mapper_list = [0] * len(self.stl_files)
        for i in range(len(self.stl_files)):
            reader_list[i] = vtk.vtkSTLReader()
            loc = pkg_resources.resource_filename("robopy", '/'.join(('media', self.name, self.stl_files[i])))
            # print(loc)
            reader_list[i].SetFileName(loc)
            mapper_list[i] = vtk.vtkPolyDataMapper()
            mapper_list[i].SetInputConnection(reader_list[i].GetOutputPort())
            actor_list[i] = vtk.vtkActor()
            actor_list[i].SetMapper(mapper_list[i])
            actor_list[i].GetProperty().SetColor(self.colors[i])  # (R,G,B)

        return reader_list, actor_list, mapper_list

    def __setup_structure_display(self):
        """
        Internal function to initialise vtk objects.
        :return: reader_list, actor_list, mapper_list
        """
        # reader_list = np.zeros(self.blueprint.size)
        # actor_list = np.zeros(self.blueprint.size)
        # print("Actor List: {}".format(actor_list))
        #
        # mapper_list = np.zeros(self.blueprint.size)
        # for i in range(len(self.stl_files)):

        for i in range(len(self.blueprint)):
            for j in range(len(self.blueprint[0])):
                for k in range(len(self.blueprint[0][0])):
                    if(self.blueprint[i][j][k]):
                        reader_list = vtk.vtkSTLReader()
                        loc = pkg_resources.resource_filename("robopy", '/'.join(('media', self.name, "block.stl")))
                        # print(loc)
                        reader_list.SetFileName(loc)
                        mapper_list = vtk.vtkPolyDataMapper()
                        mapper_list.SetInputConnection(reader_list.GetOutputPort())
                        actor_list = vtk.vtkActor()
                        actor_list.SetMapper(mapper_list)
                        color = vtk_named_colors(["DarkGreen"])

                        actor_list.GetProperty().SetColor(color[0])  # (R,G,B)
                        actor_list.SetScale(0.013)
                        actor_list.SetPosition((i, j, k))
                        # print("SCALE: {}".format(actor_list.GetScale()))
                        # print("POSITION: {}".format(actor_list.GetPosition()))
                        self.pipeline.add_actor(actor_list)
        return reader_list, actor_list, mapper_list

    def _add_block(self, position):
        reader_list = vtk.vtkSTLReader()
        loc = pkg_resources.resource_filename("robopy", '/'.join(('media', self.name, "block.stl")))
        # print(loc)
        reader_list.SetFileName(loc)
        mapper_list = vtk.vtkPolyDataMapper()
        mapper_list.SetInputConnection(reader_list.GetOutputPort())
        actor_list = vtk.vtkActor()
        actor_list.SetMapper(mapper_list)
        color = vtk_named_colors(["Purple"])

        actor_list.GetProperty().SetColor(color[0])  # (R,G,B)
        actor_list.SetScale(0.013)
        actor_list.SetPosition(position)
        print("Adding block at pos: {}".format(position))
        self.pipeline.add_actor(actor_list)

        return actor_list, reader_list, mapper_list

    def _display_path(self, path=[(0, 0, 0, "top"), (1, 0, 0, "top"), (2, 0, 0, "top"),(3, 0, 0, "top"), (4, 0, 0, "top"),(5, 0, 0, "top"),]):
        for point in path:
                prop_assembly = cubeForPath(point)
                self.pipeline.add_actor(prop_assembly)

    @staticmethod
    def _setup_file_names(num):
        file_names = []
        for i in range(0, num):
            file_names.append('link' + str(i) + '.stl')

        return file_names

    def animate(self, stances, unit='rad', frame_rate=25, gif=None, num_steps=None, display_path=True, obstacles=None, path=None, directions=None):
        """
        Animates SerialLink object over nx6 dimensional input matrix, with each row representing list of 6 joint angles.
        :param stances: nx6 dimensional input matrix.
        :param unit: unit of input angles. Allowed values: 'rad' or 'deg'
        :param frame_rate: frame_rate for animation. Could be any integer more than 1. Higher value runs through stances faster.
        :return: null
        """


        if unit == 'deg':
            stances = stances * np.pi / 180

        self.pipeline = VtkPipeline(total_time_steps=stances.shape[0] - 1, gif_file=gif)
        self.pipeline.reader_list, self.pipeline.actor_list, self.pipeline.mapper_list = self.__setup_pipeline_objs()
        self.fkine(stances, apply_stance=True, actor_list=self.pipeline.actor_list, directions=None)
        self.update_angles(stances.tolist()[0])

        # print(self.get_current_joint_config(unit='deg'))
        # print(self.end_effector_position())
        # self.pipeline.add_actor(axesCube(self.pipeline.ren))

        cube_axes = axesCubeFloor(self.pipeline.ren,
                                  self.param.get("cube_axes_x_bounds"),
                                  self.param.get("cube_axes_y_bounds"),
                                  self.param.get("cube_axes_z_bounds"),
                                  self.param.get("floor_position"))

        self.pipeline.add_actor(cube_axes)


        def execute(obj, event):
            nonlocal stances, obstacles
            self.pipeline.timer_tick()
            self.fkine(stances, apply_stance=True, actor_list=self.pipeline.actor_list, timer=self.pipeline.timer_count, num_steps=num_steps, directions=None)
            self.update_angles(stances[self.pipeline.timer_count].tolist()[0])

            if obstacles is not None:
                print("Obstacles is not none")
                for i in obstacles:
                    print(i)
                    print(self.pipeline.timer_count)
                    if i[0] == self.pipeline.timer_count:
                        print("Adding actor")
                        self._add_block(i[1])
                        self.pipeline.animate()

            if path is not None:
                print("Path is not none")
                for i in path:
                    print("Path Timer: {}".format(i))
                    print(self.pipeline.timer_count)
                    if i[0] == self.pipeline.timer_count:
                        print("Adding path")
                        for point in i[1]:
                            self.pipeline.add_actor(cubeForPath(point))
                        # self._add_block(i[1])
                        self.pipeline.animate()

            # self.pipeline.add_actor(cubeForPath((0, 0, 0, "top")))
            # self.pipeline.animate()
            # print(self.get_current_joint_config(unit='deg'))
            # print(self.end_effector_position())
            self.pipeline.iren = obj
            self.pipeline.iren.GetRenderWindow().Render()
            # for i, each in enumerate(self.pipeline.actor_list):
            #     each.SetScale(self.scale)

        self.pipeline.iren.AddObserver('TimerEvent', execute)
        self.__setup_structure_display()
        if display_path:
            self.pipeline.add_actor(self._display_path())

        xyzLabels = ['X', 'Y', 'Z']
        scale = [1.0, 1.0, 1.0]
        axes = MakeAxesActor(scale, xyzLabels)

        om2 = vtk.vtkOrientationMarkerWidget()
        om2.SetOrientationMarker(axes)
        # Position lower right in the viewport.
        om2.SetViewport(0.8, 0, 1.0, 0.2)
        om2.SetInteractor(self.pipeline.iren)
        om2.EnabledOn()
        om2.InteractiveOn()

        self.pipeline.animate()

    # def get_end_effector_pos(self, stance):
    #     return self.fkine(stance)

    def flip_base(self, ee_pos, direction, value):
        if direction == "top" or direction == "bottom":
            new_base = tr.trotz(value, unit="deg", xyz=ee_pos)

        if direction == "left" or direction == "right":
            new_base = tr.troty(value, unit="deg", xyz=ee_pos)

        if direction == "front" or direction == "back":
            new_base = tr.trotx(value, unit="deg", xyz=ee_pos)
        return new_base

class Link(ABC):
    """
    Link object class.
    """

    def __init__(self, j, theta, d, a, alpha, length, offset=None, kind='', mdh=0, flip=None, qlim=None):
        """
        initialises the link object.
        :param j:
        :param theta:
        :param d:
        :param a:
        :param alpha:
        :param offset:
        :param kind: 'r' or 'p' as input. 'r' for Revolute. 'p' for Prismatic.
        :param mdh:
        :param flip:
        :param qlim:
        """
        self.theta = theta
        self.d = d
        # self.j = j
        self.a = a
        self.alpha = alpha
        self.offset = offset
        self.kind = kind
        self.mdh = mdh
        self.flip = flip
        self.qlim = qlim
        self.length = length

        self.max_velocity = 0

        self.set_theta(theta)
        self.velocity = 0  # Link's current velocity

    def set_theta(self, theta):
        """Sets theta to the new theta and computes the new
        transformation matrix

        :param theta: The new theta for the link
        :type theta: int

        :rtype: None
        """
        self.theta = theta
        self.transform_matrix = self.A(theta)

    def update_velocity(self, accel, time):
        """Updates the current velocity of the link when acted upon
        by some acceleration over some time

        :param accel: The acceleration acting upon the link
                      (radians per second^2)
        :type accel: int

        :param time: The time the accelration is applied over (seconds)
        :type time: int

        :rtype: None
        """
        new_velocity = self.velocity + (accel * time)
        if new_velocity <= self.max_velocity:
            self.velocity = new_velocity
            new_theta = self.theta + (new_velocity * time)
            new_theta = math.atan2(math.sin(new_theta),
                                   math.cos(new_theta))
            self.set_theta(new_theta)


    def display(self, unit='rad'):
        """Display the link's properties nicely

        :rtype: None
        """
        angle = self.theta
        if unit == 'deg':
            angle = angle * 180 / pi
        print('Link angle: {}'.format(angle))
        print('Link length: {}'.format(self.length))


    def A(self, q):
        sa = math.sin(self.alpha)
        ca = math.cos(self.alpha)
        if self.flip:
            q = -q + self.offset
        else:
            q = q + self.offset
        st = 0
        ct = 0
        d = 0
        if self.kind == 'r':
            st = math.sin(q)
            ct = math.cos(q)
            d = self.d
        elif self.kind == 'p':
            st = math.sin(self.theta)
            ct = math.cos(self.theta)
            d = q

        se3_np = 0
        if self.mdh == 0:
            se3_np = np.matrix([[ct, -st * ca, st * sa, self.a * ct],
                                [st, ct * ca, -ct * sa, self.a * st],
                                [0, sa, ca, d],
                                [0, 0, 0, 1]])

        return se3_np


class Revolute(Link):
    """
    Revolute object class.
    """

    def __init__(self, j, theta, d, a, alpha, offset, qlim, length):
        """
        Initialised revolute object.
        :param j:
        :param theta:
        :param d:
        :param a:
        :param alpha:
        :param offset:
        :param qlim:
        """
        super().__init__(j=j, theta=theta, d=d, a=a, alpha=alpha, offset=offset, kind='r', qlim=qlim, length=length)
        pass


class Prismatic(Link):
    """
    Prismatic object class.
    """

    def __init__(self, j, theta, d, a, alpha, offset, qlim, length):
        """
        Initialises prismatic object.
        :param j:
        :param theta:
        :param d:
        :param a:
        :param alpha:
        :param offset:
        :param qlim:
        """
        super().__init__(j=j, theta=theta, d=d, a=a, alpha=alpha, offset=offset, kind='p', qlim=qlim,length=length)
        pass

    pass
