# Created by: Aditya Dua
# 5 October 2017
from .serial_link import SerialLink
from .serial_link import Revolute
from math import pi
import numpy as np
from . import transforms as tr
from . import graphics
from .common import ishomog


class Puma560(SerialLink):
    def __init__(self, base=None):

        self.qn = np.matrix([[0, pi / 4, pi, 0, pi / 4, 0]])
        self.qr = np.matrix([[0, pi / 2, -pi / 2, 0, 0, 0]])
        self.qz = np.matrix([[0, 0, 0, 0, 0, 0]])
        self.qs = np.matrix([[0, 0, -pi / 2, 0, 0, 0]])
        self.scale = 1
        param = {
            "cube_axes_x_bounds": np.matrix([[-1.5, 1.5]]),
            "cube_axes_y_bounds": np.matrix([[-0.7, 1.5]]),
            "cube_axes_z_bounds": np.matrix([[-1.5, 1.5]]),
            "floor_position": np.matrix([[0, -0.7, 0]])
        }

        links = [Revolute(d=0, a=0, alpha=pi / 2, j=0, theta=0, offset=0, qlim=(-160 * pi / 180, 160 * pi / 180)),
                 Revolute(d=0, a=0.4318, alpha=0, j=0, theta=0, offset=0, qlim=(-45 * pi / 180, 225 * pi / 180)),
                 Revolute(d=0.15005, a=0.0203, alpha=-pi / 2, j=0, theta=0, offset=0,
                          qlim=(-225 * pi / 180, 45 * pi / 180)),
                 Revolute(d=0.4318, a=0, alpha=pi / 2, j=0, theta=0, offset=0, qlim=(-110 * pi / 180, 170 * pi / 180)),
                 Revolute(d=0, a=0, alpha=-pi / 2, j=0, theta=0, offset=0, qlim=(-100 * pi / 180, 100 * pi / 180)),
                 Revolute(d=0, a=0, alpha=0, j=0, theta=0, offset=0, qlim=(-226 * pi / 180, 226 * pi / 180))]

        if base is None:
            base = tr.trotx(-90, unit='deg')
        else:
            assert ishomog(base, (4, 4))
        file_names = SerialLink._setup_file_names(7)
        colors = graphics.vtk_named_colors(["Red", "DarkGreen", "Blue", "Cyan", "Magenta", "Yellow", "White"])

        super().__init__(links=links, base=base, name='puma_560', stl_files=file_names, colors=colors, param=param)

class Puma560_TEST(SerialLink):
    def __init__(self, base=None, blueprint=None):

        # self.qn = np.matrix([[0, pi / 4, pi, 0, pi / 4, 0]])
        # self.qr = np.matrix([[0, pi / 2, -pi / 2, 0, 0, 0]])
        # self.qz = np.matrix([[0, 0, 0, 0, 0, 0]])
        # self.qs = np.matrix([[0, 0, -pi / 2, 0, 0, 0]])
        # self.scale = 0.1
        param = {
            "cube_axes_x_bounds": np.matrix([[0, len(blueprint)]]),
            "cube_axes_y_bounds": np.matrix([[0, len(blueprint[0])]]),
            "cube_axes_z_bounds": np.matrix([[0, len(blueprint[0][0])]]),
            "floor_position": np.matrix([[0, 0, 0]])
        }

#4.125 inches
#6.429 inches

        seg_lens = np.array([1.04775, 1.632966, 1.632966, 1.04775])*1.
        # seg_lens = np.array([1.375, 2.143, 1.375, 2.143])

        links = [Revolute(d=seg_lens[0], a=0, alpha=pi/2, j=0, theta=0, offset=0, qlim=(0, 0), length=seg_lens[0]),
                 Revolute(d=0, a=seg_lens[1], alpha=0, j=0, theta=0, offset=0, qlim=(0, 0), length=seg_lens[1]),
                 Revolute(d=0, a=seg_lens[2], alpha=0, j=0, theta=0, offset=0, qlim=(-180 * pi / 180, 180 * pi / 180), length=seg_lens[2]),
                 Revolute(d=0, a=seg_lens[3], alpha=0, j=0, theta=0, offset=0, qlim=(-180 * pi / 180, 180 * pi / 180), length=seg_lens[3]),
                 ]

        if base is None:
            base = tr.trotx(-90, unit='deg')
        else:
            assert ishomog(base, (4, 4))
        file_names = SerialLink._setup_file_names(4)
        # colors = graphics.vtk_named_colors(["Red", "DarkGreen", "Blue", "Cyan"])
        colors = graphics.vtk_named_colors(["Red", "Blue", "Blue", "Purple"])


        super().__init__(links=links, base=base, name='puma_560_TEST', stl_files=file_names, colors=colors, param=param, blueprint=blueprint)


class Orion5(SerialLink):
    def __init__(self, base=None):

        self.qz = np.matrix([[0, 0, 0, 0, 0, 0]])
        self.scale = 0.01
        # Pre-defined Stances
        # TODO

        # Turret, Shoulder, Elbow, Wrist, Claw
        links = [Revolute(d=0, a=0, alpha=0, j=0, theta=0, offset=0, qlim=0),  # Turret
                 Revolute(d=0.53, a=-0.30309, alpha=0, j=0, theta=0, offset=0, qlim=0),  # Shoulder
                 Revolute(d=0, a=-1.70384, alpha=0, j=0, theta=0, offset=0, qlim=0),  # Elbow
                 Revolute(d=0, a=-1.36307, alpha=0, j=0, theta=0, offset=0, qlim=0),  # Wrist
                 Revolute(d=0, a=0, alpha=0, j=0, theta=0, offset=0, qlim=0),
                 Revolute(d=0, a=0, alpha=0, j=0, theta=0, offset=0, qlim=0)]

        if base is None:
            base = tr.trotx(-90, unit='deg')
        else:
            assert ishomog(base, (4, 4))

        file_names = SerialLink._setup_file_names(7)
        colors = graphics.vtk_named_colors(
            ["DimGray", "IndianRed", "DimGray", "IndianRed", "DimGray", "IndianRed", "IndianRed"])

        super().__init__(links=links, base=base, name='orion5', stl_files=file_names, colors=colors)

    # def __assemble_parts(self):
    #     # Assemblies-----------------------
    #     # Biceps
    #     bicep = vtk.vtkAssembly()
    #     for i in range(14, 17):
    #         bicep.AddPart(actors[i])
    #
    #     # Forearm
    #     forearm = vtk.vtkAssembly()
    #     forearm.AddPart(actors[17])
    #     forearm.AddPart(actors[18])
    #
    #     # Wrist
    #     wrist = vtk.vtkAssembly()
    #     # wrist.AddPart(actors[16])
    #     wrist.AddPart(actors[23])
    #     wrist.AddPart(actors[24])
    #
    #     # Claw
    #     claw = vtk.vtkAssembly()
    #     for i in range(19, 23):
    #         claw.AddPart(actors[i])
    #
    #     # Shoulder
    #     shoulder = actors[5]
    #
    #     # Base Servo
    #     base_servo = actors[6]
    #
    #     # Gripper
    #     # 1
    #     gripper1 = vtk.vtkAssembly()
    #     for i in range(27, 29):
    #         gripper1.AddPart(actors[i])
    #
    #     # 2
    #     gripper2 = vtk.vtkAssembly()
    #     for i in range(25, 27):
    #         gripper2.AddPart(actors[i])
    #
    #     # Base with buttons
    #     base = vtk.vtkAssembly()
    #     for i in range(5):
    #         base.AddPart(actors[i])
    #
    #     # Turret
    #     turret = vtk.vtkAssembly()
    #     for i in range(7, 13):
    #         actors[i].GetProperty().SetColor(vtk.vtkNamedColors().GetColor3d("BlanchedAlmond"))
    #         turret.AddPart(actors[i])
    #         # ------------------------------------
