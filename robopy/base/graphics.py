# Created by: Aditya Dua
# 18 August 2017
import pkg_resources
import vtk
import math
import numpy as np


class VtkPipeline:
    def __init__(self, background=(0.15, 0.15, 0.15), total_time_steps=None, timer_rate=60, gif_file=None):
        self.ren = vtk.vtkRenderer()
        self.ren.SetBackground(background[0], background[1], background[2])
        self.ren_win = vtk.vtkRenderWindow()
        self.ren_win.AddRenderer(self.ren)
        self.ren_win.SetSize((3000, 3000))
        self.iren = vtk.vtkRenderWindowInteractor()
        self.iren.SetRenderWindow(self.ren_win)
        self.iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())

        # xyzLabels = ['X', 'Y', 'Z']
        # scale = [1.0, 1.0, 1.0]
        # axes = MakeAxesActor(scale, xyzLabels)
        #
        # om2 = vtk.vtkOrientationMarkerWidget()
        # om2.SetOrientationMarker(axes)
        # # Position lower right in the viewport.
        # om2.SetViewport(0.8, 0, 1.0, 0.2)
        # om2.SetInteractor(self.iren)
        # om2.EnabledOn()
        # om2.InteractiveOn()


        self.actor_list = []
        self.mapper_list = []
        self.source_list = []
        self.screenshot_count = 0
        self.timer_rate = timer_rate
        self.gif_data = []

        if gif_file is not None:
            try:
                assert type(gif_file) is str
            except AssertionError:
                gif_file = str(gif_file)
            self.gif_file = gif_file
        else:
            self.gif_file = None

        if total_time_steps is not None:
            assert type(total_time_steps) is int
            self.timer_count = 0
            self.total_time_steps = total_time_steps

    def render(self, ui=True):
        for each in self.actor_list:
            self.ren.AddActor(each)
        self.ren.ResetCamera()
        self.set_camera()
        self.ren_win.Render()
        self.ren_win.SetWindowName('Robot Simulation')
        if ui:
            self.iren.Initialize()
            self.iren.Start()

    def add_actor(self, actor):
        self.actor_list.append(actor)

    def set_camera(self):
        cam = self.ren.GetActiveCamera()
        # cam.Roll(90)

        # cam.SetFocalPoint(0, 0, 0)
        # cam.Pitch(90)



        # cam.SetViewUp(1, 1, 0)
        # cam.Yaw(-90)
        # cam.Elevation(-90)
        # cam.Zoom(0.6)
        # cam.SetPosition(-1, -3, 1.3)
        self.ren.SetActiveCamera(cam)

    def animate(self):
        self.ren.ResetCamera()
        # self.set
        self.ren_win.Render()
        self.iren.Initialize()
        self.iren.CreateRepeatingTimer(math.floor(1000 / self.timer_rate))  # Timer creates 60 fps
        self.render()

    def screenshot(self, filename=None):
        w2if = vtk.vtkWindowToImageFilter()
        w2if.SetInput(self.ren_win)
        w2if.Update()
        if filename is None:
            filename = 'screenshot'
        filename = filename + '%d.png' % self.screenshot_count
        writer = vtk.vtkPNGWriter()
        writer.SetFileName(filename)
        self.screenshot_count += 1
        writer.SetInputData(w2if.GetOutput())
        writer.Write()

    def timer_tick(self):
        import imageio
        self.timer_count += 1

        if self.timer_count >= self.total_time_steps:
            self.iren.DestroyTimer()
            if self.gif_file is not None:
                assert len(self.gif_data) > 0
                imageio.mimsave(self.gif_file + '.gif', self.gif_data)
                import os
                for i in range(self.screenshot_count):
                    os.remove(self.gif_file + '%d.png' % i)
                return

        if self.gif_file is not None:
            if (self.timer_count % 60) == 0:
                self.screenshot(self.gif_file)
                path = self.gif_file + '%d.png' % (self.screenshot_count - 1)
                self.gif_data.append(imageio.imread(path))


def axesUniversal():
    axes_uni = vtk.vtkAxesActor()
    axes_uni.SetXAxisLabelText("x'")
    axes_uni.SetYAxisLabelText("y'")
    axes_uni.SetZAxisLabelText("z'")
    axes_uni.SetTipTypeToSphere()
    axes_uni.SetShaftTypeToCylinder()
    axes_uni.SetTotalLength(2, 2, 2)
    axes_uni.SetCylinderRadius(0.02)
    axes_uni.SetAxisLabels(0)

    return axes_uni


def axesCube(ren, x_bound=np.matrix([[-1.5, 1.5]]), y_bound=np.matrix([[-1.5, 1.5]]), z_bound=np.matrix([[-1.5, 1.5]])):
    cube_axes_actor = vtk.vtkCubeAxesActor()
    cube_axes_actor.SetBounds(x_bound[0, 0], x_bound[0, 1], y_bound[0, 0], y_bound[0, 1], z_bound[0, 0], z_bound[0, 1])
    cube_axes_actor.SetCamera(ren.GetActiveCamera())
    cube_axes_actor.GetTitleTextProperty(0).SetColor(1.0, 0.0, 0.0)
    cube_axes_actor.GetLabelTextProperty(0).SetColor(1.0, 0.0, 0.0)

    cube_axes_actor.GetTitleTextProperty(1).SetColor(0.0, 1.0, 0.0)
    cube_axes_actor.GetLabelTextProperty(1).SetColor(0.0, 1.0, 0.0)

    cube_axes_actor.GetTitleTextProperty(2).SetColor(0.0, 0.0, 1.0)
    cube_axes_actor.GetLabelTextProperty(2).SetColor(0.0, 0.0, 1.0)

    # cube_axes_actor.XAxisMinorTickVisibilityOff()
    # cube_axes_actor.YAxisMinorTickVisibilityOff()
    # cube_axes_actor.ZAxisMinorTickVisibilityOff()

    cube_axes_actor.SetFlyModeToStaticTriad()

    return cube_axes_actor


def axes_x_y(ren):
    axis_x_y = axesCube(ren)
    axis_x_y.SetUse2DMode(1)
    axis_x_y.ZAxisLabelVisibilityOff()
    axis_x_y.SetAxisOrigin(-3, -3, 0)
    axis_x_y.SetUseAxisOrigin(1)

    return axis_x_y


def axesActor2d():
    axes = vtk.vtkAxesActor()
    axes.SetTotalLength(1, 1, 0)
    axes.SetZAxisLabelText("")

    return axes


def vtk_named_colors(colors):
    """
    Returns a list of vtk colors
    :param colors: List of color names supported by vtk
    :return: A list of vtk colors
    """
    if type(colors) is not list:
        colors = [colors]
    colors_rgb = [0] * len(colors)
    for i in range(len(colors)):
        colors_rgb[i] = list(vtk.vtkNamedColors().GetColor3d(colors[i]))
    return colors_rgb


def floor():
    plane = vtk.vtkPlaneSource()
    reader = vtk.vtkJPEGReader()
    reader.SetFileName(pkg_resources.resource_filename("robopy", "media/imgs/floor.jpg"))
    texture = vtk.vtkTexture()
    texture.SetInputConnection(reader.GetOutputPort())
    map_to_plane = vtk.vtkTextureMapToPlane()
    map_to_plane.SetInputConnection(plane.GetOutputPort())
    mapper = vtk.vtkPolyDataMapper()

    mapper.SetInputConnection(map_to_plane.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.SetTexture(texture)
    return actor


def axesCubeFloor(ren, x_bound=np.matrix([[-1.5, 1.5]]),
                  y_bound=np.matrix([[-1.5, 1.5]]),
                  z_bound=np.matrix([[-1.5, 1.5]]),
                  position=np.matrix([[0, -1.5, 0]])):
    axes = axesCube(ren, x_bound=x_bound, y_bound=y_bound, z_bound=z_bound)
    # flr = floor()
    # flr.RotateX(90)
    # flr.SetPosition(position[0, 0], position[0, 1], position[0, 2])
    # flr.SetScale(3)
    assembly = vtk.vtkAssembly()
    # assembly.AddPart(flr)
    assembly.AddPart(axes)
    return assembly

def cubeForPath(point):
    colors = vtk.vtkNamedColors()
    prop_assembly = vtk.vtkPropAssembly()

    direction = point[3]
    cube_source = vtk.vtkCubeSource()
    cube_source.SetCenter((point[0] + 0.5, point[1] + 0.5, point[2] + 0.5))
    cube_source.Update()
    face_colors = vtk.vtkUnsignedCharArray()
    face_colors.SetNumberOfComponents(3)

    if direction == "top":

        face_x_plus = colors.GetColor3ub('Red')
        face_x_minus = colors.GetColor3ub('Red')
        face_y_plus = colors.GetColor3ub('Red')
        face_y_minus = colors.GetColor3ub('Red')
        face_z_plus = colors.GetColor3ub('Cyan')
        face_z_minus = colors.GetColor3ub('Red')

    if direction == "bottom":

        face_x_plus = colors.GetColor3ub('Red')
        face_x_minus = colors.GetColor3ub('Red')
        face_y_plus = colors.GetColor3ub('Red')
        face_y_minus = colors.GetColor3ub('Red')
        face_z_plus = colors.GetColor3ub('Red')
        face_z_minus = colors.GetColor3ub('Cyan')

    if direction == "right":

        face_x_plus = colors.GetColor3ub('Cyan')
        face_x_minus = colors.GetColor3ub('Red')
        face_y_plus = colors.GetColor3ub('Red')
        face_y_minus = colors.GetColor3ub('Red')
        face_z_plus = colors.GetColor3ub('Red')
        face_z_minus = colors.GetColor3ub('Red')

    if direction == "left":

        face_x_plus = colors.GetColor3ub('Red')
        face_x_minus = colors.GetColor3ub('Cyan')
        face_y_plus = colors.GetColor3ub('Red')
        face_y_minus = colors.GetColor3ub('Red')
        face_z_plus = colors.GetColor3ub('Red')
        face_z_minus = colors.GetColor3ub('Red')

    if direction == "front":

        face_x_plus = colors.GetColor3ub('Red')
        face_x_minus = colors.GetColor3ub('Red')
        face_y_plus = colors.GetColor3ub('Red')
        face_y_minus = colors.GetColor3ub('Cyan')
        face_z_plus = colors.GetColor3ub('Red')
        face_z_minus = colors.GetColor3ub('Red')

    if direction == "back":

        face_x_plus = colors.GetColor3ub('Red')
        face_x_minus = colors.GetColor3ub('Red')
        face_y_plus = colors.GetColor3ub('Cyan')
        face_y_minus = colors.GetColor3ub('Red')
        face_z_plus = colors.GetColor3ub('Red')
        face_z_minus = colors.GetColor3ub('Red')

    face_colors.InsertNextTypedTuple(face_x_minus)
    face_colors.InsertNextTypedTuple(face_x_plus)
    face_colors.InsertNextTypedTuple(face_y_minus)
    face_colors.InsertNextTypedTuple(face_y_plus)
    face_colors.InsertNextTypedTuple(face_z_minus)
    face_colors.InsertNextTypedTuple(face_z_plus)

    cube_source.GetOutput().GetCellData().SetScalars(face_colors)
    cube_source.Update()

    cube_mapper = vtk.vtkPolyDataMapper()
    cube_mapper.SetInputData(cube_source.GetOutput())
    cube_mapper.Update()

    cube_actor = vtk.vtkActor()
    cube_actor.SetMapper(cube_mapper)

    # Assemble the colored cube and annotated cube texts into a composite prop.
    prop_assembly = vtk.vtkPropAssembly()
    prop_assembly.AddPart(cube_actor)
    return prop_assembly

def MakeAxesActor(scale, xyzLabels):
    axes = vtk.vtkAxesActor()
    axes.SetScale(scale[0], scale[1], scale[2])
    axes.SetShaftTypeToCylinder()
    axes.SetXAxisLabelText(xyzLabels[0])
    axes.SetYAxisLabelText(xyzLabels[1])
    axes.SetZAxisLabelText(xyzLabels[2])
    axes.SetCylinderRadius(0.5 * axes.GetCylinderRadius())
    axes.SetConeRadius(1.025 * axes.GetConeRadius())
    axes.SetSphereRadius(1.5 * axes.GetSphereRadius())
    tprop = axes.GetXAxisCaptionActor2D().GetCaptionTextProperty()
    tprop.ItalicOn()
    tprop.ShadowOn()
    tprop.SetFontFamilyToTimes()
    # Use the same text properties on the other two axes.
    axes.GetYAxisCaptionActor2D().GetCaptionTextProperty().ShallowCopy(tprop)
    axes.GetZAxisCaptionActor2D().GetCaptionTextProperty().ShallowCopy(tprop)
    return axes