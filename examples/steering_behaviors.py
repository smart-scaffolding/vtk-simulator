import numpy as np
# from .graphics import VtkPipeline
# from .graphics import axesCubeFloor
# from .graphics import MakeAxesActor
# from .graphics import setup_structure_display
from robopy.base.graphics import *
import time
import vtk

class SteeringBehaviors:
    def __init__(self, blueprint, time_steps=3000, gif=None):
        self.pipeline = VtkPipeline(total_time_steps=time_steps, gif_file=gif)
        self.param = {
            "cube_axes_x_bounds": np.matrix([[0, len(blueprint)]]),
            "cube_axes_y_bounds": np.matrix([[0, len(blueprint[0])]]),
            "cube_axes_z_bounds": np.matrix([[0, len(blueprint[0][0])]]),
            "floor_position": np.matrix([[0, 0, 0]])
        }
        self.blueprint = blueprint
        self.structure = None
        cube_axes = axesCubeFloor(self.pipeline.ren,
                                  self.param.get("cube_axes_x_bounds"),
                                  self.param.get("cube_axes_y_bounds"),
                                  self.param.get("cube_axes_z_bounds"),
                                  self.param.get("floor_position"))

        self.pipeline.add_actor(cube_axes)
        self.structure_actors = None
        self.NUM_ROBOTS = 10
        self.THRESHOLD = 2
        self.seek_positions = np.zeros((10, 3))
        self.create_positions()

    def create_positions(self, positions=[[0, 0, 1], [1, 0, 1], [2, 0, 1]]):
        for index in range(len(positions)-1):
            new_x = np.linspace(positions[index][0], positions[index + 1][0], 100, endpoint=True)
            new_y = np.linspace(positions[index][1], positions[index + 1][1], 100, endpoint=True)
            new_z = np.linspace(positions[index][2], positions[index + 1][2], 100, endpoint=True)
            new_positions = np.stack((new_x, new_y, new_z), axis=1)
            if len(self.seek_positions) > 1:
                # print(self.seek_positions)
                self.seek_positions = np.concatenate((self.seek_positions, new_positions))
            else:
                self.seek_positions = new_positions

    def execute(self, obj, event):

        self.pipeline.timer_tick()
        timer = self.pipeline.timer_count
        # print(timer)
        # print(self.seek_positions, end='\n\n\n')
        if timer % 5 == 0:
            if len(self.seek_positions) > 0:
                target = self.seek_positions[-1]
                print(target)
                for bot in self.robots:
                    # self.pipeline.remove_actor(bot.actor)

                    bot.seek(target)
                    self.pipeline.actor_list[bot.index].SetPosition(bot.position)
                    # self.pipeline.add_actor(bot.actor)
                    print(bot.position)
                    print("Updated bot position")
        self.pipeline.animate()
        self.pipeline.iren = obj
        self.pipeline.iren.GetRenderWindow().Render()



    def act(self):

        self.robots = []
        for bot in range(self.NUM_ROBOTS):
            self.robots.append(Vehicle(np.array([0, 0, 1]), index=bot))

        for bot in self.robots:
            self.pipeline.add_actor(bot.actor)

        self.seek_positions = self.seek_positions.tolist()

        self.pipeline.iren.AddObserver('TimerEvent', self.execute)


        # if display_path:
        #     self.pipeline.add_actor(self._display_path())

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


class Vehicle:
    def __init__(self, position, index, velocity=np.array([0, 1, 0]), acceleration=np.array([0, 0, 0]), r=6,
                 maxspeed=2,
                 maxforce=0.1):
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration

        self.r = r
        self.maxspeed=maxspeed
        self.maxforce=maxforce
        self.actor = self.create_vehicle()
        self.index = index

    def update(self):
        self.velocity = np.add(self.velocity,self.acceleration)
        np.putmask(self.velocity, self.velocity >= self.maxspeed, self.maxspeed)

        self.position = np.add(self.position, self.velocity)
        self.acceleration=np.array([0, 0, 0])

    def applyForce(self, force):
        self.acceleration = np.add(self.acceleration, force)
        self.update()

    def seek(self, target):
        desired = np.subtract(target, self.position)

        norm = np.linalg.norm(desired)
        new_vx = desired[0] * self.maxspeed / norm
        new_vy = desired[1] * self.maxspeed / norm
        new_vz = desired[2] * self.maxspeed / norm
        desired = [new_vx, new_vy, new_vz]

        steer =np.subtract(desired, self.velocity)
        np.putmask(steer, steer >= self.maxforce, self.maxforce)

        self.applyForce(steer)

    def create_vehicle(self):
        colors = vtk.vtkNamedColors()
        prop_assembly = vtk.vtkPropAssembly()

        source = vtk.vtkSphereSource()

        source.SetCenter(self.position)
        source.SetRadius(0.25)
        # source.SetRadius(0.75)

        source.Update()

        circle_mapper = vtk.vtkPolyDataMapper()
        circle_mapper.SetInputData(source.GetOutput())
        circle_mapper.Update()

        circle_actor = vtk.vtkActor()
        circle_actor.SetMapper(circle_mapper)
        circle_actor.GetProperty().SetColor(colors.GetColor3ub('Red'))  # Color red

        return circle_actor

if __name__ == '__main__':
    blueprint = np.array([
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
    ])

    steering = SteeringBehaviors(blueprint)
    steering.act()


