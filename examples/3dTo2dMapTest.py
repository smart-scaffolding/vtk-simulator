import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


blueprint = np.array([
    [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
    [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
    [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
    [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
    [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
    [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
])


"""
top layer: blueprint[:, :, -1]
bottom layer: blueprint[:, :, 0]

back layer: blueprint[:, -1, :]
front layer: blueprint[:, 0, :]

left layer: blueprint[0, :, :]
right layer: blueprint[-1, :, :]
"""


def display_blueprint(blueprint):
    fig = plt.figure(figsize=(12, 12))
    ax = Axes3D(fig)
    ax.voxels(blueprint, facecolors='green', edgecolors='gray', zorder=0)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_zlim(0, 10)
    plt.show()

if __name__ == '__main__':
    display_blueprint(blueprint)