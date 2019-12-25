from scipy.spatial import cKDTree
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random

data = []

for x in range(10):
    for y in range(10):
        for z in range(10):
            data.append([x, y, z])
            # print(f"Adding item: {x}-{y}-{z}")


tree = cKDTree(data)

WIDTH = 10
LENGTH = 10
HEIGHT = 10

colors = np.array([[['blue']*HEIGHT]*WIDTH]*LENGTH)
grid = np.array([[[1]*HEIGHT]*WIDTH]*LENGTH)
#
# dd, ii = tree.query([0, 0, 0], k=100, distance_upper_bound=10)
#
#
# for val in ii:
#     x = int(tree.data[val][0])
#     y = int(tree.data[val][1])
#     z = int(tree.data[val][2])
#     print(val)
#     print(x,y, z)
#     colors[x, y, z] = 'red'
#     print(tree.data[val])
#
# dd, ii = tree.query([9, 9, 9], k=100, distance_upper_bound=10)
#
#
# for val in ii:
#     x = int(tree.data[val][0])
#     y = int(tree.data[val][1])
#     z = int(tree.data[val][2])
#     print(val)
#     print(x,y, z)
#     colors[x, y, z] = 'g'
#     print(tree.data[val])
#
#
# # grid = np.asarray(grid)
# print(colors)
# print(tree.data)

NUM_ROBOTS = 1
THRESHOLD = 2

x, y, z = grid.shape

select_colors = ['r', 'g', 'k', 'b']
count = 0

# max = "X"
if z > y and z > x:
    maxDirection = "Z"
elif y > x and y > z:
    maxDirection = "Y"
else:
    maxDirection = "X"

increment = int(max(x, y, z)/NUM_ROBOTS)
print(f"Increment: {increment}")
if increment <= THRESHOLD:
    raise Exception("Too many robots, please try lowering the number of robots")
# colors[0:x_increment, :, :] = select_colors[0]
# colors[x_increment:x_increment*2, :, :] = select_colors[1]
# colors[x_increment*2:x_increment*3, :, :] = select_colors[2]
# colors[x_increment*3:, :, :] = select_colors[3]

start = 0
end = increment
scale = 1
for robot in range(NUM_ROBOTS):
    if robot == NUM_ROBOTS-1:
        end=None
    print(start, end, scale)
    if maxDirection == "X":
        colors[start:end, :, :] = select_colors[robot]
    if maxDirection == "Y":
        colors[:, start:end, :] = select_colors[robot]
    if maxDirection == "Z":
        colors[:, :, start:end] = select_colors[robot]

    start = end
    scale += 1
    end = increment * scale



# while(z <= HEIGHT):
#     count = 0
#     while(x <= WIDTH):
#         # count = 0
#         while(y <= LENGTH):
#             # print(x, y, z)
#             for xx in range(x-NUM_ROBOTS, x):
#                 for yy in range(y-NUM_ROBOTS, y):
#                     for zz in range(z-NUM_ROBOTS, z):
#                         if count == 0:
#                             index = 0
#                         elif count >= len(select_colors):
#                             index = (len(select_colors) % count)
#                         else:
#                             index = count
#
#                         print(f"INDEX: {index}")
#                         colors[xx, yy, zz] = select_colors[index]
#                         # print(xx, yy, zz)
#                         print(count)
#
#             y += NUM_ROBOTS
#             count += 1
#         x += NUM_ROBOTS
#         y = NUM_ROBOTS
#         count +=1
#     z += NUM_ROBOTS
#     x = NUM_ROBOTS
#     y = NUM_ROBOTS
#     count +=1
#
#
#
# # print(x, y, z)
# for xx in range(x-NUM_ROBOTS, WIDTH%NUM_ROBOTS):
#     for yy in range(y-NUM_ROBOTS, LENGTH%NUM_ROBOTS):
#         for zz in range(z-NUM_ROBOTS, HEIGHT%NUM_ROBOTS):
#             colors[xx, yy, zz] = 'k'
            # print(xx, yy, zz)

# print(colors)




fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(grid, facecolors=colors, edgecolor='k')
plt.show()

