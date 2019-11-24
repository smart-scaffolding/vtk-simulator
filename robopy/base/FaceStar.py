import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# import matplotlib as mpl
# import pandas as pd
# from collections import OrderedDict
import numpy as np
import heapq

# note: variable ending with "face" is the coordinate of a face; variable ending with "idx" is a face label of a face

blockWidth = 0.49  # it is not 0.5 cuz this would make it a lot easier to calculate which block a face belongs to


class BlockFace:

    def __init__(self, xPos, yPos, zPos, face):
        self.xPos = xPos
        self.yPos = yPos
        self.zPos = zPos
        self.face = face

    def get_face_coordinate(self):
        coordinate = [self.xPos, self.yPos, self.zPos]
        if self.face == 'front':
            coordinate[1] = coordinate[1] - blockWidth
        elif self.face == 'back':
            coordinate[1] = coordinate[1] + blockWidth
        elif self.face == 'left':
            coordinate[0] = coordinate[0] - blockWidth
        elif self.face == 'right':
            coordinate[0] = coordinate[0] + blockWidth
        elif self.face == 'top':
            coordinate[2] = coordinate[2] + blockWidth
        elif self.face == 'bottom':
            coordinate[2] = coordinate[2] - blockWidth
        else:
            return None
        return tuple(coordinate)


class FaceStar:
    def __init__(self, startFace, goalFace, blueprint, armReach):

        self.startFace = startFace.get_face_coordinate()
        if not self.startFace:
            raise Exception("Start face is invalid")
        self.goalFace = goalFace.get_face_coordinate()
        if not self.goalFace:
            raise Exception("Goal face is invalid")

        self.path = []  # this path stores the faces as coordinates
        self.bp = blueprint
        self.building_dimensions = self.bp.shape
        self.armReach = armReach  # first element: arm reach in same face situation; second element: arm reach in different face situation
        print("\nBuilding Dimensions: {}\n".format(self.building_dimensions))
        self.colors = np.array(
            [[[(0, 0, 1, 0.3)] * self.building_dimensions[2]] * self.building_dimensions[1]] * self.building_dimensions[
                0], )

        # self.logger = logging.getLogger('PathPlanning')

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    # array: the current structure, startFace: the start face, goalFace: the goal face
    def faceStar(self, startFace, goalFace):

        neighbors = [(0, 1, 0), (0, 1, -1), (0, 1, 1),
                     (0, -1, 0), (0, -1, -1), (0, -1, 1),
                     (1, 0, 0), (1, 0, -1), (1, 0, 1),
                     (-1, 0, 0), (-1, 0, -1), (-1, 0, 1),
                     (0, 0, 0), (0, 0, 1), (0, 0, -1),
                     (1, 1, 0), (1, 1, -1), (1, 1, 1),
                     (1, -1, 0), (1, -1, -1), (1, -1, 1),
                     (-1, 1, 0), (-1, 1, -1), (-1, 1, 1),
                     (-1, -1, 0), (-1, -1, -1), (-1, -1, 1)
                     ]

        neighborFaces = [(0, 1, 0),
                         (0, -1, 0),
                         (1, 0, 0),
                         (-1, 0, 0),
                         (0, 0, 1),
                         (0, 0, -1)
                         ]

        close_set = set()
        came_from = {}
        gscore = {startFace: 0}
        fscore = {startFace: self.heuristic(startFace, goalFace)}
        oheap = []

        heapq.heappush(oheap, (fscore[startFace], startFace))

        while oheap:

            currentFace = heapq.heappop(oheap)[1]

            if currentFace == goalFace:
                data = []
                while currentFace in came_from:
                    pathNode = self.parse_output(currentFace)
                    data.append(pathNode)
                    self.path.append(currentFace)
                    currentFace = came_from[currentFace]
                data.append(self.parse_output(self.startFace))
                self.path.append(self.startFace)
                return data

            close_set.add(currentFace)
            for i, j, k in neighbors:
                currentBlock = self.get_block_idx(currentFace)
                neighbor = currentBlock[0] + i, currentBlock[1] + j, currentBlock[2] + k
                # for each available neighbor
                if self.within_range_huh(neighbor[0], neighbor[1], neighbor[2]):
                    if self.bp[neighbor[0]][neighbor[1]][neighbor[2]] == 0:
                        continue
                else:
                    continue

                # for each face on neighbor
                for x, y, z in neighborFaces:
                    # if there is not a block on a face
                    nextNeighbor = neighbor[0] + x, neighbor[1] + y, neighbor[2] + z
                    if (self.within_range_huh(nextNeighbor[0], nextNeighbor[1], nextNeighbor[2]) and
                        self.bp[nextNeighbor[0]][nextNeighbor[1]][nextNeighbor[2]] == 0) or (
                    not self.within_range_huh(nextNeighbor[0], nextNeighbor[1], nextNeighbor[2])):
                        neighborFace = neighbor[0] + x * blockWidth, neighbor[1] + y * blockWidth, neighbor[
                            2] + z * blockWidth
                        if self.face_reachable_huh(currentFace, neighborFace):

                            tentative_g_score = gscore[currentFace] + self.heuristic(currentFace, neighborFace)

                            if neighborFace in close_set and tentative_g_score >= gscore.get(neighborFace, 0):
                                continue

                            if tentative_g_score < gscore.get(neighborFace, 0) or neighborFace not in [i[1] for i in
                                                                                                       oheap]:
                                came_from[neighborFace] = currentFace
                                gscore[neighborFace] = tentative_g_score
                                fscore[neighborFace] = tentative_g_score + self.heuristic(neighborFace, goalFace)
                                heapq.heappush(oheap, (fscore[neighborFace], neighborFace))

    def face_reachable_huh(self, lastFace, targetFace):
        lastFaceIdx = self.get_face_index(lastFace)
        targetFaceIdx = self.get_face_index(targetFace)
        if lastFaceIdx == 'front':
            if targetFaceIdx == 'back' and targetFace[1] > lastFace[1]:
                return False
            elif targetFaceIdx == 'front' and lastFace[0] == targetFace[0] and lastFace[2] == targetFace[2]:
                return False
            elif targetFaceIdx != 'front' and lastFace[2] != targetFace[2] and lastFace[0] != targetFace[
                0]:  # face not same xy plane or yz plane
                return False
        elif lastFaceIdx == 'back':
            if targetFaceIdx == 'front' and targetFace[1] < lastFace[1]:
                return False
            elif targetFaceIdx == 'back' and lastFace[0] == targetFace[0] and lastFace[2] == targetFace[2]:
                return False
            elif targetFaceIdx != 'back' and lastFace[2] != targetFace[2] and lastFace[0] != targetFace[
                0]:  # face not same xy plane or yz plane
                return False
        elif lastFaceIdx == 'left':
            if targetFaceIdx == 'right' and targetFace[0] > lastFace[0]:
                return False
            elif targetFaceIdx == 'left' and lastFace[1] == targetFace[1] and lastFace[2] == targetFace[2]:
                return False
            elif targetFaceIdx != 'left' and lastFace[2] != targetFace[2] and lastFace[1] != targetFace[
                1]:  # face not same xy plane or xz plane
                return False
        elif lastFaceIdx == 'right':
            if targetFaceIdx == 'left' and targetFace[0] < lastFace[0]:
                return False
            elif targetFaceIdx == 'right' and lastFace[1] == targetFace[1] and lastFace[2] == targetFace[2]:
                return False
            elif targetFaceIdx != 'right' and lastFace[2] != targetFace[2] and lastFace[1] != targetFace[
                1]:  # face not same xy plane or xz plane
                return False
        elif lastFaceIdx == 'top':
            if targetFaceIdx == 'bottom' and targetFace[2] < lastFace[2]:
                return False
            elif targetFaceIdx == 'top' and lastFace[0] == targetFace[0] and lastFace[1] == targetFace[1]:
                return False
            elif targetFaceIdx != 'top' and lastFace[1] != targetFace[1] and lastFace[0] != targetFace[
                0]:  # face not same xz plane or yz plane
                return False
        elif lastFaceIdx == 'bottom':
            if targetFaceIdx == 'top' and targetFace[2] > lastFace[2]:
                return False
            elif targetFaceIdx == 'bottom' and lastFace[0] == targetFace[0] and lastFace[1] == targetFace[1]:
                return False
            elif targetFaceIdx != 'bottom' and lastFace[1] != targetFace[1] and lastFace[0] != targetFace[
                0]:  # face not same xz plane or yz plane
                return False

        if lastFaceIdx == targetFaceIdx and self.heuristic(lastFace, targetFace) < self.armReach[0]:
            return True
        elif lastFaceIdx != targetFaceIdx and self.armReach[1] / 2 < self.heuristic(lastFace, targetFace) < \
                self.armReach[1]:
            return True
        else:
            return False

    def parse_output(self, face):
        idx_x, idx_y, idx_z = self.get_block_idx(face)
        if self.within_range_huh(idx_x, idx_y, idx_z):
            faceLabel = ''
            if self.bp[idx_x][idx_y][idx_z] == 0:
                pass
            else:
                if idx_x < face[0]:
                    faceLabel = 'right'
                elif idx_x > face[0]:
                    faceLabel = 'left'
                elif idx_y < face[1]:
                    faceLabel = 'back'
                elif idx_y > face[1]:
                    faceLabel = 'front'
                elif idx_z < face[2]:
                    faceLabel = 'top'
                elif idx_z > face[2]:
                    faceLabel = 'bottom'
            return idx_x, idx_y, idx_z, faceLabel
        else:
            return None

    def get_face_index(self, face):
        idx_x, idx_y, idx_z = self.get_block_idx(face)

        if self.within_range_huh(idx_x, idx_y, idx_z):
            if self.bp[idx_x][idx_y][idx_z] == 0:
                return None
            else:
                if idx_x < face[0]:
                    return 'right'
                elif idx_x > face[0]:
                    return 'left'
                elif idx_y < face[1]:
                    return 'back'
                elif idx_y > face[1]:
                    return 'front'
                elif idx_z < face[2]:
                    return 'top'
                elif idx_z > face[2]:
                    return 'bottom'

    def get_block_idx(self, face):
        idx_x = int(round(face[0]))
        idx_y = int(round(face[1]))
        idx_z = int(round(face[2]))
        return idx_x, idx_y, idx_z

    def within_range_huh(self, x, y, z):
        if 0 <= x < self.bp.shape[0] and 0 <= y < self.bp.shape[1] and 0 <= z < self.bp.shape[2]:
            return True
        else:
            return False

    def get_path(self):
        route = self.faceStar(self.startFace, self.goalFace)
        if route is None:
            # self.logger.error("Unable to find route between points {} and {}".format(self.startFace, self.goalFace))
            raise Exception("Path planning unable to find route")
        route = route[::-1]
        print("Path to Traverse: {}\n".format(route))
        self.route = route
        return route

    # def display_path(self):
    #     for i in self.route:
    #         self.colors[i] = '#ff0000ff'

    #     self.colors[self.route[-1]] = '#03fc62'
    #     return self.colors

    def display_path(self, path=None):
        if not path:
            path = self.path
        npPath = np.array(path)
        npPath = np.add(npPath, 0.5)

        fig = plt.figure(figsize=(12, 12))
        ax = Axes3D(fig)
        ax.voxels(self.bp, facecolors=self.colors, edgecolors='gray', zorder=0)
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.set_zlim(0, 10)

        ax.plot(npPath[:, 0], npPath[:, 1], npPath[:, 2], c='r', marker='o', markersize=25)

        plt.show()

    def display_blueprint(self):
        fig = plt.figure(figsize=(12, 12))
        ax = Axes3D(fig)
        ax.voxels(self.bp, facecolors=self.colors, edgecolors='gray', zorder=0)
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.set_zlim(0, 10)
        plt.show()

    def display_start_end(self, path=None):
        if not path:
            path = self.path
        npPath = np.array(path)
        npPath = np.add(npPath, 0.5)

        fig = plt.figure(figsize=(12, 12))
        ax = Axes3D(fig)
        ax.voxels(self.bp, facecolors=self.colors, edgecolors='gray', zorder=0)
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.set_zlim(0, 10)

        ax.plot(npPath[:, 0], npPath[:, 1], npPath[:, 2], c='r', marker='o', markersize=25)
        plt.show()


if __name__ == '__main__':
    armReach = [2.38, 1.58]
    # armReach = [3, 3]

    startFace = BlockFace(0, 0, 0, 'top')
    endFace = BlockFace(7, 2, 2, 'right')
    bp1 = np.array([
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 1, 0], [1, 1, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 1, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [1, 0, 0], [1, 1, 1], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
    ])

    faceStarPlanner = FaceStar(startFace, endFace, bp1, armReach)
    path = faceStarPlanner.get_path()
    faceStarPlanner.display_path()

    bp2 = np.array([
        [[1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0]],
    ])

    bp3 = np.array([
        [[1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [1, 0, 0], [1, 1, 1]],
    ])

    startFaceDebug = BlockFace(0, 0, 0, 'top')
    endFaceDebug = BlockFace(2, 0, 0, 'right')

    # faceStarDebug = FaceStar(startFaceDebug, endFaceDebug, bp3, armReach)
    # faceStarDebug.display_blueprint()
    # faceStarDebug.display_start_end([startFaceDebug.get_face_coordinate(),endFaceDebug.get_face_coordinate()])
    # path = faceStarDebug.get_path()
    # faceStarDebug.display_path()






