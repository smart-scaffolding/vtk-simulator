'''
	1. Divide structure into smallest divisions allowed (determined by area) [x]
	2. Create KD tree for center of each divisions
	3. Store in data structure all divisions, sorted by (x, y, z)
	4. For each level in building:
		a. While not all divisions marked "FERRY" or "UNCLAIMED":
			i. Pop first section from divisions, change status to "BUILDING"
			ii. Search for shortest path between all ferry regions and new section to build
			iii. If more points in path than robots, add sections together
			iv. For number of ferry divisions in path + 1:
				1) Assign closest robot to ferry section, robot ferries blocks to next division
				2) Assign closest robot to build section, robot builds section
When finished building, change status of section to "FERRY", and append to list
'''

from robopy.base.building_planner import *
import numpy as np
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class BuildingController():
    def __init__(self, blueprint):
        self.blueprint = blueprint
        self.divided_structure = None
        self.kd_tree = None

    def build_structure(self):
        self.building_planner = BuildingPlanner(self.blueprint)
        # building_planner.structure = intialize_structure(self.blueprint) #TODO: Put this inside class and remove here
        # self.divided_structure = building_planner.divide_structure()
        self.divided_structure = self.building_planner.create_divisions()
        data = []
        self.mappings = {}
        for divisions in self.divided_structure:
            data.append(divisions.centroid)
            self.mappings[divisions.centroid] = divisions

        self.kd_tree = cKDTree(data)

if __name__ == '__main__':
    blueprint = np.array([
                             [[1] * 9] * 9,
                         ] * 9)
    bc = BuildingController(blueprint)
    bc.build_structure()
    print(len(bc.divided_structure))
    dd, ii = bc.kd_tree.query([0, 0, 0], k=9)

    # print(bc.kd_tree.data)
    # for val in ii:
    #     centroid = bc.kd_tree.data[val]
        # print(centroid)
        # print(bc.mappings[tuple(centroid)])

    # division_map = [[[1]]]
    z_height = 4
    print(bc.mappings.keys())
    plot, ax = bc.building_planner.display(1, return_plot=True)
    keys= np.array(list(bc.mappings.keys()))
    keys = list(filter(lambda val: val[2] < z_height, keys))
    keys = np.array(keys)
    print(keys)
    ax.scatter(keys[:, 0], keys[:, 1], keys[:, 2])
    plt.show()




