import time
import sys
from copy import copy

def newSearch(mapOfWorld, goal, start):
    heap = []
    newheap = []
    x, y = goal
    lastwave = 3
    # Start out by marking nodes around G with a 3
    moves = [(x + 1, y), (x - 1, y), (x, y - 1), (x, y + 1)]

    for move in moves:
        if (mapOfWorld.positions[move] == 1):
            mapOfWorld.positions[move] = 3
            heap.append(move)

    reachedStart = False
    goal_mapOfWorld = None
    goal_lastwave = None
    for currentwave in range(4, 1000):
        lastwave = lastwave + 1
        while (heap != []):
            position = heap.pop()
            (x, y) = position
            moves = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
            # x, y = position
            for move in moves:
                if (mapOfWorld.positions[move] != 0):
                    if (mapOfWorld.positions[move] == 1 and mapOfWorld.positions[position] == currentwave - 1):
                        mapOfWorld.positions[move] = currentwave
                        newheap.append(move)
                    if (move == start):
                        reachedStart = True
                        goal_mapOfWorld = mapOfWorld
                        goal_lastwave = lastwave
                        # return mapOfWorld, lastwave

        # time.sleep(0.25)
        # mapOfWorld.display()
        # print heap

        heap = newheap
        newheap = []
    if reachedStart:
        return goal_mapOfWorld, goal_lastwave
    else:
        print("Goal is unreachable")
        return 1

# def printf(format, *args):
#     sys.stdout.write(format % args)


class Map(object):

    def __init__(self, xdim, ydim, positions):
        self.xdim = xdim
        self.ydim = ydim
        self.positions = positions

    def columnize(self, word, width, align='Left'):

        nSpaces = width - len(word)
        if len(word) == 1:
            word = " "+word
        if nSpaces < 0:
            nSpaces = 0
        if align == 'Left':
            return word + (" " * nSpaces)
        if align == 'Right':
            return (" " * nSpaces) + word
        return (" " * int(nSpaces / 2)) + word + (" " * int(nSpaces - nSpaces / 2))

    def print_grid(self, grid=[[1, 1, 0], [1, 0, 1], [0, 1, 1]], column = 10):
        print()
        print('%s%s' % (self.columnize('Table |', column * 2, 'Right'), \
                        '|'.join([self.columnize('Col %d' % i, column, 'Center') \
                                  for i in range(1, len(grid[0]))])))
        spaces = sum([column + 1 for i in range(len(grid[0]))]) + column * 2
        print('=' * spaces)
        for i, item in enumerate(grid):
            print('%s%s' % (self.columnize('Row %d |' % (i+1), column * 2, 'Right'), \
                            '|'.join([self.columnize(str(num), column, 'Center') \
                                      for num in item])))

    def nav(self, start, current, goal):
        self.pos = start
        finished = False
        self.positions[goal] = 'G'

        while (finished == False):  # Run this code until we're at the goal
            x, y = self.pos
            self.positions[self.pos] = 'R'  # Set the start on the map (this USUALLY keeps start the same)
            #         SOUTH        NORTH         WEST      EAST
            #           v           v             v          v
            moves = [(x + 1, y), (x - 1, y), (x, y - 1), (x, y + 1)]  # Establish our directions
            moveDirections = ["Backwards", "Forwards", "Left",
                              "Right"]  # Create a corresponding list of the cardinal directions
            """ We don't want least to be 0, because then nothing would be less than it.
                However, in order to make our code more robust, we set it to one of the values,
                so that we're comparing least to an actual value instead of an arbitrary number (like 10).
            """
            # Do the actual comparing, and give us the least index so we know which move was the least
            leastIndex = 1
            for w in range(len(moves)):
                move = moves[w]

                # If the position has the current wave - 1 in it, move there.
                # try:
                if (self.positions[move] == current + 1):
                    self.least = self.positions[move]
                    leastIndex = w
                # Or, if the position is the goal, stop the loop
                elif (self.positions[move] == 'G'):
                    finished = True
                    leastIndex = w
                # except:
                #     pass
            # Decrement the current number so we can look for the next number
            current = current + 1
            # self.positions[self.pos] = 'X'
            print(f"Moved {moveDirections[leastIndex]} {moves[leastIndex]}")
            self.pos = moves[leastIndex]  # This will be converted to "move robot in x direction"

            # time.sleep(0.25)
            # self.display()
            # self.print_grid(grid=convertMapToGrid(mapOfWorld, mapOfLand, start, end))

        # Change the goal position (or wherever we stop) to an "!" to show that we've arrived.
        # self.positions[self.pos] = 'X'
        # print(f"Moved {moveDirections[leastIndex]} {moves[leastIndex]}")
        self.print_grid(grid=convertMapToGrid(mapOfWorld, mapOfLand, start, end))

        # self.display()



def convertMap(mapOfWorld):
    positions = {}
    xdim = len(mapOfWorld)
    ydim = len(mapOfWorld[1])
    for y in range(ydim):
        for x in range(xdim):
            positions[(x, y)] = mapOfWorld[x][y]

    return Map(xdim, ydim, positions)

def convertMapToGrid(mapOfWorld, newMap, start, goal):
    for key, value in newMap.positions.items():
        x, y = key

        # if value == "G":
        #     value = -1
        # if value == "R":
        #     value = -1
        if (x, y) == start:
            value = "1"

        if (x, y) == goal:
            value = "30"

        try:
            mapOfWorld[x-1][y-1] = value
        except:
            pass
    return mapOfWorld

# mapOfWorld = [['W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W'],
#               ['W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'G', 'W'],
#               ['W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W'],
#               ['W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W'],
#               ['W', 'W', 'W', 'W', 'W', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W'],
#               ['W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', 'W', 'W', 'W', ' ', 'W'],
#               ['W', ' ', 'W', 'W', ' ', 'W', ' ', 'W', ' ', ' ', ' ', ' ', 'W'],
#               ['W', ' ', 'W', 'W', 'W', 'W', ' ', 'W', ' ', ' ', ' ', ' ', 'W'],
#               ['W', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', 'W', ' ', ' ', 'W'],
#               ['W', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', 'W', 'W', 'W', 'W'],
#               ['W', ' ', 'W', ' ', ' ', ' ', ' ', 'W', ' ', ' ', ' ', ' ', 'W'],
#               ['W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', 'W', 'W', 'W', 'W', 'W'],
#               ['W', ' ', 'W', 'W', ' ', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W'],
#               ['W', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W'],
#               ['W', ' ', 'W', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W'],
#               ['W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W'],
#               ['W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W'],
#               ['W', ' ', ' ', 'W', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W'],
#               ['W', ' ', 'W', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W'],
#               ['W', ' ', 'W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W'],
#               ['W', ' ', 'W', 'W', ' ', 'W', 'W', 'W', 'W', 'W', 'W', ' ', 'W'],
#               ['W', ' ', ' ', ' ', ' ', 'W', 'W', ' ', ' ', ' ', 'W', ' ', 'W'],
#               ['W', ' ', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W'],
#               ['W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W', ' ', 'W'],
#               ['W', ' ', 'W', ' ', 'W', 'W', 'W', 'W', ' ', ' ', 'W', ' ', 'W'],
#               ['W', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W'],
#               ['W', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W'],
#               ['W', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W'],
#               ['W', ' ', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W', ' ', 'W'],
#               ['W', 'R', 'W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W'],
#               ['W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W'], ]

# mapOfWorld = [['W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W'],
#               ['0', 'R', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'G', 'W'],
#               ['W', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'W'],
#               ]

mapOfWorld = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 30, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 0, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 2, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
              ]

mapOfWorld = [[0, 0, 0, 0, 0],
              [0, 1, 1, 1, 0],
              [0, 1, 1, 1, 0],
              [0, 1, 1, 1, 0],
              [0, 0, 0, 0, 0],
              ]

newWorld = [
            [1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1],
              ]

newWorld = [[1, 1, 1],
            [1, 1, 1],
            [1, 1, 1]
            ]
mapOfLand = convertMap(mapOfWorld)
# mapOfLand.print_grid(grid=mapOfWorld)
start = (1, 1)
end = (3, 3)
mapOfLand, lastwave = newSearch(mapOfLand, start, end)
newMap = copy(mapOfLand)


newMap.positions = {k:v for k,v in newMap.positions.items() if str(v) != '0'}

newMap.print_grid(grid=convertMapToGrid(newWorld, newMap, start, end))
mapOfLand.nav(start, 2, (1, 3))

newMap.positions = dict(sorted(newMap.positions.items(), key=lambda x: int(newMap.positions[x[0]])))
print(newMap.positions)
print(newMap.positions.values())

# mapOfLand.nav(start, 2, (8, 7))
# print(mapOfLand.positions[(8, 1)])

# currentSearch(mapOfWorld, findGoal(mapOfWorld), findStart(mapOfWorld))
