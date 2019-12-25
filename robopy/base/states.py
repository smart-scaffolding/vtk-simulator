from enum import Enum

class GripperSelect(Enum):
    yellow = 0
    red = 1
    both = 2

class GripperActions(Enum):
    engage = 0
    disengage = 1
    idle = 2

class RobotActions(Enum):
    findPath = 0
    placeBlock = 1
    inch = 2
    removeBlock = 3

class RobotBase(Enum):
    keepOrientation = 0
    flipOrientation = 1
