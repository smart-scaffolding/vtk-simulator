from enum import Enum, IntEnum

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

class DivisionStates(Enum):
    BUILDING = 0
    FERRY = 1
    UNCLAIMED = 2

class BuildingStates(IntEnum):
    WAITING_FOR_FERRYING = 0
    WAITING_FOR_FILLING = 1
    DONE_ORIGIN = 2
    DONE = 3
    EMPTY = 4
