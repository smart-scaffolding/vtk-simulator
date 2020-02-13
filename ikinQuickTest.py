from math import cos, sin, acos, asin, atan2, pi, sqrt
import numpy as np
from serial import Serial, PARITY_NONE, STOPBITS_ONE, EIGHTBITS
import time

DEBUG = False

# Robot state variables
AEEPOS = None
AEEORI = None
DEEPOS = None
DEEORI = None

APOSE = np.array([0,0,0,-pi/2,0,0])
DPOSE = np.array([2,0,0,-pi/2,0,0])

# Robot control variables
GRIPPER_A_ENGAGED = True
STEP_SIZE = 20

# Serial variables
USE_SERIAL = False
SERIAL = None
PORT='/dev/cu.usbmodem14201'
BAUD = 9600
TIMEOUT = 3.0
JOINT_ANGLE_PKT_SIZE = 8


# Units: inches, radians
# inputs should be in the global reference frame
def ikin(goalPos,gamma,phi,baseID,elbow_up=1):
    global AEEPOS, AEEORI, DEEPOS, DEEORI
    # Robot Parameters
    L1 = 4.125 # L1 in inches
    L2 = 6.43 # L2 in inches
    blockWidth = 3
    
    relativePos, localGamma = handlePlaneChanges(goalPos=goalPos,gamma=gamma,baseID=baseID)
    x,y,z = relativePos * blockWidth

    if DEBUG: 
        print(f'x y z is {x} {y} {z}')
        print(f'gamma is: {gamma}')
        print(f'localGamma is : {localGamma}')

    q1 = atan2(y,x) # joint1 angle
    # print(f'q1 is {q1*180/pi}')

    new_z = z - L1 # take away the height of the first link (vertical) 
    new_x = x/cos(q1)
    # print(f'new_x is {new_x}')

    x3 = new_x - L1 * cos(localGamma)
    z3 = new_z - L1 * sin(localGamma) #reduce to the 2dof planar robot

    beta = acos((L2**2 + (x3**2 + z3**2) - L2**2)/(2*L2*sqrt(x3**2 + z3**2)))

    if elbow_up == 1:
        q3 = 2 * beta
        q2 = pi/2 - (atan2(z3,x3) + beta)

    elif elbow_up == 0:
        q3 = -2 * beta
        q2 = pi/2 - (atan2(z3,x3) - beta)

    q4 = (localGamma - pi/2 + q2 + q3)*-1
    q5 = phi - q1
    q = []

    # check which ee is requested and flip angles accordingly
    if baseID == 'A':
        q = np.array([q1,q2,q3,q4,q5])
    elif baseID == 'D':
        q = np.array([q5,q4,q3,q2,q1])
    return q

def handlePlaneChanges(goalPos,gamma,baseID):
    global AEEPOS, AEEORI, DEEPOS, DEEORI
    relativePos = np.zeros(3)
    baseOri = np.zeros(3)
    basePos = np.zeros(3)
    goalOri = np.zeros(3)

    if DEBUG: 
        print(f'baseID: {baseID}')

    # gets the relativePos, basePos, and baseOri in the global reference frame
    if baseID == 'A': # requested ee is A
        relativePos = goalPos - AEEPOS
        baseOri = AEEORI
        basePos = AEEPOS
    elif baseID == 'D': # requested ee is D
        relativePos = goalPos - DEEPOS
        baseOri = DEEORI
        basePos = DEEPOS

    # the angle which the arm is extending towards from base ee
    # in global reference frame
    # !! This needs to be done before relativePos switched into local frame
    armFacing = atan2(relativePos[1],relativePos[0])
    if DEBUG: 
        print(f'relativePos: {relativePos}')
        print(f'armFacing: {armFacing}')

    # rotates relativePos from global reference frame to local reference frame
    if baseOri[0] == 1: # local +z facing global +x, rotate -90 around y
        relativePos = np.dot(getRy(-pi/2),relativePos)
    elif baseOri[0] == -1: # local +z facing global -x, rotate 90 around y
        relativePos = np.dot(getRy(pi/2),relativePos)
    elif baseOri[1] == 1: # local +z facing global +y, rotate 90 around x
        relativePos = np.dot(getRx(pi/2),relativePos)
    elif baseOri[1] == -1: # local +z facing global -y, rotate -90 around x
        relativePos = np.dot(getRx(-pi/2),relativePos)
    elif baseOri[2] == 1: # local +z facing global +z, do nothing
        pass
    elif baseOri[2] == -1: # local +z facing global -z, rotate 180 around y
        relativePos = np.dot(getRy(pi),relativePos)

    if DEBUG: 
        print(f'relativePos in local frame: {relativePos}')
    localGamma = gamma

    # updates goal orientation and 
    # switches gamma value from global reference frame to local reference frame
    if gamma == -pi/2: # goal ee will be facing down
        goalOri = np.array([0,0,1]).T
        if baseOri[2] == 1: # base ee down
            pass
        elif baseOri[2] == -1: # base ee up
            localGamma = -gamma
        else: # base ee horizontal
            localGamma = 0
    elif gamma == pi/2: # goal ee will be facing up
        goalOri = np.array([0,0,-1]).T
        if baseOri[2] == 1: # base ee down
            pass
        elif baseOri[2] == -1: # base ee up
            localGamma = -gamma
        else: # base ee horizontal
            localGamma = 0
    elif gamma == 0 or gamma == -pi: # goal ee horizontal
        if armFacing == 0 or armFacing == pi: # goal ee facing right (positive X)
            if gamma == 0:
                goalOri = np.array([-1,0,0]).T
            else:
                goalOri = np.array([1,0,0]).T
        elif armFacing == -pi: # goal ee facing left (negative X)
            if gamma == 0:
                goalOri = np.array([1,0,0]).T
            else:
                goalOri = np.array([-1,0,0]).T
        elif armFacing == pi/2: # goal ee facing back (positive Y)
            if gamma == 0:
                goalOri = np.array([0,-1,0]).T
            else:
                goalOri = np.array([0,1,0]).T
        elif armFacing == -pi/2: # goal ee facing front (negative Y)
            if gamma == 0:
                goalOri = np.array([0,1,0]).T
            else:
                goalOri = np.array([0,-1,0]).T
        
        if baseOri[2] != 0: # base ee down or up
            pass
        else: # base ee horizontal
            if baseOri.all() != goalOri.all():
                localGamma = pi/2
            else:
                localGamma = -pi/2

    # sets the goalPos and goalOri to the moving ee
    if baseID == 'A': # requested ee is A, update D to match goal
        DEEPOS = goalPos
        DEEORI = goalOri
    elif baseID == 'D': # requested ee is D, update A to match goal
        AEEPOS = goalPos
        AEEORI = goalOri

    if DEBUG: 
        print(f'AEEPOS: {AEEPOS}')
        print(f'AEEORI: {AEEORI}')
        print(f'DEEPOS: {DEEPOS}')
        print(f'DEEORI: {DEEORI}\n')
    return relativePos.T, localGamma

def setEEStartingPoses(aEEPos,aEEOri,dEEPos,dEEOri):
    global AEEPOS, AEEORI, DEEPOS, DEEORI
    AEEPOS = aEEPos
    AEEORI = aEEOri
    DEEPOS = dEEPos
    DEEORI = dEEOri

def resetEEStartingPoses():
    global AEEPOS, AEEORI, DEEPOS, DEEORI
    AEEPOS = np.array([0,0,0]).T
    AEEORI = np.array([0,0,1]).T
    DEEPOS = np.array([0,0,0]).T
    DEEORI = np.array([0,0,1]).T

# def getT(theta,alpha):
#     T = np.eye([4,4])
#     T[0,:] = [cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta)]
#     T[1,:] = [sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta)]
#     T[2,:] = [0,sin(alpha),cos(alpha),d]
#     return T

def getRx(theta):
    T = np.eye(3)
    T[1,:] = [0,cos(theta),-sin(theta)]
    T[2,:] = [0,sin(theta),cos(theta)]
    return T

def getRy(theta):
    T = np.eye(3)
    T[0,:] = [cos(theta),0,sin(theta)]
    T[2,:] = [-sin(theta),0,cos(theta)]
    return T

def getRz(theta):
    T = np.eye(3)
    T[0,:] = [cos(theta),-sin(theta),0]
    T[1,:] = [sin(theta),cos(theta),0]
    return T

def baseCheck():
    print('Base Check')
    resetEEStartingPoses()

    pos0 = np.array([0,0,0])
    pos1 = np.array([2,0,0,-pi/2,0,0]) # x, y, z, gamma, phi, requestedBase(0 for 'A'; 1 for 'D')

    qA = ikin(goalPos=pos1[:3],gamma=pos1[3],phi=pos1[4],elbow_up=1,baseID='A')
    print(f'Step one \n({pos0}) \nto \n({pos1}): \n\n{qA*180/pi}\n\n')
    
def samePlane():
    resetEEStartingPoses()
    print('Same Plane')

    pos0 = np.array([0,0,0])
    pos1 = np.array([2,2,0,-pi/2,0]) # x, y, z, gamma, phi
    pos2 = np.array([1,4,0,-pi/2,0]) # x, y, z, gamma, phi

    qA = ikin(goalPos=pos1[:3],gamma=pos1[3],phi=pos1[4],elbow_up=1,baseID='A')
    print(f'Step one \n({pos0}) \nto \n({pos1}): \n\n{qA*180/pi}\n\n')

    qD = ikin(goalPos=pos2[:3],gamma=pos2[3],phi=pos2[4],elbow_up=1,baseID='D')
    print(f'Step two \n({pos1}) \nto \n({pos2}): \n\n{qD*180/pi}\n\n')

def changePlaneNInch():
    # 1. Save the current orientation and position of the both ee.
    # Transformation matrices? Otherwise it would be a mess to keep 
    # track of the current pose and update accordingly

    # 2. Depending on which ee is closer to the next position or which
    # ee is requested, move the corresponding ee 

    # 3. Additionally, add feasibility check for each move, aka, 
    # diagnoal moves are only feasible when on the same plane and 
    # plane change is only feasible when perpendicular lines

    print('Different Plane')
    resetEEStartingPoses()
    
    pos0 = np.array([0,0,0])
    pos1 = np.array([3,0,3,0,0]) # x, y, z, gamma, phi
    pos2 = np.array([1,0,0,-pi/2,0])

    qA = ikin(goalPos=pos1[:3],gamma=pos1[3],phi=pos1[4],elbow_up=1,baseID='A')
    print(f'Step one \n({pos0}) \nto \n({pos1}): \n\n{qA*180/pi}\n\n')

    qD = ikin(goalPos=pos2[:3],gamma=pos2[3],phi=pos2[4],elbow_up=1,baseID='D')
    print(f'Step two \n({pos1}) \nto \n({pos2}): \n\n{qD*180/pi}\n\n')

def changePlaneNClimbXZ():
    resetEEStartingPoses()
    steps = []
    steps.append(np.array([2,0,3,0,0,1,0])) # x, y, z, gamma, phi, elbow(1 for up 0 for down), requestedBase(0 for 'A'; 1 for 'D')
    steps.append(np.array([2,0,1,0,0,1,1]))
    steps.append(np.array([2,0,4,0,0,1,0]))
    steps.append(np.array([2,0,2,0,0,1,1]))
    performSteps(steps)

def changePlaneNClimbYZ():
    resetEEStartingPoses()
    steps = []
    steps.append(np.array([0,2,3,0,0,1,0])) # x, y, z, gamma, phi, elbow(1 for up 0 for down), requestedBase(0 for 'A'; 1 for 'D')
    steps.append(np.array([0,2,1,0,0,1,1]))
    steps.append(np.array([0,2,4,0,0,1,0]))
    steps.append(np.array([0,2,2,0,0,1,1]))
    performSteps(steps)

def convexCorner(): # 
    print('Convex corner +x to -z')
    resetEEStartingPoses()
    steps = []
    steps.append(np.array([2,0,-1,-pi,0,1,0])) # x, y, z, gamma, phi, elbow(1 for up 0 for down), requestedBase(0 for 'A'; 1 for 'D')
    steps.append(np.array([1,0,0,-pi/2,0,1,1]))
    steps.append(np.array([2,0,-2,-pi,0,1,0]))
    steps.append(np.array([2,0,-1,-pi,0,1,1]))
    performSteps(steps)
    print('Convex corner -x to -z')
    resetEEStartingPoses()
    steps = []
    steps.append(np.array([-2,0,-1,0,0,1,0])) # x, y, z, gamma, phi, elbow(1 for up 0 for down), requestedBase(0 for 'A'; 1 for 'D')
    steps.append(np.array([-1,0,0,-pi/2,0,1,1]))
    steps.append(np.array([-2,0,-2,0,0,1,0]))
    steps.append(np.array([-2,0,-1,0,0,1,1]))
    performSteps(steps)

def performSteps(steps):
    for step in steps:
        baseID = 'A'
        start = DPOSE
        if step[6] == 1:
            baseID = 'D'
            start = APOSE
        for waypoint in genWaypoints(start,step):
            print(f'Step to \n({step[:3]}):')
            q = ikin(goalPos=waypoint[:3],gamma=waypoint[3],phi=waypoint[4],elbow_up=step[5],baseID=baseID)
            print(f'joint angles: {q*180/pi}\n\n')
            

def genWaypoints(start, stop, N = STEP_SIZE, endpoint=True):
    return np.linspace(start, stop, N)

def send_to_robot(angle, delay=2.0):
    """
    NOTE: Expects all angles to be in degrees
    Sends a single angle to robot and then delays for a certain amount of time

    :param angle: Expects angles in degrees
    :param delay: delay after sending to robot
    """
    targetAngles = map_angles_to_robot(angle)
    SERIAL.write(targetAngles)
    time.sleep(delay)

def map_angles_to_robot(q):
    """
    Creates a mapping between the angles used by the higher level code and the actual robot angles
    Example robot angle: '-027.25_' (ending in a space)
    :param q: Input angle, expects angles in degrees
    :return:
    """
    q = np.array(q) * 180.0 / pi
    gripperFiller = "0000" # should never change gripper state while moving the robot
    targetAngles = f'{q[1]:4.2f} '.zfill(8) + f'{q[2]:4.2f} '.zfill(8) + f'{q[3]:4.2f} '.zfill(8) + gripperFiller
    return str.encode(targetAngles)

def gripper_control(targetGripper, action):
    if targetGripper == 'A'


# def gripper_control_commands(engage_gripper, disengage_gripper, flip_pid, toggle_gripper):

#     if engage_gripper:
#         gripper_control = "0"
#     elif disengage_gripper:
#         gripper_control = "1"
#     else:
#         gripper_control = "2"  # stop gripper (idle)

#     pid = "1" if flip_pid else "0"
#     select_gripper = "0"
#     if toggle_gripper:
#         if select_gripper == "0":
#             select_gripper = "1"
#         else:
#             select_gripper = "0"

#     return "0" + pid + select_gripper + gripper_control

if __name__ == "__main__":

    if USE_SERIAL:
        SERIAL = Serial(port=PORT, baudrate=BAUD, parity=PARITY_NONE,
                                    stopbits=STOPBITS_ONE, bytesize=EIGHTBITS, timeout=TIMEOUT)

    # baseCheck()

    # samePlane()

    # changePlaneNInch()

    # changePlaneNClimbXZ()

    changePlaneNClimbYZ()

    # convexCorner()

    # steps = []
    # steps.append(np.array([2,0,1,0,0,1,0]))
    # steps.append(np.array([4,0,2,-pi/2,0,0,1]))
    # resetEEStartingPoses()
    # performSteps(steps)
    