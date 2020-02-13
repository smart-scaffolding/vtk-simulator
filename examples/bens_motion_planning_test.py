import zlib
import pickle
from random import choice, randint
import numpy as np
import zmq
import time

context = zmq.Context()
socket = context.socket(zmq.PUB)
# socket.bind(f"tcp://*:{port}")
# socket.bind("tcp://127.0.0.1:5559")
socket.connect("tcp://130.215.217.37:5559")

class Message:
    def __init__(self, message_id):
        self.message_id = message_id

    def __str__(self):
        return str(self.__dict__)


class AnimationUpdateMessage(Message):
    def __init__(self, robot_base, trajectory, direction=None, path=None, placedObstacle=False,
                 obstacle=None):
        super().__init__(message_id="SIMULATION")
        self.robot_base = robot_base
        self.direction = direction
        self.trajectory = trajectory
        self.path = path
        self.placedObstacle = placedObstacle
        self.obstacle = obstacle


class MessageWrapper:
    def __init__(self, topic, message):
        self.topic = topic
        self.message = message


while True:
    topic = b"ROBOT_1"

    base = np.matrix([[1, 0, 0, 0.5],
                     [0, 1, 0, 0.5],
                     [0, 0, 1, 1.],
                     [0, 0, 0, 1]])
    base1 = np.matrix([[1, 0, 0, 1.5],
                      [0, 1, 0, 0.5],
                      [0, 0, 1, 1.],
                      [0, 0, 0, 1]])
    base2 = np.matrix([[1, 0, 0, 1.5],
                      [0, 1, 0, 1.5],
                      [0, 0, 1, 1.],
                      [0, 0, 0, 1]])
    base3 = np.matrix([[1, 0, 0, 4.5],
                      [0, 1, 0, 0.5],
                      [0, 0, 1, 1.],
                      [0, 0, 0, 1]])
    base4 = np.matrix([[1, 0, 0, 4.5],
                      [0, 1, 0, 1.5],
                      [0, 0, 1, 3.],
                      [0, 0, 0, 1]])
    base5 = np.matrix([[1, 0, 0, 5.5],
                      [0, 1, 0, 0.5],
                      [0, 0, 1, 1.],
                      [0, 0, 0, 1]])

    base = choice([base, base1, base2, base3, base4, base5])

    trajectory1 = np.array([[0, 0, 0, 0]])
    trajectory2 = np.array([[0, 0, np.pi/2, 0]])
    trajectory3 = np.array([[0, 0, 0, np.pi/2]])
    trajectory4 = np.array([[0, 0, np.pi/2, np.pi/2]])
    trajectory5 = np.array([[0, np.pi/2, 0, 0]])


    trajectory_choice = choice([trajectory1, trajectory2, trajectory3, trajectory4, trajectory5])
    messagedata = AnimationUpdateMessage(robot_base=base, trajectory=trajectory_choice)
    message_obj = MessageWrapper(topic=topic, message=messagedata)
    p = pickle.dumps(message_obj, protocol=-1)
    z = zlib.compress(p)
    print(f"{topic} {z}")
    socket.send_multipart([topic, z])
    time.sleep(3)






