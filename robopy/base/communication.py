import zmq
import zlib
import pickle

from robopy.base.messages import *

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.connect("tcp://127.0.0.1:5559")

def send_to_simulator(base, trajectory, topic=b"ROBOT_1"):
    messagedata = AnimationUpdateMessage(robot_base=base, trajectory=trajectory)
    message_obj = MessageWrapper(topic=topic, message=messagedata)
    p = pickle.dumps(message_obj, protocol=-1)
    z = zlib.compress(p)
    print(f"{topic} {z}")
    socket.send_multipart([topic, z])
