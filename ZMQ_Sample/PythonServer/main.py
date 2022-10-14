from json import tool
import zmq
import time
import cv2
import numpy as np
import struct
from depth_utils import *
import argparse
import os
from IRTrack.IRToolTrack import IRToolTrack

timestamp = 0
flag = True

def receive_depth(ip_address):
    topic = "depth_frame"
    context = zmq.Context()

    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.RCVTIMEO, 5000)
    socket.subscribe("")

    port = "12387"
    socket.connect("tcp://%s:%s" % (ip_address,port))
    print("Depth subscriber connected  with  %s:%s" % (ip_address,port) )
    #socket.subscribe(topic)

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    tool_tracker = IRToolTrack(sphere_radius=6.3)
    tool_tracker.track_tool()

    while True:
        try:
            socks = dict(poller.poll(5000))

            if socket in socks and socks[socket] == zmq.POLLIN:
                msg = socket.recv_multipart()
                message_type = msg[0].decode('utf-8')
                if message_type != topic:
                    print("Received string: %s" % message_type)
                    continue

                timestamp = struct.unpack(">q", msg[1])[0]  # long long, i.e. 64-bit int

                cam_pose_array = np.frombuffer(msg[2], np.dtype(np.float32))
                pose =np.array(np.reshape(cam_pose_array,(4,3)).T)
                pose[:3,3] = pose[:3,3] - pose[:3,1] * 0.0015 # add a magic offset 

                centers = np.frombuffer(msg[3], np.dtype(np.float32)).reshape((-1, 3))
                tool_tracker.add_frame(centers, pose, timestamp)
        except KeyboardInterrupt:
            print("\n[IR] Exiting...")
            break
        except zmq.ZMQError as e:
            print("\n[IR] Exception:", e)
            continue
        except Exception as e:
            print("\n[IR] Depth Exception: ", e)
            break
    
    if tool_tracker is not None:
        tool_tracker.tracking_ir_thread.stop()
    socket.close()
    context.term()
    exit(0)


def main():
    parser = argparse.ArgumentParser(description='Save Dir.')
    
    parser.add_argument('--ip', default='192.168.50.213',
                        help='IP address of the HoloLens')

    args = parser.parse_args()

    receive_depth(args.ip)
    

if __name__ == "__main__":
    main()