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

def receive_depth(ip_address, save_path):
    topic = "depth_frame"
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.RCVTIMEO, 5000)
    socket.subscribe("")

    socket_cmd = context.socket(zmq.REP)
    socket.setsockopt(zmq.RCVTIMEO, 5000)

    port, port_cmd = "12347", "12348"
    socket.connect("tcp://%s:%s" % (ip_address,port))
    print("Depth subscriber connected  with  %s:%s" % (ip_address,port) )
    #socket.subscribe(topic)

    socket_cmd.bind("tcp://*:%s" % port_cmd)
    print("Command publisher binded  with port %s" % (port_cmd) )

    depth_save_path = os.path.join(save_path, "depth")
    if not os.path.exists(depth_save_path):
        os.makedirs(depth_save_path)

    lut_received = False
    lut = None

    # wait for lut
    while not lut_received:
        try:
            msg = socket_cmd.recv_multipart()
            message_type = msg[0].decode('utf-8')
            
            if message_type != "depth_lut":
                print("Received string: %s" % message_type)
                continue
            
            print("LUT received. Length: ", len(msg[1]))
            socket_cmd.send_string('OK')
            
            lut = np.frombuffer(msg[1], np.dtype(np.float32))
            lut = np.reshape(lut, (-1, 3))
            np.save(os.path.join(save_path, "lut"), lut)
            print("LUT saved.")
            lut_received = True
        except KeyboardInterrupt:
            print("\nExiting...")
            exit(1)
        except Exception as e:
            print(e)
            continue

    tool_tracker = IRToolTrack()
    tool_tracker.track_tool(lut)
    

    while flag:
        try:
            msg = socket.recv_multipart()
            message_type = msg[0].decode('utf-8')
            if message_type != topic:
                print("Received string: %s" % message_type)
                continue  
  
            timestamp = struct.unpack("q", msg[1])[0]  # ulong
            # print ("Depth ts: ", timestamp)

            cam_pose_array = np.frombuffer(msg[2], np.dtype(np.float32))
            pose = np.reshape(cam_pose_array,(4,3)).T 

            frame_depth = np.frombuffer(msg[3], np.uint16).reshape((512, 512))
            frame_ab = np.frombuffer(msg[4], np.uint16).reshape((512, 512))
            
            tool_tracker.add_frame(frame_depth, frame_ab, pose, timestamp)
            
            frame_depth_colored = cv2.applyColorMap(cv2.convertScaleAbs(frame_depth, alpha=0.25), cv2.COLORMAP_JET)
            frame__ab_colored = cv2.applyColorMap(cv2.convertScaleAbs(frame_ab, alpha=0.5), cv2.COLORMAP_JET)
            frame_colored = np.column_stack((frame_depth_colored, frame__ab_colored))

            cv2.imshow('HL2_Depth', frame_colored)
            cv2.waitKey(5)
        except KeyboardInterrupt:
            print("Exiting...")
            tool_tracker.tracking_ir_thread.stop()
            break
        except zmq.ZMQError as e:
            print("ZMQ error:", e)
            continue
        except Exception as e:
            print(e)
            print("Exiting...")
            tool_tracker.tracking_ir_thread.stop()
            break
        
    socket.close()
    socket_cmd.close() 
    context.term()
    exit(1)


def main():
    parser = argparse.ArgumentParser(description='Save Dir.')
    parser.add_argument("--path",
                        required=True,
                        help="Path to recording folder")
    args = parser.parse_args()
    # receive_depth("10.203.11.115", args.path)
    receive_depth("169.254.153.75", args.path)
    


if __name__ == "__main__":
    main()