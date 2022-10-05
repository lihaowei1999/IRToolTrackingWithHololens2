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

    socket_cmd = context.socket(zmq.REP)
    socket.setsockopt(zmq.RCVTIMEO, 5000)

    port, port_cmd = "12387", "12388"
    socket.connect("tcp://%s:%s" % (ip_address,port))
    print("Depth subscriber connected  with  %s:%s" % (ip_address,port) )
    #socket.subscribe(topic)

    socket_cmd.bind("tcp://*:%s" % port_cmd)
    print("Command reply server binded  with port %s" % (port_cmd) )

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)
    poller.register(socket_cmd, zmq.POLLIN)

    tool_tracker = None
    lut_received = False
    lut = None

    # ---------------------------------------
    while True:
        try:
            socks = dict(poller.poll(5000))
            # check for lut
            if socket_cmd in socks and socks[socket_cmd] == zmq.POLLIN:
                msg = socket_cmd.recv_multipart()
                print("[IR] Received msg length:", len(msg))
                message_type = msg[0].decode('utf-8')
                print("[IR] Received topic: ", message_type)

                if message_type != "depth_lut":
                    print("Received string: %s" % message_type)
                    continue
                
                print("LUT received. Length: ", len(msg[1]))
                socket_cmd.send_string('OK')
                
                lut = np.frombuffer(msg[1], np.dtype(np.float32))
                lut = np.reshape(lut, (-1, 3))
                # np.save(os.path.join(save_path, "lut"), lut)
                # print("LUT saved.")

                if tool_tracker is None:
                    tool_tracker = IRToolTrack(lut)
                    tool_tracker.track_tool()
                else:
                    tool_tracker.update_lut(lut)
                lut_received = True
            
            # if lut received, start processing depth frame
            if lut_received and socket in socks and socks[socket] == zmq.POLLIN:
                msg = socket.recv_multipart()
                message_type = msg[0].decode('utf-8')
                if message_type != topic:
                    print("Received string: %s" % message_type)
                    continue

                timestamp = struct.unpack(">q", msg[1])[0]  # long long, i.e. 64-bit int
                # print("IR received. Timestamp: ", timestamp)

                cam_pose_array = np.frombuffer(msg[2], np.dtype(np.float32))
                pose =np.array(np.reshape(cam_pose_array,(4,3)).T)
                pose[:3,3] = pose[:3,3] - pose[:3,1] * 0.0015 # add a magic offset 

                frame_depth = np.frombuffer(msg[3], np.uint16).reshape((512, 512))
                frame_ab = np.frombuffer(msg[4], np.uint16).reshape((512, 512))
                
                tool_tracker.add_frame(frame_depth, frame_ab, pose, timestamp)

                # Visualization
                frame_depth_colored = cv2.applyColorMap(cv2.convertScaleAbs(frame_depth, alpha=0.25), cv2.COLORMAP_JET)
                frame__ab_colored = cv2.applyColorMap(cv2.convertScaleAbs(frame_ab, alpha=0.5), cv2.COLORMAP_JET)
                frame_colored = np.column_stack((frame_depth_colored, frame__ab_colored))

                cv2.imshow('HL2_Depth', frame_colored)
                cv2.waitKey(5)
        except KeyboardInterrupt:
            print("\n[LUT] Exiting...")
            break
        except zmq.ZMQError as e:
            print("[LUT] Exception:", e)
            continue
        except Exception as e:
            print("Depth Exception: ", e)
            break
    
    if tool_tracker is not None:
        tool_tracker.tracking_ir_thread.stop()
    socket.close()
    socket_cmd.close() 
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