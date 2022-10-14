from typing import NewType
from numpy.lib.function_base import disp
import socket
import numpy as np
import scipy.io as io


class AHATRawFrame:
    def __init__(self,_RawDepth,_RawReflectivity,_timestamp):
        self.RawDepth=_RawDepth
        self.RawReflectivity=_RawReflectivity
        self.timestamp=_timestamp
        
class AHATFrame:
    def __init__(self,_centers,_timestamp, _pose):
        self.timestamp=_timestamp
        self.pose = _pose
        self.centers = _centers
        
class VLCRawFrame:
    def __init__(self,_RawVLC,_timestamp):
        self.RawVLC=_RawVLC
        self.timestamp=_timestamp
        
class VLCFrame:
    def __init__(self,_MatVLC,_MatVLCOrigin,_timestamp):
        self.MatVLC=_MatVLC
        self.MatOrigin=_MatVLCOrigin
        self.timestamp=_timestamp

        
class SensorType:
    AHAT_CAMERA=4
    LEFT_FRONT=0
    RIGHT_FRONT=1
    LEFT_LEFT=2
    RIGHT_RIGHT=3

class Sensor_Network:
    def __init__(self,port,sensortype):
        self._SOCKET=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self._PORT=port
        self._TYPE=sensortype
        self._ADDR=("",port)
        self._CONNECTED=False
    
    def connect(self):
        self._SOCKET.bind(self._ADDR)
        self._SOCKET.listen(1)
        self._CONN,addr=self._SOCKET.accept()
        self._CONNECTED=True
        return addr 

    def _send(self,mess):
        self._CONN.send(mess.encode("UTF-8"))
    
    def require(self):
        if self._TYPE==SensorType.AHAT_CAMERA:
            mess = "Recv image"
            self._send(mess)
            Ab_data_raw=b''
            Depth_data_raw=b''
            Time_data=b''
            len_tm=32
            while len_tm:
                buf=self._CONN.recv(len_tm)
                if not buf:
                    print("ERROR when recving")
                    return None
                Time_data+=buf
                len_tm-=len(buf)
            
            ## 52488=512*512*2 
            len_= 524288
            while len_:
                buf=self._CONN.recv(len_)
                if not buf:
                    print("ERROR when recving")
                    return None
                Ab_data_raw+=buf
                len_-=len(buf)
            
            len_= 524288
            while len_:
                buf=self._CONN.recv(len_)
                if not buf:
                    print("ERROR when recving")
                    return None
                Depth_data_raw+=buf
                len_-=len(buf)
            #print(Time_data)
            return AHATRawFrame(Depth_data_raw,Ab_data_raw,int(Time_data.decode("UTF8")))
            # return (Ab_data_raw, Depth_data_raw, Time_data)
        else:
            len_=640*480
            mess="Recv image"
            
            self._send(mess)
            #print("Req")
            img_raw=b''
            Time_data=b''
            len_tm=32
            while len_tm:
                buf=self._CONN.recv(len_tm)
                if not buf:
                    print("ERROR when recving")
                    return None
                Time_data+=buf
                len_tm-=len(buf)

            while len_:
                buf=self._CONN.recv(len_)
                if not buf:
                    print("ERROR when recving")
                    return None
                img_raw+=buf
                len_-=len(buf)
                # print(len_)
            # print(int(Time_data.decode("UTF-8")))
            return VLCRawFrame(img_raw,int(Time_data.decode("UTF-8")))
            # return (img_raw,Time_data)

class SimpleFPSCalculator:
    def __init__(self):
        self.FrameTime=[]
        
    def framerate(self,_tm):
        self.FrameTime.append(_tm)
        if len(self.FrameTime)>40:
            self.FrameTime.pop(0)
            return int(len(self.FrameTime)/(self.FrameTime[-1]-self.FrameTime[0]))
        else:
            return -1
