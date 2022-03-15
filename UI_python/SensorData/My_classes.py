from typing import NewType
from numpy.lib.function_base import disp
from sksurgerynditracker.nditracker import NDITracker
import socket
import time
import math
import numpy as np
import scipy.io as io
from tqdm import tqdm
import time
import pyquaternion
from pyquaternion import Quaternion
# from autolab_core import RigidTransform
# import cv2
import cv2
from autolab_core import RigidTransform
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


class control_enum:
    create_axis=5
    create_axis_fb=6
    receive_axis=15
    get_hololens_pos=20
    receive_hololens_pos=25
    set_position_6d=30
    set_single_object=50
    set_single_object_to_head=60
    set_unity_frame_delay=70
    set_slerp_percentage=71
    set_interplote_percentage=72
    

    

class code_generater:
    def __init__(self,control,para):
        self.control=control
        self.para=para
        self.control_dictionary=control_enum()

    def encode_mess(self):
        if self.control==self.control_dictionary.create_axis or self.control==self.control_dictionary.create_axis_fb:
            str_this = self.int2str(self.control,4)+self.float2str(self.para[0],16,16)+self.float2str(self.para[1],16,16)+self.float2str(self.para[2],16,16)+"0"*1024
            str_this=str_this[0:1024]
            return str_this
        
        elif self.control==self.control_dictionary.get_hololens_pos:
            str_this=self.int2str(self.control,4)+"0"*1024
            str_this=str_this[0:1024]
            return str_this
        
        elif self.control==self.control_dictionary.set_position_6d:
            str_this = self.int2str(self.control,4)+self.float2str(self.para[0],16,16)+self.float2str(self.para[1],16,16)+self.float2str(self.para[2],16,16)\
               +self.float2str(self.para[3],16,16)+self.float2str(self.para[4],16,16)+self.float2str(self.para[5],16,16)+self.float2str(self.para[6],16,16)+"0"*1024
            str_this=str_this[0:1024]
            return str_this
        
        elif self.control==self.control_dictionary.set_single_object:
            ## here
            str_this = self.int2str(self.control,4)+\
                self.int2str(self.para[0],4)+\
                self.float2str(self.para[1],16,16)+\
                self.float2str(self.para[2],16,16)+\
                self.float2str(self.para[3],16,16)+\
                self.float2str(self.para[4],16,16)+\
                self.float2str(self.para[5],16,16)+\
                self.float2str(self.para[6],16,16)+\
                self.float2str(self.para[7],16,16)+"0"*1024
            str_this=str_this[0:1024]
            return str_this

        elif self.control==self.control_dictionary.set_single_object_to_head:
            ## here
            str_this = self.int2str(self.control,4)+\
                self.int2str(self.para[0],4)+\
                self.float2str(self.para[1],16,16)+\
                self.float2str(self.para[2],16,16)+\
                self.float2str(self.para[3],16,16)+\
                self.float2str(self.para[4],16,16)+\
                self.float2str(self.para[5],16,16)+\
                self.float2str(self.para[6],16,16)+\
                self.float2str(self.para[7],16,16)+"0"*1024
            str_this=str_this[0:1024]
            return str_this

        elif self.control==self.control_dictionary.set_unity_frame_delay:
            str_this = self.int2str(self.control,4)+\
                self.int2str(self.para[0],4)+"0"*1024
            str_this=str_this[0:1024]
            return str_this

        elif self.control==self.control_dictionary.set_slerp_percentage:
            str_this = self.int2str(self.control,4)+\
                self.float2str(self.para[0],16,16)+"0"*1024
            str_this=str_this[0:1024]
            return str_this

        elif self.control==self.control_dictionary.set_interplote_percentage:
            str_this = self.int2str(self.control,4)+\
                self.float2str(self.para[0],16,16)+"0"*1024
            str_this=str_this[0:1024]
            return str_this

            



    def float2str(self,num,int_,deci_):
        sgn_code="0" if num < 0 else "1"
        num = num if num >= 0 else -num
        int_part=math.floor(num)
        deci_part=math.floor((num-int_part)*10**deci_)
        str_this=sgn_code+"{:0>{width}}".format(int_part, width=int_)+"{:0>{width}}".format(deci_part, width=deci_)
        
        return str_this
    def int2str(self,num,int_):
        return "{:0>{width}}".format(num, width=int_) 

class code_decoder:
    def __init__(self,str_):
        self.message=str_
        self.control_dictionary=control_enum()

    def decode_mess(self,display=False):
        control_seq=self.message[0:4]
        control=self.str2int(control_seq)
        if control == self.control_dictionary.create_axis or control == self.control_dictionary.create_axis_fb or control==self.control_dictionary.receive_axis:
            x_pos=self.str2float(self.message[4:37],16,16)
            y_pos=self.str2float(self.message[37:70],16,16)
            z_pos=self.str2float(self.message[70:103],16,16)
            if display:
                print("control:  %d"%control)
                print("x_pos:    %f"%x_pos)
                print("y_pos:    %f"%y_pos)
                print("z_pos:    %f"%z_pos)
            return control,[x_pos,y_pos,z_pos]
        
        elif control==self.control_dictionary.receive_hololens_pos:
            x_pos=self.str2float(self.message[4:37],16,16)
            y_pos=self.str2float(self.message[37:70],16,16)
            z_pos=self.str2float(self.message[70:103],16,16)
            qw=self.str2float(self.message[103:136],16,16)
            qx=self.str2float(self.message[136:169],16,16)
            qy=self.str2float(self.message[169:202],16,16)
            qz=self.str2float(self.message[202:235],16,16)
            pos=[x_pos,y_pos,z_pos]
            quat=[qw,qx,qy,qz]
            if display:
                print("control:  %d"%control)
                print("x_pos:    %f"%x_pos)
                print("y_pos:    %f"%y_pos)
                print("z_pos:    %f"%z_pos)
                print("qw   :    %f"%qw)
                print("qx   :    %f"%qx)
                print("qy   :    %f"%qy)
                print("qz   :    %f"%qz)
            return control,[x_pos,y_pos,z_pos,qw,qx,qy,qz]
    
    def str2int(self,substring):
        
        return int(substring)
        
    def str2float(self,substring_,int_,deci_):
        sgn_part=substring_[0:1]
        int_part=substring_[1:1+int_]
        deci_part=substring_[1+int_:1+int_+deci_]
        sgn_=1 if sgn_part == "1" else -1
        # print(int_part)
        # print(deci_part)
        num=sgn_*(int(int_part)+int(deci_part)*10**(-deci_))
        return num

class HololensTrackingObject:
    def __init__(self,TrackerFile,IP,PortUp=3000,PortDown=4000):
        self.TrackerFile=TrackerFile
        self.IP=IP 
        self.PortUp=PortUp
        self.PortDown=PortDown
        self.UpSocket=socket_connecter(self.IP,self.PortUp)
        self.DownSocket=socket_connecter(self.IP,self.PortDown)
        self.Connected=False
        
    def ConnectSocketChannel(self):
        if not self.Connected:
            self.UpSocket.connect()
            self.DownSocket.connect()
            self.Connected=True
            print("Connected")
        
    def SetPosition(self,TargetID,TrasnformMatrix):
        Quat=list(Quaternion(matrix=TrasnformMatrix[0:3,0:3]))
        ct=control_enum()
        code=code_generater(ct.set_single_object,[TargetID,TrasnformMatrix[0,3]/1000,-TrasnformMatrix[1,3]/1000,TrasnformMatrix[2,3]/1000,Quat[0],-Quat[1],Quat[2],-Quat[3]])
        cd=code.encode_mess()
        self.UpSocket.send(cd)
        
    def SetPositionToHead(self,TargetID,TrasnformMatrix):
        Quat=list(Quaternion(matrix=TrasnformMatrix[0:3,0:3]))
        ct=control_enum()
        code=code_generater(ct.set_single_object_to_head,[TargetID,TrasnformMatrix[0,3]/1000,-TrasnformMatrix[1,3]/1000,TrasnformMatrix[2,3]/1000,Quat[0],-Quat[1],Quat[2],-Quat[3]])
        
        cd=code.encode_mess()
        self.UpSocket.send(cd)

    def SetUnityDelay(self,delayframenum):
        ct=control_enum()
        code=code_generater(ct.set_unity_frame_delay,[delayframenum,])
        cd=code.encode_mess()
        self.UpSocket.send(cd)

    def SetSlerpPercentage(self,value):
        ct=control_enum()
        code=code_generater(ct.set_slerp_percentage,[value,])
        cd=code.encode_mess()
        self.UpSocket.send(cd)

    def SetInterplotPercentage(self,value):
        ct=control_enum()
        code=code_generater(ct.set_interplote_percentage,[value,])
        cd=code.encode_mess()
        self.UpSocket.send(cd)

    def GetHololensPosition(self):
        ct=control_enum()
        code=code_generater(ct.get_hololens_pos,[])
        cd=code.encode_mess()
        self.UpSocket.send(cd)
        mess_back=self.DownSocket.recv()
        msb_decoder=code_decoder(mess_back)
        control_re,info_re=msb_decoder.decode_mess(False)
        return info_re

        
        
class TrackingTarget:
    def __init__(self,Name,TargetName,InitiateStatus,InitiatePosition,TrackingPoints,ROMFile):
        self.Name=Name
        self.TargetName=TargetName
        self.Status=InitiateStatus
        self.Position=InitiatePosition
        self.Rotationquat=[1,0,0,0]
        self.TrackingPoints=TrackingPoints
        self.ROMFile=ROMFile
        self.PointsinTracker=np.zeros((4,3))
        self.MatrixModeltoTracker=np.zeros((4,4))
        self.HasPoint=[False,False,False,False]
        self.PointError=[]
        self.SumErr=0


    def RegistrationModelTracker(self):
        print("Data display *******")
        print(self.TrackingPoints)
        print(self.PointsinTracker)
        self.MatrixModeltoTracker,self.SumErr,self.PointError=UVD_RigidTransform(self.TrackingPoints,self.PointsinTracker)


    def SetPoint(self,id,point):
        self.PointsinTracker[id]=point
        if not self.HasPoint[id]:
            self.HasPoint[id]=True
        if all(self.HasPoint):
            self.RegistrationModelTracker()
    
    def fetchresult(self):
        if all(self.HasPoint):
            return (True,self.MatrixModeltoTracker,self.PointError,self.SumErr)
        else:
            return (False,self.HasPoint)

class socket_connecter:
    def __init__(self,ip_address,port):
        self.tcp_socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.server_addr=(ip_address,port)
        
    def connect(self):
        print("trying to connect to :")
        print(self.server_addr)
        self.tcp_socket.connect(self.server_addr)
        print("connect success")

    def send(self,mess):
        self.tcp_socket.send(mess.encode("UTF-8"))

    def recv(self,length=1024):
        mess=self.tcp_socket.recv(1024)
        mess_= mess.decode("UTF-8")
        return mess_


class AHATRawFrame:
    def __init__(self,_RawDepth,_RawReflectivity,_timestamp):
        self.RawDepth=_RawDepth
        self.RawReflectivity=_RawReflectivity
        self.timestamp=_timestamp
        
class AHATFrame:
    def __init__(self,_MatDepthProcessed,_MatReflectivityProcessed,_MatDepth,_MatReflectivity,_timestamp):
        self.MatDepth=_MatDepth
        self.MatReflectivity=_MatReflectivity
        self.MatDepthProcessed=_MatDepthProcessed
        self.MatReflectivityProcessed=_MatReflectivityProcessed
        self.timestamp=_timestamp
        
class VLCRawFrame:
    def __init__(self,_RawVLC,_timestamp):
        self.RawVLC=_RawVLC
        self.timestamp=_timestamp
        
class VLCFrame:
    def __init__(self,_MatVLC,_MatVLCOrigin,_timestamp):
        self.MatVLC=_MatVLC
        self.MatOrigin=_MatVLCOrigin
        self.timestamp=_timestamp

class VisualSensorFrames:
    def __init__(self,_AHAT,_LL,_LF,_RR,_RF,_comment=""):
        self.AHAT=_AHAT
        self.LL=_LL
        self.LF=_LF
        self.RR=_RR
        self.RF=_RF
        self.comment=_comment
        
class SensorType:
    AHAT_CAMERA=4
    LEFT_FRONT=0
    RIGHT_FRONT=1
    LEFT_LEFT=2
    RIGHT_RIGHT=3

class Sensor_reconstructor_receiver:
    def __init__(self,ip_address,port):
        self._SOCKET=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self._IP=ip_address
        self._PORT=port
        self._ADDR=(ip_address,port)
        self._CONNECTED=False
        
    def connect(self):
        self._SOCKET.bind(self._ADDR)
        self._SOCKET.listen(1)
        self._CONN,addr=self._SOCKET.accept()
        self._CONNECTED=True
        return addr 
    def _send(self,mess):
        self._CONN.send(mess.encode("UTF-8"))

    def _getxy(self,_u,_v):
        _temp=self.float2str(_u,8,8)+self.float2str(_v,8,8)+'0'*512
        _temp=_temp[0:512]
        self._send(_temp)
        len_recv=512
        mess_recv=''
        while len_recv:
            buf=self._CONN.recv(len_recv)
            if not buf:
                print("ERROR when recving")
                return None
            
    
    def int2str(self,num,int_):
        return "{:0>{width}}".format(num, width=int_)
    
    def float2str(self,num,int_,deci_):
        num = num if num >= 0 else -num
        int_part=math.floor(num)
        deci_part=math.floor((num-int_part)*10**deci_)
        str_this="{:0>{width}}".format(int_part, width=int_)+"{:0>{width}}".format(deci_part, width=deci_)
        return str_this
    
class Sensor_Network:
    def __init__(self,ip_address,port,sensortype):
        self._SOCKET=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self._IP=ip_address
        self._PORT=port
        self._TYPE=sensortype
        self._ADDR=(ip_address,port)
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


class UWP_socket:
    def __init__(self,ip_address,port):
        self.tcp_socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.connected=False
        self.server_addr=(ip_address,port)
        
    def connect(self):
        self.tcp_socket.bind(self.server_addr)
        self.tcp_socket.listen(3)
        self.conn,addr=self.tcp_socket.accept()
        self.connected=True
        
        
    def send(self,mess):
        self.conn.send(mess.encode("UTF-8"))

    def get_AHAT_image(self):
        mess = "Recv image"
        self.conn.send(mess.encode("UTF-8"))
        Ab_data_raw=b''
        Depth_data_raw=b''
        len_= 524288
        while len_:
            buf=self.conn.recv(len_)
            if not buf:
                return None
            Ab_data_raw+=buf
            len_-=len(buf)
        len_= 524288
        while len_:
            buf=self.conn.recv(len_)
            if not buf:
                return None
            Depth_data_raw+=buf
            len_-=len(buf)
        return Ab_data_raw, Depth_data_raw
        


class NDI_tracker:
    def __init__(self,path):
        self.SETTINGS = {
            "tracker type": "polaris",
            "romfiles" : path
                }
        self.tracker=None
        self.origin_pos=np.array([[221.92],[-20.86],[-56.28],[1]])
    def machine_start(self):
        self.tracker=NDITracker(self.SETTINGS)
        self.tracker.start_tracking()

    def fetch(self):
        port_handles, timestamps, framenumbers, tracking, quality = self.tracker.get_frame()
        return tracking

    def stop(self):
        self.tracker.stop_tracking()
        self.tracker.close()

    def fetch_point(self,i=1):
        tracking=self.fetch()[i-1]
        pt=tracking @self.origin_pos
        pt=pt/pt[3]
        pt=pt[0:3]
        return pt

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

def start_calibrating(show_points,hololens_socket_up,hololens_socket_down,tracker):
    ct=control_enum()
    res_points=[]
    ndi_points=[]
    for i in range(show_points.shape[0]):
        print(i)
        print([show_points[i,0,0],show_points[i,1,0],show_points[i,2,0]])
        code=code_generater(ct.create_axis,[show_points[i,0,0],show_points[i,1,0],show_points[i,2,0]])
        cd=code.encode_mess()
        # hololens_socket_up.send(code)
        # re=hololens_socket_down.recv(1024)
        code_de=code_decoder(cd)
        control,pos=code_de.decode_mess(True)
        position=np.array([[pos[0]],[pos[1]],[pos[2]]])
        res_points.append(position)
        a=input("press enter to contunue")
        ndi_point=tracker.fetch_point()
        ndi_points.append(ndi_point)
    res_points=np.array(res_points)
    ndi_points=np.array(ndi_points)
    print(res_points)
    print(ndi_points)
    return res_points,ndi_points



def plotaxs(ax,matrix,scale):
    point_000_ndi=matrix[0:3,3]
    point_010_ndi=np.zeros(3)
    point_001_ndi=np.zeros(3)
    point_100_ndi=np.zeros(3)
    
    pt_010=np.array([[0],[scale],[0],[1]])
    pt_001=np.array([[0],[0],[scale],[1]])
    pt_100=np.array([[scale],[0],[0],[1]])

    point_001_ndi=matrix[0:3,:]@pt_001
    point_010_ndi=matrix[0:3,:]@pt_010
    point_100_ndi=matrix[0:3,:]@pt_100
    
    ax.plot([point_000_ndi[0],point_001_ndi[0]],[point_000_ndi[1],point_001_ndi[1]],[point_000_ndi[2],point_001_ndi[2]],color='r',linewidth=0.4)
    ax.plot([point_000_ndi[0],point_010_ndi[0]],[point_000_ndi[1],point_010_ndi[1]],[point_000_ndi[2],point_010_ndi[2]],color='b',linewidth=0.4)
    ax.plot([point_000_ndi[0],point_100_ndi[0]],[point_000_ndi[1],point_100_ndi[1]],[point_000_ndi[2],point_100_ndi[2]],color='g',linewidth=0.4)


class HololensNDIRegister():
    def __init__(self):
        self.HololensPose=[]
        self.NDIPose=[]
        self.CurrentPoint=0
        self.PointError=[]
        self.SumError=0
        self.CurrentBase2World=[]
        self.CurrentHololens2tracker=[]
        
        
    def AddPoint(self,Hololens,NDI):
        self.HololensPose.append(Hololens)
        self.NDIPose.append(NDI)
        self.CurrentPoint+=1
        if self.CurrentPoint>=5:
            self.CurrentBase2World,self.CurrentHololens2tracker,self.PointError,self.SumError=\
                hand_eye_calibration(np.array(self.HololensPose),np.array(self.NDIPose))
        return self.CurrentPoint,self.SumError
    
    def DeletePoint(self,id):
        self.HololensPose.pop(id)
        self.NDIPose.pop(id)
        self.CurrentPoint-=1
        if self.CurrentPoint>=5:
            self.CurrentBase2World,self.CurrentHololens2tracker,self.PointError,self.SumError=\
                hand_eye_calibration(np.array(self.HololensPose),np.array(self.NDIPose))
        else:
            self.CurrentBase2World=[]
            self.CurrentHololens2tracker=[]
            self.PointError=[]
            self.SumError=0


    def Snapshot(self):
        return self.CurrentPoint,self.PointError, self.SumError

    def GetResult(self):
        return self.CurrentBase2World,self.CurrentHololens2tracker


def hand_eye_calibration(Hololens_pos,NDI_pos):
    rs_holo=Hololens_pos
    rs_ndi=NDI_pos
    rs_holo_matrix=np.zeros((len(rs_ndi),4,4))
    for i in range(len(rs_ndi)):
        trans=np.array([rs_holo[i,0],-rs_holo[i,1],rs_holo[i,2]])*1000
        quat=np.array([rs_holo[i,3],-rs_holo[i,4],rs_holo[i,5],-rs_holo[i,6]])
        rota=RigidTransform(quat,trans)
        rs_holo_matrix[i,0:3,0:3]=rota.rotation
        rs_holo_matrix[i,0:3,3]=rota.translation
        rs_holo_matrix[i,3,3]=1

    for i in range(len(rs_ndi)-1,-1,-1):
        if math.isnan(rs_ndi[i,0,0]):
            rs_ndi=np.delete(rs_ndi,i,0)
            rs_holo_matrix=np.delete(rs_holo_matrix,i,0)
    ndi2tracker=np.zeros(rs_ndi.shape)
    base2hololens=np.zeros(rs_ndi.shape)
    for i in range(rs_holo_matrix.shape[0]):
        ndi2tracker[i,:,:]=np.linalg.inv(rs_ndi[i,:,:])
        base2hololens[i,:,:]=np.linalg.inv(rs_holo_matrix[i,:,:])

    R_ndi2tracker=ndi2tracker[:,0:3,0:3]
    t_ndi2tracker=ndi2tracker[:,0:3,3]
    R_base2hololens=base2hololens[:,0:3,0:3]
    t_base2hololens=base2hololens[:,0:3,3]


    q=cv2.calibrateRobotWorldHandEye(R_ndi2tracker,t_ndi2tracker,R_base2hololens,t_base2hololens)
    base2world=np.zeros((4,4))
    base2world[0:3,0:3]=q[0]
    base2world[0:3,3]=q[1][0:3,0]
    base2world[3,3]=1
    hololens2tracker=np.zeros((4,4))
    hololens2tracker[0:3,0:3]=q[2]
    hololens2tracker[0:3,3]=q[3][0:3,0]
    hololens2tracker[3,3]=1
    tracker2hololens=np.linalg.inv(hololens2tracker)
    errors=np.zeros((len(rs_ndi)))
    tracker2ndi_from_hololens=np.zeros(rs_ndi.shape)
    for i in range(len(tracker2ndi_from_hololens)):
        tracker2ndi_from_hololens[i,:,:]=base2world@(rs_holo_matrix[i,:,:])@tracker2hololens
    for i in range(len(rs_ndi)):
        errors[i]=math.sqrt(sum(rs_ndi[i,0:3,3]-tracker2ndi_from_hololens[i,0:3,3])**2)
    err_ =   sum(errors)/len(errors)
    return base2world,hololens2tracker,errors,err_