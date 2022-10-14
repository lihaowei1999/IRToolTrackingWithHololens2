from .My_classes import *
import os
import copy
from .AHATNDITracker import *
import copy
import threading
from scipy.spatial.transform import Rotation as R
import zmq
from pathlib import Path
import time

class AHATIRToolTracking(threading.Thread):
    def __init__(self,_AHATCurrentFrame,Tools,ToolNames,port,sphere_radius,depth_offset):
        super(AHATIRToolTracking,self).__init__()
        self.AHATCurrentFrame=_AHATCurrentFrame
        self._upperlim=256*20
        self._downlim=256*1.5
        self._minSize=10
        self._maxSize=180
        self._distanceTolerance=8
        self._preretval=0
        self._prelabel=[]
        self._prestats=[]
        self._precentroids=[]
        self._prelabelcorr=[]
        self.Tools=Tools
        self.ToolNames=ToolNames
        self._start=False
        self.root_ws = Path(os.path.dirname(os.path.realpath(__file__)))

        self.sphere_radius=sphere_radius
        self.depth_offset=depth_offset

        self.flipx = np.ones((4,4))
        self.flipx[1,0]=-1;self.flipx[0,1]=-1;self.flipx[2,0]=-1;self.flipx[0,2]=-1;self.flipx[0,3]=-1
        self.flipy = np.ones((4,4))
        self.flipy[1,0]=-1;self.flipy[0,1]=-1;self.flipy[2,1]=-1;self.flipy[1,2]=-1;self.flipy[1,3]=-1
        self.flipz = np.ones((4,4))
        self.flipz[2,0]=-1;self.flipz[0,2]=-1;self.flipz[1,2]=-1;self.flipz[2,1]=-1;self.flipz[2,3]=-1

        self.port_pub = port
        self.pub_lock = threading.Lock()
        self.context = zmq.Context()
        self.socket_pub = self.context.socket(zmq.PUB)
        self.socket_pub.set_hwm(5)
        self.socket_pub.bind("tcp://*:%s" % self.port_pub)
        print("Publisher binded with port %s" % self.port_pub)

        self._startmutex=threading.Lock()
        self.initiate()
        

    def initiate(self):
        self.Scene=AHAT_NDIToolScene()
        # Set searching para
        self.Scene.setPara(6,6)
        self.Scene.LoadTools(self.Tools)
        # _intrin=loadmat(str(self.root_ws / 'AHAT_Para_Python.mat'))
        # self.mtx=_intrin['Mtx']
        # self.distcoef=_intrin['dist']

    def run(self):
        # try:
        with self._startmutex:
            self._start=True
        while True:
            if not self._start:
                return

            if self.AHATCurrentFrame.empty():
                time.sleep(0.010)
                continue
            Curr_fr=self.AHATCurrentFrame.get()
            
            holo_xyd = np.array(Curr_fr.centers)
            holo_xyd[:,2] = holo_xyd[:,2] + self.sphere_radius - self.depth_offset
            holo_xyz = []
            for i in range(len(holo_xyd)):
                temp_vec = np.array([holo_xyd[i,0], holo_xyd[i,1], 1])
                holo_xyz.append(temp_vec / np.linalg.norm(temp_vec) * holo_xyd[i,2] )
            holo_xyz = np.array(holo_xyz)
            _tempScene=AHAT_NDIToolSceneFrame(holo_xyz,holo_xyd,_timestamp=Curr_fr.timestamp)

            self.Scene.SearchTools(_tempScene)

            extrin_rhs = np.row_stack((Curr_fr.pose,np.array([0,0,0,1])))

            tool_pose_rhs = _tempScene.TransformatrixFiltered[0]
            if tool_pose_rhs[3,3]==0:
                time.sleep(0.010)
                continue

            tool_pose_rhs[:3,3] = tool_pose_rhs[:3,3] / 1000.0
            toolInWorld_rhs = extrin_rhs @ tool_pose_rhs
            toolInWorld_lhs = toolInWorld_rhs * self.flipz

            with self.pub_lock:
                self.socket_pub.send_multipart([b'ir_tool', 
                    toolInWorld_lhs[:3,3].astype('<f').tobytes(), # tool in world, unity coord
                    R.from_matrix(toolInWorld_lhs[:3,:3]).as_quat().astype('<f').tobytes()])
            
    
    def stop(self):
        try:
            with self._startmutex:
                self._start=False
            print("End Tracking")
        except Exception as e:
            print(e)
    

    def lookup_lut(self, v, u):
        v1 = int(v)
        v2 = min(v1+1, 511)
        u1 = int(u)
        u2 = min(u1+1, 511)

        kv = v - v1
        ku = u - u1

        vec_v1u1 = self.lut[v1,u1]
        vec_v2u1 = self.lut[v2,u1]
        vec_v1u2 = self.lut[v1,u2]
        vec_v2u2 = self.lut[v2,u2]

        vec = (1-kv)*(1-ku)*vec_v1u1 + (1-kv)*ku*vec_v1u2 + kv*(1-ku)*vec_v2u1 + kv*ku*vec_v2u2

        return vec / np.linalg.norm(vec)


    def update_lut(self, lut):
        self.lut = lut.reshape((512,512,3))
    

    @staticmethod
    def trans_quat_to_mat(trans, quat):
        T = np.eye(4)
        T[:3,:3] = R.from_quat(quat).as_matrix()
        T[:3,3] = trans
        return T

    
    @staticmethod
    def trans_rotm_to_mat(trans, rotm):
        T = np.eye(4)
        T[:3,:3] = rotm
        T[:3,3] = trans
        return T


class AHAT_DataTransferThread(threading.Thread):
    def __init__(self,logger,lower_bound,higher_bound,_RawAHATQueue,_AHATQueue,_AHATForProcessQueue):
        super(AHAT_DataTransferThread, self).__init__()
        self.low_bound=lower_bound
        self.high_bound=higher_bound
        self.logger=logger
        self.RawAHATQueue=_RawAHATQueue
        self.AHATQueue=_AHATQueue
        self.AHATProcessQueue=_AHATForProcessQueue
        self.boundMutex=threading.Lock()
        self._start=False
        self._StartMutex=threading.Lock()
    
    def setbound(self,lower,upper):
        try:
            with self.boundMutex:
                self.high_bound=upper
                self.low_bound=lower
        except Exception as e:
            self.logger.error(e)
    
    def run(self):
        try:
            with self._StartMutex:
                self._start=True
            self.logger.info("Start Transforming data")
            
            while True:
                if not self._start:
                    return
                Frame_this=None
                if (not self.RawAHATQueue.empty()) and (not self.AHATQueue.full()):
                    Ab_low=self.low_bound
                    Ab_high=self.high_bound
                    _RawAHATFrame=self.RawAHATQueue.get()
                    imgab=np.frombuffer(_RawAHATFrame.RawReflectivity,dtype="uint16").reshape(512,512)
                    imgdepth=np.frombuffer(_RawAHATFrame.RawDepth,dtype="uint16").reshape(512,512)
                    _,imgdepth=cv2.threshold(imgdepth,1000,0,cv2.THRESH_TRUNC)
                    imgdepth_processed=np.uint8(imgdepth/4)
                    imgab_processed=copy.deepcopy(imgab)
                    imgab_processed[imgab_processed<Ab_low]=Ab_low
                    imgab_processed[imgab_processed>Ab_high]=Ab_high
                    imgab_processed=np.uint8((imgab_processed-Ab_low)/(Ab_high-Ab_low)*255)
                    Frame_this=AHATFrame(imgdepth_processed,imgab_processed,imgdepth,imgab,_RawAHATFrame.timestamp)
                    # self.AHATQueue.put(AHATFrame(imgdepth_processed,imgab_processed,imgdepth,imgab,_RawAHATFrame.timestamp))
                else:
                    time.sleep(0.005)
                    continue
                
                if not self.AHATProcessQueue.full():
                    self.AHATProcessQueue.put(Frame_this)
                
                if not self.AHATQueue.full():
                    self.AHATQueue.put(Frame_this)

                time.sleep(0.001)

        except Exception as e:
            self.logger.error(e)
    
    def stop(self):
        try:
            with self._StartMutex:
                self._start=False
            self.logger.info("END transforming data")    
        except Exception as e:
            self.logger.error(e)