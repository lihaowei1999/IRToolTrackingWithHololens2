from ast import Str
from asyncio.log import logger
import logging
from signal import pthread_kill
import sys
import os
from numpy.core.defchararray import lower
from numpy.core.numeric import NaN
from My_classes import *
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal, QMutex, QMutexLocker
from PyQt5.QtGui import QImage, QPixmap
from QPlainTextEditLogger import QPlainTextEditLogger
from mainWindow import Ui_MainWindow
import queue
import copy
from scipy.io import savemat
from AHATNDITracker import *
import copy



class AHATIRToolTracking(QThread):
    SignalTool=pyqtSignal(tuple)
    SignalFPS=pyqtSignal(int)
    def __init__(self,logger,_AHATCurrentFrame,Tools,ToolNames):
        super(AHATIRToolTracking,self).__init__()
        self.logger=logger 
        self.AHATCurrentFrame=_AHATCurrentFrame
        self._upperlim=256*15
        self._downlim=256*1.5
        self._minSize=10
        self._maxSize=300
        self._distanceTolerance=12
        self._preretval=0
        self._prelabel=[]
        self._prestats=[]
        self._precentroids=[]
        self._prelabelcorr=[]
        self.Tools=Tools
        self.ToolNames=ToolNames
        self._start=False
        self._startmutex=QMutex()
        self.Fpscal=SimpleFPSCalculator()
        self.initiate()
        

    def initiate(self):
        self.Scene=AHAT_NDIToolScene()
        # Set searching para
        self.Scene.setPara(6,6)
        self.Scene.LoadTools(self.Tools)
        _intrin=loadmat('AHAT_Para_Python.mat')
        self.mtx=_intrin['Mtx']
        self.distcoef=_intrin['dist']

    def run(self):
        # try:
        with QMutexLocker(self._startmutex):
            self._start=True
        while True:
            if not self._start:
                return
            if self.AHATCurrentFrame.empty():
                QThread.msleep(10)
                continue
            Curr_fr=self.AHATCurrentFrame.get()
            _Frame=Curr_fr.MatReflectivity
            _DepthFrame=Curr_fr.MatDepth
            
            # _im=_Frame
            _im=copy.deepcopy(_Frame)
            _im[_im>=self._upperlim]=self._upperlim
            _im[_im<=self._downlim]=self._downlim
            _im=(_im-self._downlim)*(256*256/self._upperlim)/(self._upperlim-self._downlim)*255
            _mask=np.array(_im/256,dtype='uint8')
            _im[_mask==0]=0
            _displayTag=np.zeros([512,512])
            # retval,labels,stats,centroids=cv2.connectedComponentsWithStats(_mask,connectivity=8)
            retval,labels,stats,centroids,label_corr=DetectBalls(_mask,self._minSize,self._maxSize,self._distanceTolerance,self._preretval,[],[],self._precentroids,self._prelabelcorr)
            self._preretval=retval
            self._precentroids=centroids
            self._prelabelcorr=label_corr
            # print(centroids)
            _temp_xy = cv2.undistortPoints(centroids,self.mtx,self.distcoef)
            _xyd=[]
            _xyz=[]
            for j in range(centroids.shape[0]):
                _u=centroids[j,0]
                _v=centroids[j,1]
                _d=_DepthFrame[int(_v),int(_u)]
                _xyd.append([_temp_xy[j][0][0],_temp_xy[j][0][1],_d])
                # _ori_xyz=[_temp_xy[j][0][0]*_d,_temp_xy[j][0][1]*_d,_d]
                # _l=np.sqrt(sum(np.array(_ori_xyz)**2))
                _ori_xyz=[_temp_xy[j][0][0],_temp_xy[j][0][1],1]
                _l=np.sqrt(sum(np.array(_ori_xyz)**2))
                _real_xyz=_ori_xyz/_l*_d
                _lreal=np.sqrt(sum(np.array(_real_xyz)**2))
                # _xyz.append(list(_ori_xyz/_l*(_l+5.82)))

                _xyz.append(list(_real_xyz/_lreal*(_lreal+5.82)))
                
                
                # _ori_xyz=[_temp_xy[j][0][0]*_d,_temp_xy[j][0][1]*_d,_d]
                # _l=np.sqrt(sum(np.array(_ori_xyz)**2))
                # _xyz.append(list(_ori_xyz/_l*(_l+5.82)))
            _tempScene=AHAT_NDIToolSceneFrame(np.array(_xyz),np.array(_xyd),_timestamp=Curr_fr.timestamp)
            _tempScene.PixelXYData=centroids
            res_status,res_TransformMatrix,res_err,solu=self.Scene.SearchTools(_tempScene)
            fps=self.Fpscal.framerate(time.time())
            if fps>0:
                self.SignalFPS.emit(fps)
            # print(_tempScene.SolutionPoints)


            # print("In thread")
            # print(_tempScene.timestamp)
            # print(res_status)
            # print(res_TransformMatrix)
            # print(res_err)
            # print(solu)

            # _x=[]
            # _y=[]
            # _z=[]
            # _c=[]
            # cs=['#0000FF','#008000','#FF0000','#0000FF','#008000','#FF0000']
            # for i in range(len(self.Scene.Tools)):
            #     if res_status[i]:
            #         for j in range(len(solu[i])):
            #             _x.append(_xyz[solu[i][j]][0])
            #             _y.append(_xyz[solu[i][j]][1])
            #             _z.append(_xyz[solu[i][j]][2])
            #             _c.append(cs[i])


            # self.SignalTool.emit((res_status,res_err,res_TransformMatrix,_tempScene))
            self.SignalTool.emit((_tempScene,))
            QThread.msleep(1)

        # except Exception as e:
        #     print(e)
        #     self.logger.error(e)
    
    def stop(self):
        try:
            with QMutexLocker(self._startmutex):
                self._start=False
            self.logger.info("End Tracking")

        except Exception as e:
            self.logger.error(e)

class AHATDataCollectThread(QThread):
    Signal_collectPercentag=pyqtSignal(int)
    def __init__(self,logger,AHATDisplayFrame,frames,_pth):
        super(AHATDataCollectThread,self).__init__()
        self.logger=logger
        self.AHATDisplayFrame=AHATDisplayFrame
        self.allframenum=frames
        self.framenum=frames
        self.savingpath=_pth
        self._start=False
        self.result=[]
        self.continueFalseEnum=0
        self._startMutex=QMutex()

    def run(self):
        with QMutexLocker(self._startMutex):
            self._start=True
        self.logger.info("Start Collecting")
        while self.framenum>0:
            if not self._start:
                self.Signal_collectPercentag.emit(0)
                self.logger.info("Stop Collecting")
                return
            
            if not self.AHATDisplayFrame.empty():
                self.continueFalseEnum=0
                _thisFrame=self.AHATDisplayFrame.get()
                self.result.append(_thisFrame)
                self.framenum-=1
                self.Signal_collectPercentag.emit(int((self.allframenum-self.framenum)/self.allframenum*100))
            else:
                self.continueFalseEnum+=1
            QThread.msleep(10)
            if self.continueFalseEnum>200:
                
                self.logger.error("Not Streaming, Please start the stream and re-try")
                return
        if self.framenum==0:
            self.logger.info("Start Saving")
            np.save(self.savingpath,self.result)
            self.logger.info("End Saving")

    def stop(self):
        try:
            with QMutexLocker(self._startMutex):
                self._start=False  
        except Exception as e:
            self.logger.error(e)

class AHATDataPerfusionThread(QThread):
    SignalPercentage=pyqtSignal(int)
    def __init__(self,logger,raw_AHAT_Queue,_path):
        super(AHATDataPerfusionThread,self).__init__()
        self.logger=logger
        self.RawAHATQueue=raw_AHAT_Queue
        self.DataPath=_path
        self._start=False
        self._StartMutex=QMutex()
        self.CurentFrame=0
        self.AllFrameNum=0
        self.timeinterv=30 #ms
        self.Data=None
        self.FpsCal=None
    
    def run(self):
        with QMutexLocker(self._StartMutex):
            self._start=True
        if not os.path.exists(self.DataPath):
            self.logger.error("No Data File : "+self.DataPath+ "\n Please Check")
            return
        self.Data=np.load(self.DataPath,allow_pickle=True)
        self.AllFrameNum=len(self.Data)
        self.logger.info("Start Data input")
        while True:
            if not self._start:
                self.logger.info("Data Input Stop")
                return
            temp_AHATDisplayFrame=self.Data[self.CurentFrame]
            self.SignalPercentage.emit(int(100*self.CurentFrame/self.AllFrameNum))
            self.CurentFrame+=1
            if self.CurentFrame>=self.AllFrameNum:
                self.CurentFrame=0
            temp_RawFrame=AHATRawFrame(temp_AHATDisplayFrame.MatDepth,temp_AHATDisplayFrame.MatReflectivity,temp_AHATDisplayFrame.timestamp)
            if not self.RawAHATQueue.full():
                self.RawAHATQueue.put(temp_RawFrame)
            QThread.msleep(self.timeinterv)

    def stop(self):
        with QMutexLocker(self._StartMutex):
            self._start=False

class AHAT_ImageDisplayThread(QThread):
    Singal_img=pyqtSignal(tuple)
    Signal_fps=pyqtSignal(int)
    def __init__(self,logger,_AHAT_Queue,_CurrentDisplay):
        super(AHAT_ImageDisplayThread, self).__init__()
        self.logger=logger
        # self.imgDepth=img_depth_queue
        # self.imgAb=img_ab_queue
        self.AHATQueue=_AHAT_Queue
        self._start=False
        self._StartMutex=QMutex()
        self._display_tm=[]
        self._fps_display_time=0
        self.CurrentDisplayFrame=_CurrentDisplay
        
    def run(self):
        try:
            with QMutexLocker(self._StartMutex):
                self._start=True
            self.logger.info("Start Displaying")
            while True:
                if not self._start:
                    return
                
                if not self.AHATQueue.empty():
                    AHATFrame=self.AHATQueue.get()
                    imDepth=cv2.cvtColor(AHATFrame.MatDepthProcessed,cv2.COLOR_GRAY2RGB)
                    imAb=cv2.cvtColor(AHATFrame.MatReflectivityProcessed,cv2.COLOR_GRAY2RGB)
                    pix_depth=QtGui.QPixmap(QImage(imDepth.data,512,512,QImage.Format_RGB888))
                    pix_Ab=QtGui.QPixmap(QImage(imAb.data,512,512,QImage.Format_RGB888))
                    self.Singal_img.emit(("depth",pix_depth,"ab",pix_Ab))
                    self._display_tm.append(time.time())
                    if len(self._display_tm)>40:
                        self._display_tm.pop(0)
                        _fps=int(len(self._display_tm)/(self._display_tm[-1]-self._display_tm[0]))
                        if time.time()>(self._fps_display_time+1):
                            self.Signal_fps.emit(_fps)
                            self._fps_display_time=time.time()
                    if self.CurrentDisplayFrame.full():
                        _=self.CurrentDisplayFrame.get()
                    self.CurrentDisplayFrame.put(AHATFrame)
                    
                else:
                    QThread.msleep(10)
        except Exception as e:
            self.logger.error(e)
    
    def stop(self):
        try:
            with QMutexLocker(self._StartMutex):
                self._start=False
            if not self.CurrentDisplayFrame.empty():
                _=self.CurrentDisplayFrame.get()
            self.logger.info("END Displaying")    
        except Exception as e:
            self.logger.error(e)

class AHAT_DataFetchingThread(QThread):
    def __init__(self,logger,socket,raw_AHAT_Queue):
        super(AHAT_DataFetchingThread, self).__init__()
        self.socket=socket
        self.logger=logger
        self.RawAHATQueue=raw_AHAT_Queue
        self._start=False
        self._StartMutex=QMutex()
    
    def run(self):
        try:
            with QMutexLocker(self._StartMutex):
                self._start=True
            self.logger.info("Start Collecting data")
            while True:
                if not self._start or not self.socket._CONNECTED:
                    return     
                if not self.RawAHATQueue.full():
                    _AHATRaw=self.socket.require()
                    self.RawAHATQueue.put(_AHATRaw)
                    
                else:
                    QThread.msleep(20)
        except Exception as e:
            self.logger.error(e)
    
    def stop(self):
        try:
            with QMutexLocker(self._StartMutex):
                self._start=False
            self.logger.info("END Collecting data")    
        except Exception as e:
            self.logger.error(e)

class AHAT_DataTransferThread(QThread):
    def __init__(self,logger,lower_bound,higher_bound,_RawAHATQueue,_AHATQueue,_AHATForProcessQueue):
        super(AHAT_DataTransferThread, self).__init__()
        self.low_bound=lower_bound
        self.high_bound=higher_bound
        self.logger=logger
        self.RawAHATQueue=_RawAHATQueue
        self.AHATQueue=_AHATQueue
        self.AHATProcessQueue=_AHATForProcessQueue
        self.boundMutex=QMutex()
        self._start=False
        self._StartMutex=QMutex()
    
    def setbound(self,lower,upper):
        try:
            with QMutexLocker(self.boundMutex):
                self.high_bound=upper
                self.low_bound=lower
        except Exception as e:
            self.logger.error(e)
    
    def run(self):
        try:
            with QMutexLocker(self._StartMutex):
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
                    QThread.msleep(5)
                    continue
                
                if not self.AHATProcessQueue.full():
                    self.AHATProcessQueue.put(Frame_this)
                
                if not self.AHATQueue.full():
                    self.AHATQueue.put(Frame_this)

                QThread.msleep(1)

        except Exception as e:
            self.logger.error(e)
    
    def stop(self):
        try:
            with QMutexLocker(self._StartMutex):
                self._start=False
            self.logger.info("END transforming data")    
        except Exception as e:
            self.logger.error(e)

class VLC_DataFetchingThread(QThread):
    def __init__(self,logger,socket,RawVLCQueue):
        super(VLC_DataFetchingThread, self).__init__()
        self.socket=socket
        self.logger=logger
        self._VLCQueue=RawVLCQueue
        self._start=False
        self._StartMutex=QMutex()
    
    def run(self):
        try:
            with QMutexLocker(self._StartMutex):
                self._start=True
            self.logger.info("Start Collecting data")
            while True:
                if not self._start or not self.socket._CONNECTED:
                    return
                
                if (not self._VLCQueue.full()):
                    _RawFrame=self.socket.require()
                    self._VLCQueue.put(_RawFrame)
                    # self._VLCQueue.put(_RawFrame.RawVLC)
                    # print(_RawFrame.timestamp)
                else:
                    QThread.msleep(20)
                
        except Exception as e:
            self.logger.error(e)
    
    def stop(self):
        try:
            with QMutexLocker(self._StartMutex):
                self._start=False
            self.logger.info("END Collecting data")    
        except Exception as e:
            self.logger.error(e)

class VLC_DataTransferThread(QThread):
    def __init__(self,logger,raw_VLC_queue,img_VLC_queue,_rot):
        super(VLC_DataTransferThread, self).__init__()
        self.logger=logger
        self.raw_VLC=raw_VLC_queue
        self.img_VLC=img_VLC_queue
        self._start=False
        self._StartMutex=QMutex()
        self.rot=_rot
    
    def run(self):
        try:
            with QMutexLocker(self._StartMutex):
                self._start=True
            self.logger.info("Start Transforming data")
            
            while True:
                if not self._start:
                    return
                if (not self.raw_VLC.empty()) and (not self.img_VLC.full()):
                    _RawFrame=self.raw_VLC.get()
                    _VLC=np.rot90(np.frombuffer(_RawFrame.RawVLC,dtype="uint8").reshape(480,640),self.rot)
                    self.img_VLC.put(VLCFrame(_VLC,_VLC,_RawFrame.timestamp))
                else:
                    QThread.msleep(20)
        except Exception as e:
            self.logger.error(e)
    
    def stop(self):
        try:
            with QMutexLocker(self._StartMutex):
                self._start=False
            self.logger.info("END transforming data")    
        except Exception as e:
            self.logger.error(e)

class VLC_ImageDisplayThread(QThread):
    Singal_img=pyqtSignal(tuple)
    Signal_fps=pyqtSignal(int)
    def __init__(self,logger,img_VLC_queue,_CurrentDisplay):
        super(VLC_ImageDisplayThread, self).__init__()
        self.logger=logger
        self.imgVLC=img_VLC_queue
        self._start=False
        self._StartMutex=QMutex()
        self._display_tm=[]
        self._fps_display_time=0
        self.CurrentDisplayFrame=_CurrentDisplay
        
    
    def run(self):
        try:
            with QMutexLocker(self._StartMutex):
                self._start=True
            self.logger.info("Start Displaying")
            while True:
                if not self._start:
                    return
                if (not self.imgVLC.empty()):
                    _FrameVLC=self.imgVLC.get()
                    _VLC=cv2.cvtColor(_FrameVLC.MatVLC,cv2.COLOR_GRAY2RGB)
                    _PIX=QtGui.QPixmap(QImage(_VLC.data,480,640,QImage.Format_RGB888))
                    self.Singal_img.emit(("VLC",_PIX))
                    self._display_tm.append(time.time())
                    if len(self._display_tm)>40:
                        self._display_tm.pop(0)
                        _fps=int(len(self._display_tm)/(self._display_tm[-1]-self._display_tm[0]))
                        if time.time()>(self._fps_display_time+1):
                            self.Signal_fps.emit(_fps)
                            self._fps_display_time=time.time()
                    if self.CurrentDisplayFrame.full():
                        _=self.CurrentDisplayFrame.get()
                    self.CurrentDisplayFrame.put(_FrameVLC)
                    
                    
                else:
                    QThread.msleep(20)
        except Exception as e:
            self.logger.error(e)
    
    def stop(self):
        try:
            with QMutexLocker(self._StartMutex):
                self._start=False
            if not self.CurrentDisplayFrame.empty():
                _=self.CurrentDisplayFrame.get()
            self.logger.info("END Displaying")    
        except Exception as e:
            self.logger.error(e)