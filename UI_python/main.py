import logging
import sys
import os
from tkinter.messagebox import NO
from tokenize import Single
from types import FrameType
from typing import Final
from matplotlib.transforms import Transform
from numpy.core.defchararray import lower
from numpy.core.numeric import NaN
from My_classes import *
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QThread, QTimer, pyqtSignal, QMutex, QMutexLocker
from PyQt5.QtGui import QImage, QPixmap
from QPlainTextEditLogger import QPlainTextEditLogger
from mainWindow import Ui_MainWindow
import queue
import copy
from scipy.io import savemat
from AHATNDITracker import *
from ThreadFactory import *
import pyigtl
from scipy.io import savemat,loadmat
from scipy import optimize
import json

class MainDialog(QtWidgets.QMainWindow, Ui_MainWindow):
    
    def __init__(self):
        super(MainDialog, self).__init__()
        self.setupUi(self)

        ## Setup Logger
        self.initLogger()


        self.setWindowFlag(QtCore.Qt.WindowMinMaxButtonsHint)
        self.logger.info(f'System started.')
        
        ## Variant for sensor datas
        self.down_limit_ab=128
        self.up_limit_ab=512
        self._mx=10
        self._mx_vlc=10
        self.RawAHATQueue=queue.Queue(maxsize=self._mx)
        self.AHATQueue=queue.Queue(maxsize=self._mx)
        self.LFRawQueue=queue.Queue(maxsize=self._mx_vlc)
        self.LFImQueue=queue.Queue(maxsize=self._mx_vlc)
        self.LLRawQueue=queue.Queue(maxsize=self._mx_vlc)
        self.LLImQueue=queue.Queue(maxsize=self._mx_vlc)
        self.RFRawQueue=queue.Queue(maxsize=self._mx_vlc)
        self.RFImQueue=queue.Queue(maxsize=self._mx_vlc)
        self.RRRawQueue=queue.Queue(maxsize=self._mx_vlc)
        self.RRImQueue=queue.Queue(maxsize=self._mx_vlc)
        self.AHATForCalFrame=queue.Queue(maxsize=self._mx_vlc)
        self.RR_isworking=False
        self.RF_isworking=False
        self.LF_isworking=False
        self.LL_isworking=False
        self.AHAT_isworking=False
        
        
        
        self.AHATDisplayFrame=queue.Queue(maxsize=1)
        self.RRDisplayFrame=queue.Queue(maxsize=1)
        self.RFDisplayFrame=queue.Queue(maxsize=1)
        self.LLDisplayFrame=queue.Queue(maxsize=1)
        self.LFDisplayFrame=queue.Queue(maxsize=1)
        self.SnapShotFrame=queue.Queue(maxsize=1)
        

        
        self._isTrackingTool=False
        
        self._isDisplaying3Dinformation=False
        

        self.igtDisplayStatus=False
        self.igtlClient=None

        self.CollectAHATDataThread=None
        self.ProvideAHATDataThread=None
        self._isRecordingAHATDATA=False
        self._isProvidingAHATDATA=False

        self._workingDirect="Cache/"
        self.RootPath="Cache/"

        self._iscollectingTrackingData=False
        self._collectedTrakingData=[]


        self._iscollectingToolDefData=False
        self.ToolDefTotalFrame=0
        self.ToolDefCurrentFrame=0
        self.ToolDefData=None
        self.ToolDefPreviewFrame=queue.Queue(maxsize=1)

        self.ToolListRootDir="ToolList/"

        self.ToolInfo=None
        self.ToolNames=None

        self.Init_Signal_and_Slot()
        self.DisplayTestTimer=None
        self.TempDisplayPointPosition=None
        self.TempTranformMatrix=None

        self.registerDataX=0
        self.registerDataY=0
        self.registerDataZ=0
        self.registerXangleData=0
        self.registerYangleData=0
        self.registerZangleData=0
        self.registerTime =0    # delay for unity
        self.slerppercent=0.4
        self.interplotpercent=0.5
        self.KalmanFilterStatus=False
        self.IGTDisplayStatus=False
        self.IGTClient=None

        ## for precision test
        self.PrecisionTestNDI=None
        self.PrecisionTestNDISetting=None
        self.PrecisionTestHololensTrackingQueue=queue.Queue(maxsize=1)
        self.PrecisionTestHololensTrackingQueueMutex=QMutex()
        self.PrecisionTestHololensDataCache=[]


        ## Precision test
        self.PrecisionTestDataHololens=[]
        self.PrecisionTestDataNDI=[]
        self.PrecisionTestSavingRootDir="Test/PrecisionTest"
        
        self.PrecisionTestSingleCaptureHololensCache=[]
        self.PrecisionTestSingleCaptureNDICache=[]
        self.PrecisionTestTrackingDataCollectionTargetNum=2000

        self._isPerceptionDataCollecting=False


        self.TrackingMartrixs=[]


        #self.Img2Tracker=np.load("Model_1_Img_2_Tracker.npy")

        self.Experiment_isCalibrating=False
        self.Experiment_isTrackingObject=False
        self.ExperimentEndTransform=np.diag([1.0,1.0,1.0,1.0])
        self.ControlId=0

# Checked
    def Init_Signal_and_Slot(self):
        self.AHATExperimentTrackingButton.clicked.connect(self.OnAHATExperimentTrackingButtonClicked)
        self.IGTDisplayCheckBox.clicked['bool'].connect(self.IGTDisplayCheckBoxClicked)

# Checked
    def initLogger(self):
        self.logger = logging.getLogger('Hololens')
        # disable sys.stdout
        self.logger.propagate = False

        self.logTextBox = QPlainTextEditLogger(self)
        simple_format = "%(asctime)s [%(levelname)s] %(filename)s [line:%(lineno)d] %(message)s"
        datefmt = "%Y-%m-%d %H:%M:%S"

        self.logger.handlers = []

        self.logTextBox.setFormatter(logging.Formatter(fmt=simple_format, datefmt=datefmt))
        self.logger.addHandler(self.logTextBox)
        self.logger.setLevel(logging.DEBUG)
        ## 
        self.verticalLayout_6.addWidget(self.logTextBox.widget)
        streamHandler = logging.StreamHandler()
        streamHandler.setLevel(logging.DEBUG)
        streamHandler.setFormatter(logging.Formatter(fmt=simple_format, datefmt=datefmt))
        self.logger.addHandler(streamHandler)

# Checked
    def IR_hololens_connect(self):
        self.Sensor_AHAT=Sensor_Network("192.168.1.29",3001,SensorType.AHAT_CAMERA)
        self.Sensor_LF=Sensor_Network("192.168.1.29",3002,SensorType.LEFT_FRONT)
        self.Sensor_RF=Sensor_Network("192.168.1.29",3003,SensorType.RIGHT_FRONT)
        self.Sensor_LL=Sensor_Network("192.168.1.29",3004,SensorType.LEFT_LEFT)
        self.Sensor_RR=Sensor_Network("192.168.1.29",3005,SensorType.RIGHT_RIGHT)
        self.logger.info("waiting for connection")
        self.Sensor_AHAT.connect()
        self.Sensor_LF.connect()
        self.Sensor_RF.connect()
        self.Sensor_LL.connect()
        self.Sensor_RR.connect()
        self.logger.info("connect successfully")
        
# Checked
    def Down_limit_change(self,val):
        self.Down_limit_value.setText(str(val))
        self.down_limit_ab=val
        self.Set_para()
    
# Checked
    def Up_limit_change(self,val):
        self.Up_limit_value.setText(str(val))
        self.up_limit_ab=val
        self.Set_para()

# Checked
    def Set_para(self):
        if self.AHAT_isworking:
            self.AHAT_trans_Thread.setbound(self.down_limit_ab,self.up_limit_ab)

# Checked
    def Refresh_AHAT_display(self,imgs):
        self.Depth_image.setPixmap(imgs[1])
        self.Ab_image.setPixmap(imgs[3])
        
# Checked
    def Refersh_LF_display(self,imgs):
        self.LFImage.setPixmap(imgs[1])
        
# Checked
    def Refersh_LL_display(self,imgs):
        self.LLImage.setPixmap(imgs[1])
        
# Checked
    def Refersh_RF_display(self,imgs):
        self.RFImage.setPixmap(imgs[1])
        
# Checked
    def Refersh_RR_display(self,imgs):
        self.RRImage.setPixmap(imgs[1])
          
# Checked
    def AHAT_Display(self,stat):
        if stat > 0:
            self.AHAT_isworking=True
            self.AHAT_fet_Thread=AHAT_DataFetchingThread(self.logger,self.Sensor_AHAT,self.RawAHATQueue)
            self.AHAT_trans_Thread=AHAT_DataTransferThread(self.logger,self.down_limit_ab,self.up_limit_ab,self.RawAHATQueue,self.AHATQueue,self.AHATForCalFrame)
            self.AHAT_dis_Thread=AHAT_ImageDisplayThread(self.logger,self.AHATQueue,self.AHATDisplayFrame)
            self.AHAT_dis_Thread.Singal_img.connect(self.Refresh_AHAT_display)
            self.AHAT_dis_Thread.Signal_fps.connect(self.AHATfps)
            self.AHAT_fet_Thread.start()
            self.AHAT_trans_Thread.start()
            self.AHAT_dis_Thread.start()
        else:
            self.AHAT_isworking=False
            self.AHAT_dis_Thread.stop()
            self.AHAT_trans_Thread.stop()
            self.AHAT_fet_Thread.stop()
            self.AHAT_dis_Thread=None
            self.AHAT_trans_Thread=None
            self.AHAT_fet_Thread=None
            self.RawAHATQueue=queue.Queue(maxsize=self._mx)
            self.AHATQueue=queue.Queue(maxsize=self._mx)
        
# Checked
    def LF_Display(self,stat):
        if stat>0:
            self.LF_isworking=True
            self.LF_fet_Thread=VLC_DataFetchingThread(self.logger,self.Sensor_LF,self.LFRawQueue)
            self.LF_trans_Thread=VLC_DataTransferThread(self.logger,self.LFRawQueue,self.LFImQueue,-1)
            self.LF_Display_Thread=VLC_ImageDisplayThread(self.logger,self.LFImQueue,self.LFDisplayFrame)
            self.LF_Display_Thread.Singal_img.connect(self.Refersh_LF_display)
            self.LF_Display_Thread.Signal_fps.connect(self.LFfps)
            self.LF_fet_Thread.start()
            self.LF_trans_Thread.start()
            self.LF_Display_Thread.start()
        else:
            self.LF_isworking=False
            self.LF_Display_Thread.stop()
            self.LF_trans_Thread.stop()
            self.LF_fet_Thread.stop()
            self.LF_Display_Thread=None
            self.LF_trans_Thread=None
            self.LF_fet_Thread=None
            self.LFRawQueue=queue.Queue(maxsize=self._mx_vlc)
            self.LFImQueue=queue.Queue(maxsize=self._mx_vlc)
        
# Checked
    def LL_Display(self,stat):
        if stat>0:
            self.LL_isworking=True
            self.LL_fet_Thread=VLC_DataFetchingThread(self.logger,self.Sensor_LL,self.LLRawQueue)
            self.LL_trans_Thread=VLC_DataTransferThread(self.logger,self.LLRawQueue,self.LLImQueue,1)
            self.LL_Display_Thread=VLC_ImageDisplayThread(self.logger,self.LLImQueue,self.LLDisplayFrame)
            self.LL_Display_Thread.Singal_img.connect(self.Refersh_LL_display)
            self.LL_Display_Thread.Signal_fps.connect(self.LLfps)
            self.LL_fet_Thread.start()
            self.LL_trans_Thread.start()
            self.LL_Display_Thread.start()
        else:
            self.LL_isworking=False
            self.LL_Display_Thread.stop()
            self.LL_trans_Thread.stop()
            self.LL_fet_Thread.stop()
            self.LL_Display_Thread=None
            self.LL_trans_Thread=None
            self.LL_fet_Thread=None
            self.LLRawQueue=queue.Queue(maxsize=self._mx_vlc)
            self.LLImQueue=queue.Queue(maxsize=self._mx_vlc)
        
# Checked
    def RF_Display(self,stat):
        if stat>0:
            self.RF_isworking=True
            self.RF_fet_Thread=VLC_DataFetchingThread(self.logger,self.Sensor_RF,self.RFRawQueue)
            self.RF_trans_Thread=VLC_DataTransferThread(self.logger,self.RFRawQueue,self.RFImQueue,1)
            self.RF_Display_Thread=VLC_ImageDisplayThread(self.logger,self.RFImQueue,self.RFDisplayFrame)
            self.RF_Display_Thread.Singal_img.connect(self.Refersh_RF_display)
            self.RF_Display_Thread.Signal_fps.connect(self.RFfps)
            self.RF_fet_Thread.start()
            self.RF_trans_Thread.start()
            self.RF_Display_Thread.start()
        else:
            self.RF_isworking=False
            self.RF_Display_Thread.stop()
            self.RF_trans_Thread.stop()
            self.RF_fet_Thread.stop()
            self.RF_Display_Thread=None
            self.RF_trans_Thread=None
            self.RF_fet_Thread=None
            self.RFRawQueue=queue.Queue(maxsize=self._mx_vlc)
            self.RFImQueue=queue.Queue(maxsize=self._mx_vlc)
        
# Checked
    def RR_Display(self,stat):
        if stat>0:
            self.RR_isworking=True
            self.RR_fet_Thread=VLC_DataFetchingThread(self.logger,self.Sensor_RR,self.RRRawQueue)
            self.RR_trans_Thread=VLC_DataTransferThread(self.logger,self.RRRawQueue,self.RRImQueue,-1)
            self.RR_Display_Thread=VLC_ImageDisplayThread(self.logger,self.RRImQueue,self.RRDisplayFrame)
            self.RR_Display_Thread.Singal_img.connect(self.Refersh_RR_display)
            self.RR_Display_Thread.Signal_fps.connect(self.RRfps)
            self.RR_fet_Thread.start()
            self.RR_trans_Thread.start()
            self.RR_Display_Thread.start()
        else:
            self.RR_isworking=False
            self.RR_Display_Thread.stop()
            self.RR_trans_Thread.stop()
            self.RR_fet_Thread.stop()
            self.RR_Display_Thread=None
            self.RR_trans_Thread=None
            self.RR_fet_Thread=None
            self.RRRawQueue=queue.Queue(maxsize=self._mx_vlc)
            self.RRImQueue=queue.Queue(maxsize=self._mx_vlc)
    
# Checked
    def RRfps(self,_fps):
        self.fps_RR.setText(str(_fps)+" fps")
    
# Checked
    def LLfps(self,_fps):
        self.fps_LL.setText(str(_fps)+" fps")
        
# Checked
    def RFfps(self,_fps):
        self.fps_RF.setText(str(_fps)+" fps")
        
# Checked
    def LFfps(self,_fps):
        self.fps_LF.setText(str(_fps)+" fps")

# Checked
    def AHATfps(self,_fps):
        self.fps_AHAT.setText(str(_fps)+" fps")
    
# Checked
    def KalmanFIlterCheckBoxClicked(self,_Status):
        if _Status:
            # Start Kalman Filter Here
            self.KalmanFilterStatus=True
            self.logger.info("Tool Tracking Kalman FIlter On")
        
        else:
            # Stop Kalman Filter Here
            self.KalmanFilterStatus=False
            self.logger.info("Tool Tracking Kalman Filter Off")

# Checked
    def IGTDisplayCheckBoxClicked(self,_Status):
        if _Status:
            # Start Kalman Filter Here
            self.IGTDisplayStatus=True
            self.logger.info("Tool Display Slicer On")
        
        else:
            # Stop Kalman Filter Here
            self.IGTDisplayStatus=False
            self.logger.info("Tool Display Slicer Off")

# Checked
    def CollectSensorTestData(self):
        if not self._isRecordingAHATDATA:
            self._isRecordingAHATDATA=True
            self.CollectStartButton.setText("Stop")
            FrameNum=self.CollectFrameSpinBox.value()
            filename=self.RootPath+self.VideoNameLineEdit.text()+".npy"
            self.CollectAHATDataThread=AHATDataCollectThread(self.logger,self.AHATDisplayFrame,FrameNum,filename)
            self.CollectAHATDataThread.Signal_collectPercentag.connect(self.AlterCollectProgressBar)
            self.CollectAHATDataThread.start()
        else:
            self.CollectAHATDataThread.stop()
            self.CollectAHATDataThread=None
            self._isRecordingAHATDATA=False
            self.CollectStartButton.setText("Collect")

# Checked
    def AlterCollectProgressBar(self,num):
        self.CollectProgressorbar.setValue(num)

# Checked
    def TestSensorDataPerfusion(self):
        if not self._isProvidingAHATDATA:
            self._isProvidingAHATDATA=True
            self.ProvideStartButton.setText("Stop")
            FilePath=self.RootPath+self.VideoNameLineEdit.text()+".npy"
            self.ProvideAHATDataThread=AHATDataPerfusionThread(self.logger,self.RawAHATQueue,FilePath)
            self.ProvideAHATDataThread.SignalPercentage.connect(self.AlterCollectProgressBar)
            self.AHAT_trans_Thread=AHAT_DataTransferThread(self.logger,self.down_limit_ab,self.up_limit_ab,self.RawAHATQueue,self.AHATQueue,self.AHATForCalFrame)
            self.AHAT_dis_Thread=AHAT_ImageDisplayThread(self.logger,self.AHATQueue,self.AHATDisplayFrame)
            self.AHAT_dis_Thread.Singal_img.connect(self.Refresh_AHAT_display)
            self.AHAT_dis_Thread.Signal_fps.connect(self.AHATfps)
            self.ProvideAHATDataThread.start()
            self.AHAT_trans_Thread.start()
            self.AHAT_dis_Thread.start()
        else:
            self._isProvidingAHATDATA=False
            self.ProvideStartButton.setText("Provide")
            self.AHAT_dis_Thread.stop()
            self.AHAT_trans_Thread.stop()
            self.ProvideAHATDataThread.stop()
            self.AHAT_dis_Thread=None
            self.AHAT_trans_Thread=None
            self.ProvideAHATDataThread=None
            self.RawAHATQueue=queue.Queue(maxsize=self._mx)
            self.AHATQueue=queue.Queue(maxsize=self._mx)      

# Checked
    def OnToolConstructionButtonClicked(self):
        def ToolDefLossFunction(targetpose,args):
            AllData=args
            Loss=np.mean(np.sqrt(np.sum((AllData-targetpose)**2,1)))
            return Loss
        FileName="Cache/"+self.ToolDefFileNameEditLine.text()+".npy"
        
        def ToolDefLossFunction(targetpose,args):
            AllData=args
            Loss=np.mean(np.sqrt(np.sum((AllData-targetpose)**2,1)))
            return Loss

        _upperlim=256*15
        _downlim=256*1.5
        _minSize=10
        _maxSize=300
        _distanceTolerance=12
        _preretval=0
        _prelabel=[]
        _prestats=[]
        _precentroids=[]
        _prelabelcorr=[]
        _intrin=loadmat('AHAT_Para_Python.mat')
        mtx=_intrin['Mtx']
        distcoef=_intrin['dist']
        AllAHATData=np.load(FileName,allow_pickle=True)
        Data_Length=len(AllAHATData)
        xyzs=[]
        for num_fr in range(Data_Length):
            Curr_fr=AllAHATData[num_fr]
            _Frame=Curr_fr.MatReflectivity
            _DepthFrame=Curr_fr.MatDepth
            # _im=_Frame
            _im=copy.deepcopy(_Frame)
            _im[_im>=_upperlim]=_upperlim
            _im[_im<=_downlim]=_downlim
            _im=(_im-_downlim)*(256*256/_upperlim)/(_upperlim-_downlim)*255
            _mask=np.array(_im/256,dtype='uint8')
            _im[_mask==0]=0
            _displayTag=np.zeros([512,512])
            retval,labels,stats,centroids,label_corr=DetectBalls(_mask,_minSize,_maxSize,_distanceTolerance,_preretval,[],[],_precentroids,_prelabelcorr)
            _preretval=retval
            _precentroids=centroids
            _prelabelcorr=label_corr
            _temp_xy = cv2.undistortPoints(centroids,mtx,distcoef)
            _xyd=[]
            _xyz=[]
            for j in range(centroids.shape[0]):
                _u=centroids[j,0]
                _v=centroids[j,1]
                _d=_DepthFrame[int(_v),int(_u)]
                _xyd.append([_temp_xy[j][0][0],_temp_xy[j][0][1],_d])
                _ori_xyz=[_temp_xy[j][0][0],_temp_xy[j][0][1],1]
                _l=np.sqrt(sum(np.array(_ori_xyz)**2))
                _real_xyz=_ori_xyz/_l*_d
                _lreal=np.sqrt(sum(np.array(_real_xyz)**2))
                _xyz.append(list(_real_xyz/_lreal*(_lreal+5.75)))
            xyzs.append(_xyz)
        DataDefCollected=np.array(xyzs)
        BaseFrame=DataDefCollected[0]
        NumPoints=BaseFrame.shape[0]
        assert NumPoints>=4 
        DetectionTolerance=5
        EffectivePoints=[]
        for i in range(DataDefCollected.shape[0]):
            # 
            DataFrame=DataDefCollected[i]
            FrameDataWithRightSequence=[]
            for j in range(NumPoints):
                Dist=np.sum((DataFrame-BaseFrame[j])**2,1)
                if min(Dist)>=DetectionTolerance:
                    continue
                FrameDataWithRightSequence.append(DataFrame[np.where(Dist==min(Dist))[0][0]])
            if not len(FrameDataWithRightSequence)==NumPoints:
                continue
            # reshape n*3->3n
            EffectivePoints.append(np.reshape(FrameDataWithRightSequence,[3*NumPoints]))
        ToolDefFrames=np.array(EffectivePoints)
        EffectiveFrameNum=ToolDefFrames.shape[0]

        MeanToolDefintion=np.mean(ToolDefFrames,0)
        Res=optimize.minimize(ToolDefLossFunction,MeanToolDefintion,args=(ToolDefFrames,))
        FinalLoss=Res['fun']/np.sqrt(NumPoints)
        ToolShape=np.reshape(Res['x'],[NumPoints,3])
        ToolShape=ToolShape-np.mean(ToolShape,0)
        
        ToolName=self.ToolDefNameEditLine.text()
        Pathtosave=os.path.join(self.ToolListRootDir,(ToolName+".mat"))
        savemat(Pathtosave,{"ToolName":ToolName,"ToolShape":ToolShape,"Loss":FinalLoss})
        self.logger.info("\n \n"+"Construct a Tool\n"+
                        "Marker Num : "+str(NumPoints)+"\n"+"Tool Shape\n"+str(ToolShape)+"\n"+
                        "Definition Error : " +str(FinalLoss)+"  mm (RMSE)\n"+
                        "Save to : "+Pathtosave+"\n")

# Checked
    def OnAHATExperimentTrackingButtonClicked(self):
        if not self.Experiment_isTrackingObject:
            self.Experiment_isTrackingObject=True
            ToolFolder=self.ToolListRootDir
            self.AHATTrackIRToolFromFolder(ToolFolder,self.AHATExperimentCalibrationTrackingDataReceivedCallback,1)
            self.AHATExperimentTrackingTimer=QtCore.QTimer()
            self.AHATExperimentTrackingTimer.timeout.connect(self.OnAHATExperimentTrackingTimerTimeOut)
            self.AHATExperimentTrackingTimer.start(30)
        else:
            self.Experiment_isTrackingObject=False
            self.AHATExperimentTrackingTimer.timeout.disconnect(self.OnAHATExperimentTrackingTimerTimeOut)
            self.AHATExperimentTrackingTimer=None
            ToolFolder=self.ToolListRootDir
            self.AHATTrackIRToolFromFolder(ToolFolder,self.AHATExperimentCalibrationTrackingDataReceivedCallback,0)
    
# Checked
    def OnAHATExperimentTrackingTimerTimeOut(self):
        # self.logger.info("Tracking Result : "+"\n"+str(self.TrackingMartrixs))
        if self.IGTDisplayStatus:
            if not self.IGTClient:
                self.IGTClient=pyigtl.OpenIGTLinkClient("127.0.0.1", 18944)
                self.logger.info("IGT established")
            for tool_num in range(len(self.TrackingMartrixs)):
                TransformMatrix=np.array(self.TrackingMartrixs[tool_num])
                mess=pyigtl.TransformMessage(TransformMatrix,device_name="Tool_"+str(tool_num))
                self.IGTClient.send_message(mess)
        else:
            self.logger.info("Tracking Result : "+"\n"+str(self.TrackingMartrixs))

# Checked
    def AHATExperimentCalibrationTrackingDataReceivedCallback(self,info):
        if self.KalmanFilterStatus:
            self.TrackingMartrixs=info[0].TransformatrixFiltered
        else:
            self.TrackingMartrixs=info[0].Transformatrix

# Checked
    def AHATTrackIRToolFromFolder(self,FolderName,_callback,stat):
        ToolLoader=self.LoadToolFromFolder(FolderName)
        if not ToolLoader[0]:
            self.logger.error("Error Exists when loading tools")
            return
        self.ToolInfo=ToolLoader
        self.ToolNames=ToolLoader[1]
        if stat > 0 :
            self._isTrackingTool=True
            # self.tracking_ir_thread=AHATIRToolTracking(self.logger,self.AHATDisplayFrame)
            self.tracking_ir_thread=AHATIRToolTracking(self.logger,self.AHATForCalFrame,self.ToolInfo[2],self.ToolInfo[1])
            self.tracking_ir_thread.SignalTool.connect(_callback)
            self.tracking_ir_thread.SignalFPS.connect(self.EmptyTask)
            self.tracking_ir_thread.start()
        else:
            self._isTrackingTool=False
            self.tracking_ir_thread.stop()
            self.tracking_ir_thread=None

# Checked
    def EmptyTask(self,data):
        pass

# Checked
    def LoadToolFromFolder(self,FolderName):
        ToolListRootDir=FolderName
        AvailableFiles=os.listdir(ToolListRootDir)
        if not "config.json" in AvailableFiles:
            self.logger.error("Config File Not Found")
            return (False,)
        ConfigPath=os.path.join(ToolListRootDir,"config.json")
        with open(ConfigPath,'r') as f:
            ToolNamesList=json.load(f)
        ToolNames=[]
        ToolShapes=[]
        for i in range(len(ToolNamesList)):
            ToolFileName=ToolNamesList[i]+".mat"
            if not ToolFileName in AvailableFiles:
                self.logger.error("Tool Cannot Find : "+ToolFileName)
                return (False,)
                # return 
            FullName=os.path.join(ToolListRootDir,ToolFileName)
            Tool=loadmat(FullName)

            ToolNames.append(Tool['ToolName'][0])
            ToolShapes.append(NDI_Tool(Tool['ToolShape']))
        return (True,ToolNames,ToolShapes)

    


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    main_dialog = MainDialog()
    main_dialog.show()
    sys.exit(app.exec_())