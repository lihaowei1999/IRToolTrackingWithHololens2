#from _typeshed import NoneType
import logging
import sys
import os
from tkinter.messagebox import NO
from tokenize import Single
from types import FrameType
from typing import Final
from numpy.core.defchararray import lower
from numpy.core.numeric import NaN
# import qwt
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
from FunctionFactory import *
from XMLDataLoader import *
import pyigtl
from scipy.io import savemat,loadmat
from scipy import optimize
import json
from sksurgerynditracker.nditracker import NDITracker


class MainDialog(QtWidgets.QMainWindow, Ui_MainWindow):
    
    def __init__(self):
        super(MainDialog, self).__init__()
        self.setupUi(self)
        
        self.initLogger()
        
        self.simple_test=1
        self.version='run'
        self.Hololens_pos=[]
        self.tracker_pos=[]
        
        self._is_tracking = False

        self.setWindowFlag(QtCore.Qt.WindowMinMaxButtonsHint)
        self.logger.info(f'System started.')
        
        self.down_limit_ab=128
        self.up_limit_ab=512
        
        self._is_getting_image=False
        self._is_transforming_Data=False
        self._is_displaying=False
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
        
        self._workingDirect="/home/lihaowei/Documents/Git/HololensWorkingUI/UI/SensorData/Cache"
        self._DirectMutex=QMutex()
        
        self.AHATDisplayFrame=queue.Queue(maxsize=1)
        self.RRDisplayFrame=queue.Queue(maxsize=1)
        
        
        self.RFDisplayFrame=queue.Queue(maxsize=1)
        self.LLDisplayFrame=queue.Queue(maxsize=1)
        self.LFDisplayFrame=queue.Queue(maxsize=1)
        self.SnapShotFrame=queue.Queue(maxsize=1)
        
        self._isTakingVideo=False

        self.CumulativeFramePhoto=[]
        
        self._isTrackingTool=False
        
        self._isDisplaying3Dinformation=False
        
        self._isDisplaySnapShotInfomation=False
        
        self._CalibrationDataStorage=[]
        
        self._CalibrationNDIData=[]
        
        self.igtDisplayStatus=False
        self.igtlClient=None

        self.CollectAHATDataThread=None
        self.ProvideAHATDataThread=None
        self._isRecordingAHATDATA=False
        self._isProvidingAHATDATA=False


        self.RootPath="Cache/"
        self.defaultRecordingPath="Cache/AHATRecordData.npy"
        self.defaultRecordingTrackingData="Cache/AHATTrackingRecordData.npy"


        self._iscollectingTrackingData=False
        self._collectedTrakingData=[]


        self._iscollectingToolDefData=False
        self.ToolDefTotalFrame=0
        self.ToolDefCurrentFrame=0
        self.DefaultToolDefSavePath="Cache/ToolDefFile.npy"
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
        self.Init_UI_Setup()

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

        self.LF2AHAT=loadmat("LF2AHAT.mat")["T"]

        #self.Img2Tracker=np.load("Model_1_Img_2_Tracker.npy")

        self.Experiment_isCalibrating=False
        self.Experiment_isTrackingObject=False
        self.ExperimentEndTransform=np.diag([1.0,1.0,1.0,1.0])
        self.ControlId=0

    def Init_Signal_and_Slot(self):
        self.HololenUnityDisplayConnection.clicked.connect(self.OnHololensUnityDisplayConnectionButtonClicked)
        #self.DisplayTestButton.clicked.connect(self.OnDisplayTestButtonClicked)
        self.RegisterXSlider.valueChanged.connect(self.OnRegisterXSliderValueChanged)
        self.RegisterYSlider.valueChanged.connect(self.OnRegisterYSliderValueChanged)
        self.RegisterZSlider.valueChanged.connect(self.OnRegisterZSliderValueChanged)
        self.RegisterXangleSlider.valueChanged.connect(self.OnRegisterXangleSliderValueChanged)
        self.RegisterYangleSlider.valueChanged.connect(self.OnRegisterYangleSliderValueChanged)
        self.RegisterZangleSlider.valueChanged.connect(self.OnRegisterZangleSliderValueChanged)
        self.RegisterResultSavingButton.clicked.connect(self.OnRegisterResultSavingButtonClicked)

        self.UnityDelaySettingSlider.valueChanged.connect(self.OnUnityDelaySettingSlidervalueChanged)
        self.UnityDelaySettingButton.clicked.connect(self.OnUnityDelaySettingButtonClicked)
        self.unityslerpsettingbutton.clicked.connect(self.OnunityslerpsettingbuttonClicked)
        self.unityslerpsettingslider.valueChanged.connect(self.OnunityslerpsettingsliderValueChanged)
        self.unityintersettingbutton.clicked.connect(self.OnunityintersettingbuttonClicked)
        self.unityintersettingslider.valueChanged.connect(self.OnunityintersettingsliderValueChanged)

        self.AHATExperimentCalibrationButton.clicked.connect(self.OnAHATExperimentCalibrationButtonClicked)
        self.AHATExperimentTrackingButton.clicked.connect(self.OnAHATExperimentTrackingButtonClicked)
        self.AHATExperimentClearViewButton.clicked.connect(self.OnAHATExperimentClearViewButtonClicked)


    def Init_UI_Setup(self):
        RegisterData=loadmat("scripts/ViewControl/RegisterData.mat")
        self.RegisterXSlider.setValue(int(RegisterData["x"][0][0]*10))
        self.RegisterYSlider.setValue(int(RegisterData["y"][0][0]*10))
        self.RegisterZSlider.setValue(int(RegisterData["z"][0][0]*10))
        self.RegisterXangleSlider.setValue(int(RegisterData["ax"][0][0]*500))
        self.RegisterYangleSlider.setValue(int(RegisterData["ay"][0][0]*500))
        self.RegisterZangleSlider.setValue(int(RegisterData["az"][0][0]*500))

        DelayData=loadmat("scripts/ViewControl/DelaySetup.mat")
        self.UnityDelaySettingSlider.setValue(DelayData["delay"][0][0])
        SlerpData=loadmat("scripts/ViewControl/SlerpPercentSetup.mat")
        self.unityslerpsettingslider.setValue(int(SlerpData["slerp"][0][0]*100))
        InterData=loadmat("scripts/ViewControl/InterplotPercentSetup.mat")
        self.unityintersettingslider.setValue(int(InterData["inter"][0][0]*100))

        self.AHATExperimentTrackingObjectSelectionBox.addItems(["Model_1","Model_2","Model_3","Drill"])



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
        
    def Down_limit_change(self,val):
        self.Down_limit_value.setText(str(val))
        self.down_limit_ab=val
        self.Set_para()
    
    def Up_limit_change(self,val):
        self.Up_limit_value.setText(str(val))
        self.up_limit_ab=val
        self.Set_para()

    def Set_para(self):
        if self.AHAT_isworking:
            self.AHAT_trans_Thread.setbound(self.down_limit_ab,self.up_limit_ab)

    def Refresh_AHAT_display(self,imgs):
        self.Depth_image.setPixmap(imgs[1])
        self.Ab_image.setPixmap(imgs[3])
        
    def Refersh_LF_display(self,imgs):
        self.LFImage.setPixmap(imgs[1])
        
    def Refersh_LL_display(self,imgs):
        self.LLImage.setPixmap(imgs[1])
        
    def Refersh_RF_display(self,imgs):
        self.RFImage.setPixmap(imgs[1])
        
    def Refersh_RR_display(self,imgs):
        self.RRImage.setPixmap(imgs[1])
        
            
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
    
    def RRfps(self,_fps):
        self.fps_RR.setText(str(_fps)+" fps")
    
    def LLfps(self,_fps):
        self.fps_LL.setText(str(_fps)+" fps")
        
    def RFfps(self,_fps):
        self.fps_RF.setText(str(_fps)+" fps")
        
    def LFfps(self,_fps):
        self.fps_LF.setText(str(_fps)+" fps")
        
    def AHATfps(self,_fps):
        self.fps_AHAT.setText(str(_fps)+" fps")

    def LoadToolFromFile(self):
        ToolListRootDir=self.ToolListRootDir
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

        # self.DrawQWTPlot()

    def ChangeFPSTracking(self,_fps:int):
        self.TrackingFrameRateLabel.setText(str(_fps)+"fps")
    
    def NDI_Connection(self):
        self.logger.info("Start Connection to NDI")
        self.tracker=NDI_tracker(["/home/lihaowei/Documents/GitHub/Hololens_Qt/trial_1/tracker/boardtracker.rom"])
        self.tracker.machine_start()
        self.logger.info("NDI Connected")
        
        
    # require a list input
    def SnapShotProviding(self):        
        if not self._isDisplaySnapShotInfomation:
            self._isDisplaySnapShotInfomation=True
            self.StartSnapshotButton.setText("Stop")
            self.SnapShotDisplayThread=AHATCalibrationInput(self.logger,self.AHATDisplayFrame)
            self.SnapShotDisplayThread.SignalCalibrationInput.connect(self.DealSnapShotInfomation)
            self.SnapShotDisplayThread.start()
        else:
            self._isDisplaySnapShotInfomation=False
            self.SnapShotDisplayThread.stop()
            self._isDisplaySnapShotInfomation=None
            self.StartSnapshotButton.setText("Start SnapShot")
            
        
        
    def DealSnapShotInfomation(self,_info):
        self.setSnapShotLabel(_info[0])
        if not self.SnapShotFrame.empty():
            _=self.SnapShotFrame.get()
        self.SnapShotFrame.put(_info)
    
    def TakeCalibrationData(self):
        if not self.SnapShotFrame.empty():
            _cur=self.SnapShotFrame.get()
        self._CalibrationDataStorage.append(_cur)
        _curndi=self.tracker.fetch()
        print(_curndi)
        self._CalibrationNDIData.append(_curndi)
        print(len(self._CalibrationDataStorage))

        

    def KalmanFIlterCheckBoxClicked(self,_Status):
        if _Status:
            # Start Kalman Filter Here
            self.KalmanFilterStatus=True
            self.logger.info("Tool Tracking Kalman FIlter On")
        
        else:
            # Stop Kalman Filter Here
            self.KalmanFilterStatus=False
            self.logger.info("Tool Tracking Kalman Filter Off")

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

    def AlterCollectProgressBar(self,num):
        self.CollectProgressorbar.setValue(num)

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


    def IGTDisplayChange(self,status):
        self.igtDisplayStatus=status


    def OnToolPreviewButtonClicked(self):
        if not self._isDisplaySnapShotInfomation:
            self._isDisplaySnapShotInfomation=True
            self.ToolDefPreviewButton.setText("Stop")
            self.SnapShotDisplayThread=AHATCalibrationInput(self.logger,self.AHATDisplayFrame)
            self.SnapShotDisplayThread.SignalCalibrationInput.connect(self.OnCollectToolDefSingleFrame)
            self.SnapShotDisplayThread.start()
        else:
            self._isDisplaySnapShotInfomation=False
            self.SnapShotDisplayThread.stop()
            self._isDisplaySnapShotInfomation=None
            self.ToolDefPreviewButton.setText("Preview")
            

    def OnCollectToolDefSingleFrame(self,_info):
        if not self.ToolDefPreviewFrame.empty():
            _=self.ToolDefPreviewFrame.get()
        self.ToolDefPreviewFrame.put(_info[0])


    def OnToolDefCollectionButtonClicked(self):
        print(self.ToolDefFileNameEditLine.text())
        if not self._iscollectingToolDefData:
            self._iscollectingToolDefData=True
            self.ToolDefTotalFrame=self.ToolDefFrameSpinBox.value()
            self.ToolDefCollectButton.setText("Stop")
            ToolFileName="Cache/"+self.ToolDefFileNameEditLine.text()+".npy"
            ToolDefFrameNum=self.ToolDefFrameSpinBox.value()
            self.ToolDefDataCollectingThread=ToolDefDataCollection(self.logger,self.ToolDefPreviewFrame,ToolDefFrameNum,ToolFileName)
            self.ToolDefDataCollectingThread.SignalCollectionPercentage.connect(self.OnToolDefProgressBarChange)
            self.ToolDefDataCollectingThread.start()

        else:
            self._iscollectingToolDefData=False
            self.ToolDefCollectButton.setText("Collect")
            self.ToolDefDataCollectingThread.stop()
            self.ToolDefDataCollectingThread=None


    def OnToolDefProgressBarChange(self,val):
        self.ToolDefprogressBar.setValue(val[0])
        

    def OnToolConstructionButtonClicked(self):
        def ToolDefLossFunction(targetpose,args):
            AllData=args
            Loss=np.mean(np.sqrt(np.sum((AllData-targetpose)**2,1)))
            return Loss
        FileName="Cache/"+self.ToolDefFileNameEditLine.text()+".npy"
        DataDefCollected=np.load(FileName)
        BaseFrame=DataDefCollected[0]
        # assert n >4 
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
        print("Effective Frame Num : "+str(EffectiveFrameNum))
        MeanToolDefintion=np.mean(ToolDefFrames,0)
        print(np.reshape(MeanToolDefintion,[4,3]))
        Res=optimize.minimize(ToolDefLossFunction,MeanToolDefintion,args=(ToolDefFrames,))
        FinalLoss=Res['fun']/np.sqrt(NumPoints)
        ToolShape=np.reshape(Res['x'],[NumPoints,3])
        ToolShape=ToolShape-np.mean(ToolShape,0)
        ToolName=self.ToolDefNameEditLine.text()
        print(ToolShape)
        print(FinalLoss)
        Pathtosave=os.path.join(self.ToolListRootDir,(ToolName+".mat"))
        savemat(Pathtosave,{"ToolName":ToolName,"ToolShape":ToolShape,"Loss":FinalLoss})


    def OnHololensUnityDisplayConnectionButtonClicked(self):
        self.HololensUnityDisplayController=LoadDeviceData("scripts/ViewControl/HololensDevice.xml")[3][0]
        self.HololensUnityDisplayController.ConnectSocketChannel()
        self.logger.info("Hololens Unity 3D View Connected")



    def OnRegisterXSliderValueChanged(self,value):
        val=value/10 # mm
        self.LabelRegisterXDisplay.setText(str(val))
        self.registerDataX=val

    def OnRegisterYSliderValueChanged(self,value):
        val=value/10 # mm
        self.LabelRegisterYDisplay.setText(str(val))
        self.registerDataY=val

    def OnRegisterZSliderValueChanged(self,value):
        val=value/10 # mm
        self.LabelRegisterZDisplay.setText(str(val))
        self.registerDataZ=val

    def OnRegisterXangleSliderValueChanged(self,value):
        val=value/500
        self.LabelRegisterXAngleDisplay.setText(str(val))
        self.registerXangleData=val
    
    def OnRegisterYangleSliderValueChanged(self,value):
        val=value/500
        self.LabelRegisterYAngleDisplay.setText(str(val))
        self.registerYangleData=val

    def OnRegisterZangleSliderValueChanged(self,value):
        val=value/500
        self.LabelRegisterZAngleDisplay.setText(str(val))
        self.registerZangleData=val

    def OnRegisterResultSavingButtonClicked(self):
        savemat("scripts/ViewControl/RegisterData.mat",{"x":self.registerDataX,"y":self.registerDataY,"z":self.registerDataZ,"ax":self.registerXangleData,"ay":self.registerYangleData,"az":self.registerZangleData})

    def OnUnityDelaySettingSlidervalueChanged(self,value):
        self.registerTime=value
        self.UnityDelayDisplayLabel.setText(str(value))

    def OnUnityDelaySettingButtonClicked(self):
        self.logger.info("Setting Unity Delay to "+str(self.registerTime)+" frames")
        self.HololensUnityDisplayController.SetUnityDelay(self.registerTime)
        savemat("scripts/ViewControl/DelaySetup.mat",{"delay":self.registerTime})

    def OnunityslerpsettingbuttonClicked(self):
        self.HololensUnityDisplayController.SetSlerpPercentage(self.slerppercent)
        savemat("scripts/ViewControl/SlerpPercentSetup.mat",{"slerp":self.slerppercent})

    def OnunityslerpsettingsliderValueChanged(self,value):
        self.slerppercent=value/100
        self.unityslerpvaluedisplaylabel.setText(str(self.slerppercent))

    def OnunityintersettingbuttonClicked(self):
        self.HololensUnityDisplayController.SetInterplotPercentage(self.interplotpercent)
        savemat("scripts/ViewControl/InterplotPercentSetup.mat",{"inter":self.interplotpercent})

    def OnunityintersettingsliderValueChanged(self,value):
        self.interplotpercent=value/100
        self.unityintervaluedisplaylabel.setText(str(self.interplotpercent))

    def OnAHATExperimentTrackingButtonClicked(self):
        ToolDefFolder=["ExperimentTool/Model_1/",
                        "ExperimentTool/Model_2/",
                        "ExperimentTool/Model_3/",
                        "ExperimentTool/Drill/"]
        TransformDefFile=["ExperimentTool/RegistrationResults/Model_1.mat",
                            "ExperimentTool/RegistrationResults/Model_2.mat",
                            "ExperimentTool/RegistrationResults/Model_3.mat",
                            "ExperimentTool/RegistrationResults/Needle.mat"]
        ControlIds=[1,2,3,5]
        if not self.Experiment_isTrackingObject:
            self.Experiment_isTrackingObject=True
            SelectNum=self.AHATExperimentTrackingObjectSelectionBox.currentIndex()
            ToolFolder=ToolDefFolder[SelectNum]
            if SelectNum==3:
                self.ExperimentEndTransform=loadmat(TransformDefFile[SelectNum])['T']
            else:
                self.ExperimentEndTransform=np.linalg.inv(loadmat(TransformDefFile[SelectNum])['T'])
            self.ControlId=ControlIds[SelectNum]
            self.AHATTrackIRToolFromFolder(ToolFolder,self.AHATExperimentCalibrationTrackingDataReceivedCallback,1)
            self.AHATExperimentTrackingTimer=QtCore.QTimer()
            self.AHATExperimentTrackingTimer.timeout.connect(self.OnAHATExperimentTrackingTimerTimeOut)
            self.AHATExperimentTrackingTimer.start(30)

        else:
            self.Experiment_isTrackingObject=False
            self.AHATExperimentTrackingTimer.timeout.disconnect(self.OnAHATExperimentTrackingTimerTimeOut)
            self.AHATExperimentTrackingTimer=None
            self.AHATTrackIRToolFromFolder(ToolFolder,self.AHATExperimentCalibrationTrackingDataReceivedCallback,0)
    


    def OnAHATExperimentTrackingTimerTimeOut(self):
        target2AHAT=self.TrackingMartrixs[0]
        theta=[self.registerXangleData*3.1415/180,self.registerYangleData*3.1415/180,self.registerZangleData*3.1415/180]
        R_x = np.array([[1,         0,                  0                   ],
                [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                ])            
        R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                        [0,                     1,      0                   ],
                        [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                        ])
                    
        R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                        [math.sin(theta[2]),    math.cos(theta[2]),     0],
                        [0,                     0,                      1]
                        ])
        R=np.diag([1.,1.,1.,1.])
        R[0:3,0:3]=R_z@R_y@R_x
        R[0][3]=self.registerDataX
        R[1][3]=self.registerDataY
        R[2][3]=self.registerDataZ
        HololensPos=self.HololensUnityDisplayController.GetHololensPosition()
        trans=np.array([HololensPos[0],-HololensPos[1],HololensPos[2]])*1000
        quat=np.array([HololensPos[3],-HololensPos[4],HololensPos[5],-HololensPos[6]])
        rota=RigidTransform(quat,trans)
        rs_holo_matrix=np.zeros((4,4))
        rs_holo_matrix[0:3,0:3]=rota.rotation
        rs_holo_matrix[0:3,3]=rota.translation
        rs_holo_matrix[3,3]=1

        TargetPos=rs_holo_matrix@R@target2AHAT@self.ExperimentEndTransform
        print(TargetPos)
        self.HololensUnityDisplayController.SetPosition(self.ControlId,TargetPos)




    def OnAHATExperimentCalibrationButtonClicked(self):
        if not self.Experiment_isCalibrating:
            self.Experiment_isCalibrating=True
            #self.AHAT_acq.setCheckState(True)
            self.AHATTrackIRToolFromFolder("ExperimentTool/Calibration/",self.AHATExperimentCalibrationTrackingDataReceivedCallback,1)
            self.AHATExperimentCalibrationTimer=QtCore.QTimer()
            self.AHATExperimentCalibrationTimer.timeout.connect(self.OnAHATExperimentCalibrationTimerTimeOut)
            self.AHATExperimentCalibrationTimer.start(30)

        else:
            self.Experiment_isCalibrating=False
            self.AHATExperimentCalibrationTimer.timeout.disconnect(self.OnAHATExperimentCalibrationTimerTimeOut)
            self.AHATExperimentCalibrationTimer=None
            self.AHATTrackIRToolFromFolder("ExperimentTool/Calibration/",self.AHATExperimentCalibrationTrackingDataReceivedCallback,0)
            #self.AHAT_acq.setCheckState(False)

    def OnAHATExperimentCalibrationTimerTimeOut(self):
        if len(self.TrackingMartrixs)>0:
            target2AHAT=self.TrackingMartrixs[0]
            theta=[self.registerXangleData*3.1415/180,self.registerYangleData*3.1415/180,self.registerZangleData*3.1415/180]
            R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])            
            R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                            [0,                     1,      0                   ],
                            [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                            ])
                        
            R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                            [math.sin(theta[2]),    math.cos(theta[2]),     0],
                            [0,                     0,                      1]
                            ])
            R=np.diag([1.,1.,1.,1.])
            R[0:3,0:3]=R_z@R_y@R_x
            R[0][3]=self.registerDataX
            R[1][3]=self.registerDataY
            R[2][3]=self.registerDataZ
            print(R)
            HololensPos=self.HololensUnityDisplayController.GetHololensPosition()
            trans=np.array([HololensPos[0],-HololensPos[1],HololensPos[2]])*1000
            quat=np.array([HololensPos[3],-HololensPos[4],HololensPos[5],-HololensPos[6]])
            rota=RigidTransform(quat,trans)
            rs_holo_matrix=np.zeros((4,4))
            rs_holo_matrix[0:3,0:3]=rota.rotation
            rs_holo_matrix[0:3,3]=rota.translation
            rs_holo_matrix[3,3]=1
            
            TargetPos=rs_holo_matrix@R@np.linalg.inv(self.LF2AHAT)@target2AHAT
            TargetPos=rs_holo_matrix@R@target2AHAT
            self.HololensUnityDisplayController.SetPosition(6,TargetPos)

        else:
            QThread.msleep(20)

    def AHATExperimentCalibrationTrackingDataReceivedCallback(self,info):
        if self.KalmanFilterStatus:
            self.TrackingMartrixs=info[0].TransformatrixFiltered
        else:
            self.TrackingMartrixs=info[0].Transformatrix

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

    def EmptyTask(self,data):
        pass
    
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

    
    def OnAHATExperimentClearViewButtonClicked(self):
        TransformMatrix=np.diag([1,1,1,1])
        TransformMatrix[0,3]=0
        TransformMatrix[1,3]=0
        TransformMatrix[2,3]=-10000
        for i in range(7):
            self.HololensUnityDisplayController.SetPosition(i,TransformMatrix)
            time.sleep(1/20)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    main_dialog = MainDialog()
    main_dialog.show()
    sys.exit(app.exec_())