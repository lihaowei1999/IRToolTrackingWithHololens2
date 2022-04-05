import logging
import sys
import os
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
import copy

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
    return base2world,hololens2tracker,err_

def SaveVisualFrames(Frames,path):
    
    _AHATDepthRaw=np.array([Frame.AHAT.MatDepth for Frame in Frames])
    _AHATReflectivityRaw=np.array([Frame.AHAT.MatReflectivity for Frame in Frames])
    _AHATDepthProcessed=np.array([Frame.AHAT.MatDepthProcessed for Frame in Frames])
    _AHATReflectivityProcessed=np.array([Frame.AHAT.MatReflectivityProcessed for Frame in Frames])
    _AHATTimestamp=np.array([Frame.AHAT.timestamp for Frame in Frames])
    print("loaded AHAT data")
    _LLRaw=np.array([Frame.LL.MatOrigin for Frame in Frames])
    _LLProcessed=np.array([Frame.LL.MatVLC for Frame in Frames])
    _LLTimestamp=np.array([Frame.LL.timestamp for Frame in Frames])
    print("loadede LL data")
    _LFRaw=np.array([Frame.LF.MatOrigin for Frame in Frames])
    _LFProcessed=np.array([Frame.LF.MatVLC for Frame in Frames])
    _LFTimestamp=np.array([Frame.LF.timestamp for Frame in Frames])
    print("loadede LF data")
    _RRRaw=np.array([Frame.RR.MatOrigin for Frame in Frames])
    _RRProcessed=np.array([Frame.RR.MatVLC for Frame in Frames])
    _RRTimestamp=np.array([Frame.RR.timestamp for Frame in Frames])
    print("loadede RR data")
    _RFRaw=np.array([Frame.RF.MatOrigin for Frame in Frames])
    _RFProcessed=np.array([Frame.RF.MatVLC for Frame in Frames])
    _RFTimestamp=np.array([Frame.RF.timestamp for Frame in Frames])
    print("loadede RF data")
    _comment=np.array([Frame.comment for Frame in Frames])
    
    _tosave={'AHATDepthRaw':_AHATDepthRaw,'AHATReflectivityRaw':_AHATReflectivityRaw,
                'AHATDepthProcessed':_AHATDepthProcessed,"AHATReflectivityProcessed":_AHATReflectivityProcessed,
                'LLRaw':_LLRaw,'LLProcessed':_LLProcessed,
                'LFRaw':_LFRaw,'LFProcessed':_LFProcessed,
                'RRRaw':_RRRaw,'RRProcessed':_RRProcessed,
                'RFRaw':_RFRaw,'RFProcessed':_RFProcessed,
                'AHATTimestamp':_AHATTimestamp,
                'LLTimestamp':_LLTimestamp,
                'LFTimestamp':_LFTimestamp,
                'RRTimestamp':_RRTimestamp,
                'RFTimestamp':_RFTimestamp,
                'Comment':_comment}
    print("ready to save")
    _tm=time.localtime(time.time())
    _tm="SensorData_{:0>4d}{:0>2d}{:0>2d}{:0>2d}{:0>2d}{:0>2d}.mat".format(_tm[0],_tm[1],_tm[2],_tm[3],_tm[4],_tm[5]) 
    _pt=os.path.join(path,_tm)
    savemat(_pt,_tosave)
    print("saved")