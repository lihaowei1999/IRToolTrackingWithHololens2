from pathlib import Path
import numpy as np
import cv2
import os
import json
from scipy.io import loadmat
from .AHATNDITracker import *
from .ThreadFactory import *
import queue

class IRToolTrack:
    def __init__(self):
        ## Variant for sensor datas
        self.down_limit_ab=128
        self.up_limit_ab=512
        self._mx=5

        self.AHATForCalFrame=queue.Queue(maxsize=self._mx)
        self.AHAT_isworking=False

        self.ToolFolder = Path(os.path.dirname(os.path.realpath(__file__))) / 'ToolList'

        ToolLoader=self.LoadToolFromFolder(str(self.ToolFolder))
        if not ToolLoader[0]:
            self.logger.error("Error Exists when loading tools")
            return
        self.ToolInfo=ToolLoader
        self.ToolNames=ToolLoader[1]


    def track_tool(self, lut):
        self._isTrackingTool=True
        
        self.tracking_ir_thread=AHATIRToolTracking(self.AHATForCalFrame,self.ToolInfo[2],self.ToolInfo[1], lut, "12349")
        self.tracking_ir_thread.start()

    
    def add_frame(self, frame_depth, frame_ab, pose, timestamp):
        if not self.AHATForCalFrame.full():
            Ab_low=self.down_limit_ab
            Ab_high=self.up_limit_ab

            _,imgdepth=cv2.threshold(frame_depth,1000,0,cv2.THRESH_TRUNC)
            imgdepth_processed=np.uint8(imgdepth/4)
            imgab_processed=copy.deepcopy(frame_ab)
            imgab_processed[imgab_processed<Ab_low]=Ab_low
            imgab_processed[imgab_processed>Ab_high]=Ab_high
            imgab_processed=np.uint8((imgab_processed-Ab_low)/(Ab_high-Ab_low)*255)
            Frame_this=AHATFrame(imgdepth_processed,imgab_processed,frame_depth,frame_ab, timestamp, pose)
            
            self.AHATForCalFrame.put(Frame_this)
        else:
            print(".", end='', flush=True)
        

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