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
    def __init__(self, sphere_radius):
        ## Variant for sensor datas
        self.down_limit_ab=128
        self.up_limit_ab=512
        self._mx=5
        self.sphere_radius = sphere_radius

        self.port_pub = "12389"
        self.depth_offset = 0 # unit: mm

        self.AHATForCalFrame=queue.Queue(maxsize=self._mx)
        self.AHAT_isworking=False

        self.ToolFolder = Path(os.path.dirname(os.path.realpath(__file__))) / 'ToolList'

        ToolLoader=self.LoadToolFromFolder(str(self.ToolFolder))
        if not ToolLoader[0]:
            self.logger.error("Error Exists when loading tools")
            return
        self.ToolInfo=ToolLoader
        self.ToolNames=ToolLoader[1]


    def track_tool(self):
        self._isTrackingTool=True
        
        self.tracking_ir_thread=AHATIRToolTracking(self.AHATForCalFrame,
                                                self.ToolInfo[2],
                                                self.ToolInfo[1], 
                                                self.port_pub,
                                                self.sphere_radius,
                                                self.depth_offset)
        self.tracking_ir_thread.start()

    
    def add_frame(self, centers, pose, timestamp):
        if not self.AHATForCalFrame.full():
            Frame_this=AHATFrame(centers, timestamp, pose)
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
    

    def save_next_frame(self):
        self.save_frame = True

    def update_lut(self, lut):
        self.lut = lut
        self.tracking_ir_thread.update_lut(lut)