from xml.dom.minidom import parse
from My_classes import *

def LoadDeviceData(xmlFilePath):
    DomTree=parse(xmlFilePath)
    booklist=DomTree.documentElement
    RegisHelperFileName=''
    NDINeedleTracker=''
    NDITipShift=np.zeros((4,1))
    HololensEquipmentList=[]

    if booklist.nodeName=='item':
        Items=booklist.childNodes
        for item in Items:
            if item.nodeName=='TrackerRegistrationHelper':
                for chara in item.childNodes:
                    if chara.nodeName=='FileName':
                        RegisHelperFileName=chara.childNodes[0].data
            if item.nodeName=='NDINeedle':
                for chara in item.childNodes:
                    if chara.nodeName=='FileName':
                        NDINeedleTracker=chara.childNodes[0].data
                    if chara.nodeName=='Tipshift':              
                        NDITipShift[0][0]=float(chara.getElementsByTagName('x')[0].childNodes[0].data)
                        NDITipShift[1][0]=float(chara.getElementsByTagName('y')[0].childNodes[0].data)
                        NDITipShift[2][0]=float(chara.getElementsByTagName('z')[0].childNodes[0].data)
                        NDITipShift[3][0]=1
            if item.nodeName=='HololensDevice':
                for hololens in item.childNodes:
                    if hololens.nodeName=='Hololens':
                        ROMName=hololens.getElementsByTagName('FileName')[0].childNodes[0].data
                        IPAddr=hololens.getElementsByTagName('IP')[0].childNodes[0].data
                        tempHololens=HololensTrackingObject(ROMName,IPAddr)
                        HololensEquipmentList.append(tempHololens)
    return [RegisHelperFileName],[NDINeedleTracker],NDITipShift,HololensEquipmentList

def LoadTrackObjectData(xmlFilePath):
    DomTree=parse(xmlFilePath)
    booklist=DomTree.documentElement
    TrackingObjectList=[]
    if booklist.nodeName=='item':
        Items=booklist.childNodes
        for item in Items:
            if item.nodeName=='TrackingObject':
                Name=item.getElementsByTagName('Name')[0].childNodes[0].data
                TargetName=item.getElementsByTagName('TargetName')[0].childNodes[0].data
                InitiateStatus=item.getElementsByTagName('InitiateStatus')[0].childNodes[0].data
                PositionNode=item.getElementsByTagName('InitiatePosition')[0].childNodes
                Position=[0,0,0]
                Position[0]=float(PositionNode[0].childNodes[0].nodeValue)
                Position[1]=float(PositionNode[1].childNodes[0].nodeValue)
                Position[2]=float(PositionNode[2].childNodes[0].nodeValue)
                RegisSetup=item.getElementsByTagName('RegistrationSetUp')[0]
                RoMName=RegisSetup.getElementsByTagName('TrackerOnModel')[0].childNodes[0].data
                MarkerPos=RegisSetup.getElementsByTagName('MarkerBallInSlicer')[0]
                TrackingPoints=[]
                TrackingPoints.append(eval(MarkerPos.getElementsByTagName('Point1')[0].childNodes[0].data))
                TrackingPoints.append(eval(MarkerPos.getElementsByTagName('Point2')[0].childNodes[0].data))
                TrackingPoints.append(eval(MarkerPos.getElementsByTagName('Point3')[0].childNodes[0].data))
                TrackingPoints.append(eval(MarkerPos.getElementsByTagName('Point4')[0].childNodes[0].data))
                TempTrackingObject=TrackingTarget(Name,TargetName,InitiateStatus,Position,np.array(TrackingPoints),RoMName)
                
                TrackingObjectList.append(TempTrackingObject)
    return TrackingObjectList