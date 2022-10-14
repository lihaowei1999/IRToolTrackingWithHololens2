import cv2
import numpy as np
from scipy.io import savemat,loadmat
import math
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import copy
import os
from pathlib import Path
# region growing for binarized data
# the coding of label start from 1
def DetectBalls(img,_minSize,_maxSize,distanceTolerance,pre_retval=0,pre_labels=[],pre_stats=[],pre_centroids=[],pre_labelcorr=[]):
    retval,labels,stats,centroids=cv2.connectedComponentsWithStats(img,connectivity=8)
    stats_ori=stats
    centroids_ori=centroids
    _relabel=[]
    for i in range(retval-1,-1,-1):
        if stats[i,4]<_minSize or stats[i,4]>_maxSize:
            retval-=1
            labels[labels==i]=-1
            stats=np.delete(stats,i,0)
            centroids=np.delete(centroids,i,0)
        else:
            _relabel.insert(0,i)
    # Previous frame not considered, relabel and return immediately
    # if not pre_retval:
    if not False:
        for i in range(retval):
            labels[labels==_relabel[i]]=i
        label_corr=list(range(retval))
        # print("returning")
        return [retval,labels,np.array(stats),np.array(centroids),np.array(label_corr)]

def UVD_RigidTransform(p,q,w=[]):
    Num_points=p.shape[0]
    p_average=np.mean(p,0)
    q_average=np.mean(q,0)
    p_residual=np.array([_p_iter-p_average for _p_iter in p])
    q_residual=np.array([_q_iter-q_average for _q_iter in q])
    if not len(w)==Num_points:
        w=np.diag(np.ones(Num_points))
    S=p_residual.T@w@q_residual
    # here S=U Sigma V
    U,Sigma,V=np.linalg.svd(S)
    detuv=np.linalg.det(V.T@U.T)
    _sig=np.diag(np.ones(U.shape[1]))
    if detuv<0:
        _sig[-1,-1]=-1
    R=V.T@_sig@U.T
    t=q_average.T-R@p_average.T
    TransformMatrix=np.zeros((4,4))
    TransformMatrix[0:3,0:3]=R
    TransformMatrix[0:3,3]=t
    TransformMatrix[3,3]=1
    # Validation
    p_temp=np.zeros((4,Num_points))
    p_temp[-1,:]=1
    p_temp[0:3,:]=p.T
    q_backproj=TransformMatrix@p_temp
    q_temp=np.zeros((3,Num_points))
    _q=np.array(q)
    q_temp[0:3,:]=_q.T
    err_=q_backproj[0:3,:]-q_temp
    err=np.mean(np.sqrt(np.sum(err_**2,1)))
    return TransformMatrix,err

class AHAT_NDIToolSceneFrame:
    def __init__(self,_Points=[],_xyd=[],_timestamp=0):
        if not len(_Points)==0:
            self.xyd_Data=_xyd
            self.Points=_Points
            self.Side_List=[]
            self.Map=[]
            self.timestamp=_timestamp
            # For First Round Search
            self.PossibleSolutionTools=None
            self.ErrSolutionTools=None
            # For Final Round Search
            self.SearchingStatus=None
            self.Transformatrix=None
            self.SearchingErr=None
            self.FinalSolution=None
            self.SolutionPoints=None
            ## after kalman filter
            self.xyd_filtered=None
            self.xyz_filtered=None
            self.TransformatrixFiltered=None
            self.FinalSolutionFiltered_xyz=None
            self.FinalSolutionFiltered_xyd=None
            self.SearchingErrFiltered=None
            self.PixelXYData=None
            self.ConstructMap()
            

    def ConstructMap(self):
        
        if len(self.Points)==0:
            print('No Point Exists')
            return
        else:
            self.Num_points=self.Points.shape[0]
            self.Map=np.zeros([self.Num_points,self.Num_points])
            # Sequentially build Map
            for i in range(self.Num_points):
                for j in range(i+1,self.Num_points):
                    # side from i to j
                    Distance=math.sqrt(sum((self.Points[i,:]-self.Points[j,:])**2))
                    self.Map[i,j]=Distance
                    self.AppendSide(Side(i,j,Distance))
                    # print(str(i)+'    '+str(j))

    def AppendSide(self,side):
        if len(self.Side_List) < 1:
            self.Side_List.append(side)
        else:
            
            _Lengths=[k.length for k in self.Side_List]
            # print(_Lengths)
            if side.length >= _Lengths[-1]:
                self.Side_List.append(side)
                return
            if side.length <= _Lengths[0]:
                self.Side_List.insert(0,side)
                return
            _inds=[k>side.length for k in _Lengths]
            #  print(_inds)
            _ind=_inds.index(True) if True in _inds else len(_inds)
            self.Side_List.insert(_ind,side)
            return
            

class Side:
    def __init__(self,_in,_out,_length):
        self.id_in=_in
        self.id_out=_out
        self.length=_length


class NDI_Tool:
    def __init__(self,_Points):
        if not len(_Points)==0:
            self.Points=_Points
            self.Side_List=[]
            self.Map=[]
            self.ConstructMap()

    def ConstructMap(self):
        if len(self.Points)==0:
            print('No Point Exists')
            return
        else:
            self.Num_points=self.Points.shape[0]
            self.Map=np.zeros([self.Num_points,self.Num_points])
            # Sequentially build Map
            for i in range(self.Num_points):
                for j in range(i+1,self.Num_points):
                    # side from i to j
                    Distance=math.sqrt(sum((self.Points[i,:]-self.Points[j,:])**2))
                    self.Map[i,j]=Distance
                    self.AppendSide(Side(i,j,Distance))

    def AppendSide(self,side):
        if len(self.Side_List) < 1:
            self.Side_List.append(side)
        else:
            
            _Lengths=[k.length for k in self.Side_List]
            # print(_Lengths)
            if side.length >= _Lengths[-1]:
                self.Side_List.append(side)
                return
            if side.length <= _Lengths[0]:
                self.Side_List.insert(0,side)
                return
            _inds=[k>side.length for k in _Lengths]
            #  print(_inds)
            _ind=_inds.index(True)
            self.Side_List.insert(_ind,side)
            return

class DepthFilterKalman:
    def __init__(self):
        self.Filter=None
        
    def startup(self,value):
        self.Filter=KalmanFilter(dim_x=2, dim_z=1)
        self.Filter.x = np.array([[value],
                                [0.]])       # initial state (location and velocity)

        self.Filter.F = np.array([[1.,1.],
                        [0.,1.]])    # state transition matrix

        self.Filter.H = np.array([[1.0,0.]])    # Measurement function
        self.Filter.P *= 1000.                 # covariance matrix
        self.Filter.R = 5                      # state uncertainty
        self.Filter.Q = Q_discrete_white_noise(2, 0.1, .3) # process uncertainty

    def AddData(self,value):
        if not self.Filter:
            self.startup(value)
            return value
        else:
            self.Filter.predict()
            self.Filter.update(value)
            # print(value-self.Filter.x[0][0])
            return self.Filter.x[0][0]

    def cleanup(self):
        self.Filter=None



class AHAT_NDIToolScene:
    def __init__(self,TolSide=6,TolAvg=5):
        # Tolerance Point is set to be 500
        self.Tools=[]
        self.Paras={}
        self.ToleranceSide=TolSide
        self.ToleranceAvg=TolAvg
        self.PreviousStatus={}
        self.PossibleSolutionTools=[]
        self.ErrSolutionTools=[]
        ## for every tool, the number of kalman filters equal to the number of points
        self.KalmanFilters=[]
        self.root_ws = Path(os.path.dirname(os.path.realpath(__file__)))
        Prime_list_file=loadmat(str(self.root_ws/"Prime_list_500.mat"))
        self.Prime_list_500=Prime_list_file['Prime_list_500']
        

    def InitFilter(self):
        for tl in range(len(self.Tools)):
            KalmanForTool=[]
            for kf in range(len(self.Tools[tl].Points)):
                KalmanForTool.append(DepthFilterKalman())
            self.KalmanFilters.append(KalmanForTool)
        
    

    def LoadTools(self,ToolList):
        self.Tools=ToolList
        
        self.ValidateTools()
        self.InitFilter()
    
    def setPara(self,Tolerance_Side=6,Tolerance_Avg=5):
        self.ToleranceSide=Tolerance_Side
        self.ToleranceAvg=Tolerance_Avg

    def ShowTools(self):
        pass

    # Validate that the tool can be separated(enough difference exists)
    def ValidateTools(self):
        # Validate if the difference between Tools is enough
        pass

    def SearchTools(self,ToolSceneFrame:AHAT_NDIToolSceneFrame):
        # doesn't consider information between frames now
        self.PossibleSolutionTools=[]
        self.ErrSolutionTools=[]
        # First search possible tools for every single tool
        for i in range(len(self.Tools)):
            Possible_Solutions,Err_Solutions=self.SearchTool(ToolSceneFrame,self.Tools[i],self.ToleranceSide,self.ToleranceAvg)
            self.PossibleSolutionTools.append(Possible_Solutions)
            self.ErrSolutionTools.append(Err_Solutions)
        ToolSceneFrame.PossibleSolutionTools=self.PossibleSolutionTools
        ToolSceneFrame.ErrSolutionTools=self.ErrSolutionTools
        # print("##############")
        # print(self.PossibleSolutionTools)
        # Search Real Tool For Every Tool
        res_status,res_TransformMatrix,res_err,res_solu=self.UnionSegmentation(ToolSceneFrame)
        # res_solu : [[id,],]
        # Return xyz position as well
        xyz_solu=[]
        Point3List=ToolSceneFrame.Points
        for i in range(len(res_status)):
            if not res_status[i]:
                xyz_solu.append([])
            else:
                xyz_tool=[]
                for j in range(len(res_solu[i])):
                    xyz_tool.append(Point3List[res_solu[i][j]])
                xyz_solu.append(np.array(xyz_tool))
        
        

        ToolSceneFrame.SearchingStatus=res_status
        ToolSceneFrame.Transformatrix=res_TransformMatrix
        ToolSceneFrame.SearchingErr=res_err
        ToolSceneFrame.FinalSolution=res_solu
        ToolSceneFrame.SolutionPoints=xyz_solu

        self.FilterResult(ToolSceneFrame)

        return res_status,res_TransformMatrix,res_err,res_solu
        # return status,TransformMatrix,err_
    
    def FilterResult(self,ToolSceneFrame):
        origin_xydData=ToolSceneFrame.xyd_Data
        finalsolution_xyz=[]
        finalsolution_xyd=[]
        errfilterd=[]
        tffiltered=[]

        for toolnum in range(len(self.Tools)):
            if not ToolSceneFrame.SearchingStatus[toolnum]:
                for Kalf in self.KalmanFilters[toolnum]:
                    Kalf.cleanup()
                finalsolution_xyd.append([])
                finalsolution_xyz.append([])
                tffiltered.append(np.zeros([4,4]))
                errfilterd.append(-1)
            else:
                xyd_this_tool=[]
                for p in range(len(self.Tools[toolnum].Points)):
                    xyd_this_tool.append(origin_xydData[ToolSceneFrame.FinalSolution[toolnum][p]])
                
                xyd_this_tool_filterd=copy.deepcopy(xyd_this_tool)
                xyz_this_tool=[]
                for pointnum in range(len(xyd_this_tool)):
                    # print(self.KalmanFilters[toolnum][pointnum])
                    # print(xyd_this_tool)
                    # print(xyd_this_tool[pointnum][2])
                    d_f=self.KalmanFilters[toolnum][pointnum].AddData(xyd_this_tool[pointnum][2])
                    # print(d_f-xyd_this_tool_filterd[pointnum][2])
                    xyd_this_tool_filterd[pointnum][2]=d_f
                    # print(np.array(xyd_this_tool)-np.array(xyd_this_tool_filterd))
                    l=math.sqrt(xyd_this_tool_filterd[pointnum][0]**2+xyd_this_tool_filterd[pointnum][1]**2+1)
                    xyz_this_tool.append(np.array([xyd_this_tool_filterd[pointnum][0]/l*d_f,xyd_this_tool_filterd[pointnum][1]/l*d_f,d_f/l]))

                # tranfiltered,err=UVD_RigidTransform(np.array(xyz_this_tool),self.Tools[toolnum].Points)
                tranfiltered,err=UVD_RigidTransform(self.Tools[toolnum].Points,np.array(xyz_this_tool))
                # print(np.array(xyd_this_tool)-np.array(xyd_this_tool_filterd))
                finalsolution_xyd.append(np.array(xyd_this_tool_filterd))
                finalsolution_xyz.append(np.array(xyz_this_tool))
                errfilterd.append(err)
                tffiltered.append(tranfiltered)
        ToolSceneFrame.FinalSolutionFiltered_xyz=finalsolution_xyz
        ToolSceneFrame.FinalSolutionFiltered_xyd=finalsolution_xyd
        ToolSceneFrame.SearchingErrFiltered=errfilterd
        ToolSceneFrame.TransformatrixFiltered=tffiltered




    def SearchToolsWithKalman(self,ToolSceneFrame:AHAT_NDIToolSceneFrame):
        pass


    def UnionSegmentation(self,ToolSceneFrame:AHAT_NDIToolSceneFrame):
        res_status=[]
        res_TransformMatrix=[]
        res_err=[]
        res_solu=[]
        cache=[]
        Scene_Points=ToolSceneFrame.Points
        solu_num=np.zeros(len(self.Tools))
        for tool_num in range(len(self.Tools)):
            temp_status=[]
            temp_Transformatrix=[]
            temp_err=[]
            
            Tool_Points=self.Tools[tool_num].Points
            PossibleSolutionTool=self.PossibleSolutionTools[tool_num]
            if not len(PossibleSolutionTool)>0:
                temp_status.append(False)
                temp_Transformatrix.append(np.zeros((4,4)))
                temp_err.append(-1)
                res_status.append(temp_status)
                res_TransformMatrix.append(temp_Transformatrix)
                res_err.append(temp_err)
                res_solu.append([])
                # cache.append((tool_num,False,np.zeros((4,4)),-1,[]))
                continue
            
            for i in range(len(PossibleSolutionTool)):
                Matching_id=PossibleSolutionTool[i]
                Scene_Tool_Points=[list(Scene_Points[_iter]) for _iter in Matching_id]
                TransformMatrix,err=UVD_RigidTransform(Tool_Points,Scene_Tool_Points)
                temp_status.append(True)
                temp_Transformatrix.append(TransformMatrix)
                temp_err.append(err)
            # PossibleSolutionTool
            # delete repetitive solutions
            temp_id_PossibleSolution=[]
            for _sltemp in PossibleSolutionTool:
                temp_id_PossibleSolution.append(np.prod([self.Prime_list_500[0][_itertemp] for _itertemp in _sltemp]))
            temp_id_notepetitive=set(temp_id_PossibleSolution)
            _norepstatus=[]
            _norep_transformMatrix=[]
            _norep_err=[]
            _norep_solu=[]
            for _id_notreprtitive in temp_id_notepetitive:
                _id_sl_this_vl=np.where(np.array(temp_id_PossibleSolution)==_id_notreprtitive)
                _er_sl_this_vl=np.array(temp_err)[temp_id_PossibleSolution==_id_notreprtitive]
                _min_er_pos=np.where(_er_sl_this_vl==min(_er_sl_this_vl))
                _min_tot_pos=_id_sl_this_vl[0][_min_er_pos[0][0]]
                _norepstatus.append(True)
                _norep_err.append(temp_err[_min_tot_pos])
                _norep_solu.append(PossibleSolutionTool[_min_tot_pos])
                _norep_transformMatrix.append(temp_Transformatrix[_min_tot_pos])
                cache.append((tool_num,True,temp_Transformatrix[_min_tot_pos],temp_err[_min_tot_pos],PossibleSolutionTool[_min_tot_pos]))
                solu_num[tool_num]=solu_num[tool_num]+1
            # res_status.append(temp_status)
            # res_TransformMatrix.append(temp_Transformatrix)
            # res_err.append(temp_err)
            res_status.append(_norepstatus)
            res_TransformMatrix.append(_norep_transformMatrix)
            res_err.append(_norep_err)
            res_solu.append(_norep_solu)


        # Data: set{Tool_id->tuple(Tool_id,Status:Bool,TransformMatrix4x4,Err,Solution:[])}
        #        
        res_set={}
        # only deal with existed information

        while sum(solu_num)>0:
            errs_temp=[sol[3] for sol in cache]
            min_er_pos=np.where(np.array(errs_temp)==min(errs_temp))[0][0]
            this_chache=cache[min_er_pos]
            res_set[this_chache[0]]=this_chache
            solu_num[this_chache[0]]=solu_num[this_chache[0]]-1
            # delete
            _=cache.pop(min_er_pos)
            for cache_i in range((len(cache)-1),-1,-1):
                if cache[cache_i][0]==this_chache[0] or len(np.intersect1d(this_chache[4],cache[cache_i][4]))!=0:
                    solu_num[cache[cache_i][0]]=solu_num[cache[cache_i][0]]-1
                    cache.pop(cache_i)
        hav_id=list(res_set.keys())
        for false_tool_num in range(len(self.Tools)):
            if not false_tool_num in hav_id:
                res_set[false_tool_num]=(false_tool_num,False,np.zeros((4,4)),-1,[])
        r_status=[]
        r_err=[]
        r_tran=[]
        r_solu=[]
        for tn in range(len(self.Tools)):
            r_status.append(res_set[tn][1])
            r_tran.append(res_set[tn][2])
            r_err.append(res_set[tn][3])
            r_solu.append(res_set[tn][4])
        
        
        # print(r_solu)
        return r_status,r_tran,r_err,r_solu
        # return res_status,res_TransformMatrix,res_err,res_solu

        

    def SearchTool(self,ToolSceneFrame:AHAT_NDIToolSceneFrame,Tool:NDI_Tool,ToleranceSide,ToleranceAvg):
        Current_searching_status=[]
        ToolPointsNum=Tool.Num_points
        ScenePointsNum=ToolSceneFrame.Num_points
        Possible_solutions=[]
        Err_solutions=[]
        ## pre process, delete the impossible sides, re-arrange the points
        Tool_Sides=Tool.Side_List
        Tool_Map=Tool.Map
        Tool_Sides_length=[k.length for k in Tool_Sides]
        Scene_Map=ToolSceneFrame.Map
        Scene_Sides=ToolSceneFrame.Side_List
        Scene_Sides_length=[k.length for k in Scene_Sides]
        # print(len(SceneSideList))
        # search for sides
        Eligible_side=list(np.where([abs(i-Tool_Map[0,1])<ToleranceSide for i in Scene_Sides_length])[0])
        if len(Eligible_side)==0:
            return Possible_solutions,Err_solutions
        for i in Eligible_side:
            Current_searching_status.append(tuple([[Scene_Sides[i].id_in,Scene_Sides[i].id_out],[Scene_Sides_length[i]-Tool_Map[0,1]]]))
            Current_searching_status.append(tuple([[Scene_Sides[i].id_out,Scene_Sides[i].id_in],[Scene_Sides_length[i]-Tool_Map[0,1]]]))
        while not len(Current_searching_status)==0:
            _status_this_process=Current_searching_status.pop()
            searched_id=_status_this_process[0]
            searched_err=_status_this_process[1]
            num_has_searched=len(searched_id)
            if num_has_searched==ToolPointsNum:
                Possible_solutions.append(searched_id)
                Err_solutions.append(searched_err)
                continue
            for new_node_id in range(ScenePointsNum):
                if not new_node_id in searched_id:
                    # temp_err=[abs(Scene_Map[searched_id[old_node_id],new_node_id]-Tool_Map[old_node_id,num_has_searched]) for old_node_id in range(num_has_searched)]
                    temp_err=[]
                    for old_node_id in range(num_has_searched):
                        if searched_id[old_node_id]<new_node_id:
                            temp_err.append(abs(Scene_Map[searched_id[old_node_id],new_node_id]-Tool_Map[old_node_id,num_has_searched]))
                        else:
                            temp_err.append(abs(Scene_Map[new_node_id,searched_id[old_node_id]]-Tool_Map[old_node_id,num_has_searched]))
                    if sum([int(er>ToleranceSide) for er in temp_err])>0:
                        continue
                    if np.mean(np.concatenate((searched_err,temp_err)))>ToleranceAvg:
                        continue
                    Current_searching_status.append(tuple([list(np.concatenate((searched_id,[new_node_id]))),list(np.concatenate((searched_err,temp_err)))]))
            # break
        return Possible_solutions,Err_solutions