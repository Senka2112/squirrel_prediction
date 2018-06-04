#!/usr/bin/python

from squirrel_prediction_msgs.srv import *
import rospy

import numpy as np

import mvm_mmmvr_lib.mvm_mvm_cls as mvm_mvm_cls
import mvm_mmmvr_lib.mmr_setparams as mmr_setparams
import mvm_mmmvr_lib.load_data as load_data
import mvm_mmmvr_lib.mvm_validation_cls as mvm_validation_cls
import mvm_mmmvr_lib.egreedy as egreedy

rospy.init_node('recommend_action')

def test_mvm_main(workmode, data_path, input_file, number_of_columns):

  params=mmr_setparams.cls_params()

  xdatacls=mvm_mvm_cls.cls_mvm()
  xplore=egreedy.EGreedy(0.2)
  nfold=xdatacls.nfold

  nfold0=nfold    ## n-fold cross validation
  npar = 1
  
  for ipar in range(npar):

    Y0=np.array([0,1,2,3])
    
    ctables=load_data.cls_label_files(data_path, input_file, input_file,number_of_columns)  ## data loading object
    ctables.irowcol=xdatacls.rowcol  ## set the row-col or col-row processing
    
    (xdata,nrow2,ncol2,ifixtrain,ifixtest)=ctables.load_twofiles()
    print("broj redova ",nrow2)
    print('xdata size lines',len(xdata))
    print('number of columns',len(xdata[0]))
    print('line 1 ',xdata[0][0], ' ',xdata[0][1],' ', xdata[0][2])
    print('line 1 ',xdata[1][0], ' ',xdata[1][1],' ', xdata[1][2])
    xdatacls.categorymax=xdata[2].max()-xdata[2].min()+1                    
    xdatacls.load_data(xdata,[],xdatacls.categorymax, \
                     int(nrow2),int(ncol2),Y0)
    xdatacls.ifixtrain=ifixtrain
    xdatacls.ifixtest=ifixtest

    xdatacls.YKernel.ymax=1 # it will be recomputed in mvm_ranges
    xdatacls.YKernel.ymin=0
    xdatacls.YKernel.yrange=100 # it will be recomputed in classcol_ranges
    xdatacls.YKernel.ystep=(xdatacls.YKernel.ymax-xdatacls.YKernel.ymin) \
                            /xdatacls.YKernel.yrange
 
    xdatacls.prepare_repetition_folding(init_train_size=100)
    nrepeat0=xdatacls.nrepeat0
    nfold0=xdatacls.nfold0
    xdatacls.category=3
    

    # ----------------------------------------------------------------------

    nval=max(xdatacls.YKernel.valrange)+1
    xconfusion3=np.zeros((nrepeat0,nfold0,xdatacls.YKernel.ndim,nval,nval))

    ireport=0
    for irepeat in range(nrepeat0):

      xdatacls.nfold0=xdatacls.nfold
      xdatacls.prepare_repetition_training()
   
      for ifold in range(nfold0):
        xdatacls.prepare_fold_training(ifold) ##here calculated confidences
        #print(xdatacls.glm_model.row_num)
        #print(xdatacls.glm_model.col_num)
        #print(xdatacls.glm_model.row_std)
        #print(xdatacls.glm_model.col_std)
        #print(xdatacls.glm_model.col_mean)
        #print(xdatacls.glm_model.row_mean)
        #print(xdatacls.glm_model.xdata0[0])
        #print(xdatacls.glm_model.xdata0[1])
        act=xplore.action(xdatacls.glm_model.conf)
        print(act)
        obj1=act[0]
        rel=act[1]%number_of_columns
        obj2=(act[1]-rel)/number_of_columns
	actn=np.zeros(2)
        actn[0]=obj1*len(xdatacls.glm_model.conf)+obj2
	actn[1]=rel
        print(actn)


  print('(squirrel prediction) Recommendation of an action done.')    
  
  return actn

def callback(data):
    #input
    act=test_mvm_main(0, data.data_path, data.input_file, data.number_of_columns)
    #print(act)
    resp = RecommendActionResponse()
    resp.row=act[0]
    resp.column=act[1]
    resp.finished=True
    #RecommendActionResponse.FAILURE = uint8(0)
    #RecommendRelationsResponse.result = uint8(1)
    return resp

if __name__ == "__main__":
	rospy.sleep(2)
	s = rospy.Service("/squirrel_action_recommendation", RecommendAction, callback)
	rospy.loginfo("(squirrel prediction) Ready for recommending actions.")
	rospy.spin()
