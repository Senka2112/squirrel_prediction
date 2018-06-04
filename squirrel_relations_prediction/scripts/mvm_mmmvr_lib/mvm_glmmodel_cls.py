######################
## Version 0.1 #######
## /**********************************************************************
##   Copyright 2015, Sandor Szedmak  
##   email: sandor.szedmak@uibk.ac.at
##          szedmak777@gmail.com
##
##   This file is part of Maximum Margin Multi-valued Regression code(MMMVR).
##
##   MMMVR is free software: you can redistribute it and/or modify
##   it under the terms of the GNU General Public License as published by
##   the Free Software Foundation, either version 3 of the License, or
##   (at your option) any later version. 
##
##   MMMVR is distributed in the hope that it will be useful,
##   but WITHOUT ANY WARRANTY; without even the implied warranty of
##   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
##   GNU General Public License for more details.
##
##   You should have received a copy of the GNU General Public License
##   along with MMMVR.  If not, see <http://www.gnu.org/licenses/>.
##
## ***********************************************************************/
## Edit: 2018, Senka Krivic
## added: confidences calculations
## email: krivic.senka@gmail.com
######################
import time
import numpy as np 
## ####################

## ######################################################################
def newton(y,xestim,rfunc,rfuncprime,niter=10):
  x=np.copy(xestim)
  for i in range(niter):
    x=x-(rfunc(x)-y)/rfuncprime(x)
  return(x)
## ###########################################################
## class cls_empty_class:
##   pass
## ###########################################################
## rfunc := exp(x), rfunc inverse:= log(x),
class rfunc_exp_cls():
  def __init__(self):
    return
  ## ----------------------------------------------------------
  def rfunc(self,x):
    y=np.exp(x)
    return(y)
  ## ----------------------------------------------------------
  def rfunc_inverse(self,x):
    y=np.log(x)
    return(y)
## ###########################################################
## rfunc := x exp(x), rfunc inverse:= computed by newton method,
class rfunc_xexp_cls():
  def __init__(self):
    return
  ## ---------------------------------------------
  def rfunc(self,x):
    y=np.exp(x)*x
    return(y)
  ## ----------------------------------------------------------
  def rfunc_prime(self,x):
    yprime=np.exp(x)*(1+x)
    return(yprime)
  ## ----------------------------------------------------------
  def rfunc_inverse(self,x):
    xestim=self.rfunc_init(x)
    return(newton(x,xestim,self.rfunc,self.rfunc_prime))    
  ## -----------------------------------------------------------
  def rfunc_init(self,x):
    lb=-0.5
    ix=np.where(x<lb)[0]
    xx=np.copy(x)
    xx[ix]=lb
    xestim=np.log(xx)
    return(xestim)

  ## ###########################################################
class cls_glmmodel:

  def __init__(self):
    self.col_mean=None
    self.row_mean=None
    self.total_mean=None
    self.product_correction=0  
    return

  ## --------------------------------------------------------------
  def mvm_glm_link(self,xdatacls):
    """
    It computes the row and column wise averages to additive and to multiplicative models, and centralize the training data

    Input:
    xdatacls      data class
    """

    ##print('GLM')

    xdata=xdatacls.xdata_tra ## list contains 3 arrays [0] rows, [1] cols, [2] values of known data in the table
    mdata=xdata[0].shape[0] ## number of rows of xdata[0]
    nrow=xdatacls.nrow ## total number of rows = number of objects
    ncol=xdatacls.ncol ##total number of cols= number of objects * number of properties

    tydim=xdata[2].shape # should be same value as mdata
    if len(tydim)==1:
      nydim=1
    else:
      nydim=tydim[1] #1

  ## =1 multiplicative model =0 additive model  

    col_sum=np.zeros((nrow,nydim)) #contains sum of known values per column
    row_sum=np.zeros((ncol,nydim))  #contains sum of known values per row
    col_num=np.zeros(nrow) #contains number of known values per column
    row_num=np.zeros(ncol) #contains number of known values per row
    conf = np.zeros((nrow,ncol)) # here are confidences (informativeness of data) is stored

    if xdatacls.glm_model.rfunc is not None:  ## None is for recommender
      xvalue=xdatacls.glm_model.rfunc.rfunc_inverse(xdata[2])
      xdata[2]=xvalue
    else:
      xvalue=xdata[2]

    for idata in range(mdata):
      vdata=xvalue[idata]
      col_sum[xdata[0][idata]]+=vdata
      row_sum[xdata[1][idata]]+=vdata
      col_num[xdata[0][idata]]+=1
      row_num[xdata[1][idata]]+=1

    total_sum=np.sum(col_sum)
    total_num=np.sum(col_num)
    total_mean=total_sum/total_num

    ## to avoid to divide by zero
    col_num=col_num+(col_num==0)
    row_num=row_num+(row_num==0)

    col_mean=col_sum/np.outer(col_num,np.ones(nydim))
    row_mean=row_sum/np.outer(row_num,np.ones(nydim))

    ## added for std -----------------------------
    ## computing standard deviation
    col_std=np.zeros((nrow,nydim)) # std of the column
    row_std=np.zeros((ncol,nydim)) # std of the row
    for idata in range(mdata):
      vdata=xvalue[idata]
      ## computing variance first
      col_std[xdata[0][idata]]+=(vdata-col_mean[xdata[0][idata]])**2 #calculating variance per each value in col
      row_std[xdata[1][idata]]+=(vdata-row_mean[xdata[1][idata]])**2 #calculating variance per each value in row

    col_std=np.sqrt(col_std/np.outer(col_num,np.ones(nydim))) #compute std
    row_std=np.sqrt(row_std/np.outer(row_num,np.ones(nydim))) #compute std
    
    ## -------------------------------------------

    if nydim==1:
      col_mean=np.squeeze(col_mean)
      row_mean=np.squeeze(row_mean)

    for idata in range(mdata):
      irow=xdata[0][idata]
      icol=xdata[1][idata]
      xdata[2][idata]+=-col_mean[irow]-row_mean[icol]+total_mean 
  
    ## computing confidences --------------------------------------------------
    for irow in range(nrow):
      for icol in range(ncol):
        conf[irow][icol]=((1-row_std[icol])*row_num[icol]/nrow + (1-col_std[irow])*col_num[irow]/ncol)/2
        #conf[irow][icol]=(1-col_std[irow])*col_num[irow]/ncol 
    for idata in range(mdata):
        irow=xdata[0][idata]
        icol=xdata[1][idata]
        conf[irow][icol]=1.0 #set known values confs to 1
    product_correction=1.0
    ## xdata[2]=np.exp(xdata[2])-product_correction
    ## self.col_mean=np.exp(col_mean)
    ## self.row_mean=np.exp(row_mean)
    ## self.total_mean=np.exp(total_mean)
    self.col_mean=col_mean
    self.row_mean=row_mean
    self.total_mean=total_mean
    ## to restore the expected value from 0 to 1 in case of gemetric mean
    self.product_correction=product_correction  

    ## added for std -----------------------------
    self.col_num=col_num
    self.row_num=row_num
    self.col_std=col_std
    self.row_std=row_std
    self.nrow=nrow
    self.ncol=ncol
    self.xdata0=xdata
    self.conf=conf
    ## ------------------------------------------


    return
## ###########################################################
  def mvm_glm_orig(self,xdatacls):
    """
    It computes the row and column wise averages to additive and to multiplicative models, and centralize the training data

    Input:
    xdatacls      data class
    """

    ## print('GLM')

    xdata=xdatacls.xdata_tra
    mdata=xdata[0].shape[0]
    nrow=xdatacls.nrow
    ncol=xdatacls.ncol

    tydim=xdata[2].shape
    if len(tydim)==1:
      nydim=1
    else:
      nydim=tydim[1]

  ## =1 multiplicative model =0 additive model  

    col_sum=np.zeros((nrow,nydim))
    row_sum=np.zeros((ncol,nydim))
    col_num=np.zeros(nrow)
    row_num=np.zeros(ncol)

    if xdatacls.glmmean==1:  ## geometric mean
      for idata in range(mdata):
        vdata=np.log(xdata[2][idata])
        col_sum[xdata[0][idata]]+=vdata
        row_sum[xdata[1][idata]]+=vdata
        col_num[xdata[0][idata]]+=1
        row_num[xdata[1][idata]]+=1
    else:   ## arithmetic mean
      for idata in range(mdata):
        vdata=xdata[2][idata]
        col_sum[xdata[0][idata]]+=vdata
        row_sum[xdata[1][idata]]+=vdata
        col_num[xdata[0][idata]]+=1
        row_num[xdata[1][idata]]+=1

    total_sum=np.sum(col_sum)
    total_num=np.sum(col_num)
    total_mean=total_sum/total_num

    ## to avoid to divide by zero
    col_num=col_num+(col_num==0)
    row_num=row_num+(row_num==0)

    col_mean=col_sum/np.outer(col_num,np.ones(nydim))
    row_mean=row_sum/np.outer(row_num,np.ones(nydim))
    if nydim==1:
      col_mean=np.squeeze(col_mean)
      row_mean=np.squeeze(row_mean)

    if xdatacls.glmmean==1:
      col_mean=np.exp(col_mean)
      row_mean=np.exp(row_mean)
      total_mean=np.exp(total_mean)

    if xdatacls.glmmean==1:
      product_correction=1.0
      for idata in range(mdata):
        irow=xdata[0][idata]
        icol=xdata[1][idata]
        xdata[2][idata]*=total_mean/(col_mean[irow]*row_mean[icol])
      xdata[2]-=product_correction  ## to push the expected value from 1 to 0
    else:
      for idata in range(mdata):
        irow=xdata[0][idata]
        icol=xdata[1][idata]
        xdata[2][idata]+=-col_mean[irow]-row_mean[icol]+total_mean 

    self.col_mean=col_mean
    self.row_mean=row_mean
    self.total_mean=total_mean
    ## to restore the expected value from 0 to 1 in case of gemetric mean
    self.product_correction=product_correction  

    return
## ###########################################################
              
## ####################################################################
