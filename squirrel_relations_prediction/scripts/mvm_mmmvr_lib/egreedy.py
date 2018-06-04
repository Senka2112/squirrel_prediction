# -*- coding: utf-8 -*-
"""
Created on Fri Jun  1 00:15:38 2018

@author: c7031098
Senka Krivic
e-greedy for action exploration
"""

import numpy as np
import numpy.random as nr

class EGreedy(object):

    def __init__(self, epsilon):
        
        self.epsilon=epsilon
        
    def action(self,conf):
        
        self.conf=conf
        (i,j) = np.unravel_index(conf.argmin(), conf.shape) #find minimum confidence
        minval=conf[i][j]
        min_ind=np.argwhere(conf==minval)#get all min values indices
        #print(min_ind)
        (i,j)=min_ind[nr.choice(min_ind.shape[0]),:] #get one of min values randomly

        if nr.rand() < self.epsilon:
            (i,j)=self.rand_action(conf)

        return (i,j)
    
    def rand_action(self,conf):
        
        unknownc=np.argwhere(conf<1)#remove known values
        (i,j)=unknownc[nr.choice(unknownc.shape[0]),:] #select random pair of indices

        return (i,j)

#    def train(self, env, nsteps = 1000):
#
#        for i in range(nsteps):
#            a = self.action()
#            r = env.run(a) # the environment 
#            self.update(a,r)
#            
def main():
    confs=np.array([[0.3,0.2,0.0],[0.0,0.0,0.1]])
    explr=EGreedy(0.2)
    print(explr.action(confs))
    
    #print(confs)
    
#main()