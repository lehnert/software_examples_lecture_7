# -*- coding: utf-8 -*-


from navigation import state_machine as sm

import numpy as np
import math
import time

class NavClass:
 
    
    #FOV = number of degrees for camera view
    def __init__(self, FOV):
        self.FOV = FOV
        self.attractive_field = np.zeros(FOV)
        self.repulsive_field = np.zeros(FOV)
        
        self.forward_vel = 0        #desired forward velocity
        self.rot_vel = 0            #desired rotational velocity
        
        self.my_sm = sm.stateMachine()

    def update(self):
        #run state machine update
        self.my_sm.update_state()
        #compute potential fields
        self.computeAttractiveField(10)
        time.sleep(0.01)

    def  computeAttractiveField(self,goal):
        # for the input goal in degrees iterate
        # through attractive field and compute field
        gradient = 1.0/180.0
        
        for i in range(0,math.ceil(self.FOV/2)):
            self.attractive_field[goal - i] = 1 - i*gradient
            self.attractive_field[goal + i] = 1 - i*gradient

