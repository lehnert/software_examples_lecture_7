# -*- coding: utf-8 -*-
from multiprocessing import Queue
import os
import time
import cv2

def nav_loop(q):
    
    #message = q.get()
    print('module name:', __name__)
    print('parent process:', os.getppid())
    print('process id:', os.getpid())
    while True:
        print('module name:', __name__)
        print('parent process:', os.getppid())
        print('process id:', os.getpid())
        print('Process 1')
        frame = q.get(block=True)
        cv2.imshow("multiprocessing example", frame)
        cv2.waitKey(1)
        #time.sleep(0.1)
        
    
    #goal_range = 1.1
    #goal_bearing = 50.0
    
    #q.put([goal_range, goal_bearing,"finished processing nav in process 2"])
    #e.set()
