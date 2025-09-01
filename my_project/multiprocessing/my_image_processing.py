# -*- coding: utf-8 -*-
from multiprocessing import Queue
import os
import time
import cv2


def image_processing_loop(q):
    
    #msg = q.get()

    camera = cv2.VideoCapture(0)
    
    while True:
        print('module name:', __name__)
        print('parent process:', os.getppid())
        print('process id:', os.getpid())
        print('Process 2')

        retVal, currentFrame = camera.read()

        q.put(currentFrame)
        #time.sleep(0.1)
    
    #do processing on image here
    
    #return processed image or return values
    #q.put('processed image data')
   
