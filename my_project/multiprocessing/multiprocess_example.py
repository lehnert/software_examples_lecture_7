# -*- coding: utf-8 -*-
import os, sys
ROOT_DIR = os.path.abspath("../")
sys.path.append(ROOT_DIR)

import multiprocessing as mp

from my_nav_subsystem import nav_loop
from my_image_processing import image_processing_loop
 
if __name__ == '__main__':
        
    q = mp.Queue()
    #l = mp.Lock()
    #e = mp.Event()
    # q.put('test message')
    
    p1 = mp.Process(target=image_processing_loop, args=(q,))
    p2 = mp.Process(target=nav_loop, args=(q,))

    p1.start()
    p2.start()
    #will need to think about syncronisation here such as mp.Event or mp.Lock()
    #e.wait()   
    #print(q.get(False))

    
    #rejoins the process with this one, effectively destroying it
    p1.join()
    p2.join()

