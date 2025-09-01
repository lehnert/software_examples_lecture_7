# import os, sys
# ROOT_DIR = os.path.abspath("../")
# sys.path.append(ROOT_DIR)


from navigation import NavClass as nv
from vision import CamFrameGrabber as cam

import time
import cv2

if __name__ == '__main__':
  

    Frequency = 10 #Hz
    Interval = 1.0/Frequency

    camera = cam.CamFrameGrabber(src=0, width=320, height=640).start()
   
    navSystem = nv.NavClass(60)
    
    # execute control rate loop
    # very simple main robot loop
    while True:
        print('Executing Loop')
        now = time.time()            # get the time
        
        frame = camera.getCurrentFrame()
        elapsed = time.time()
        
        #do image processing here
        
        #objects_detected = process_image(frame)
        #update navigation system
        #f_vel, rot_vel, pick_sample, pick_rock = navSystem.update(objects_detected, sampleSystem)
        navSystem.update()
        #mobility.execute(f_vel, rot_vel)

        #sample_system(pick_sample, pick_rock)
        # send new commands to motors
        
        #mobility.updateMotors(navSystem.forward_vel, navSystem.rot_vel)

        elapsed = time.time() - now  # how long was it running?
        time.sleep(Interval-elapsed) # wait for amount of time left from interval
        
        cv2.imshow("test window", frame)
        cv2.waitKey(1)


        ## this is not needed but good for debugging rate
        elapsed2 = time.time() - now
        rate2 = 1.0/elapsed2
        print("Processing Rate after sleep is: {}.".format(rate2))