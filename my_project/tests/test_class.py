#some example code for running scripts within a folder inside your project
import os, sys
ROOT_DIR = os.path.abspath("../")
sys.path.append(ROOT_DIR)

from vision import object_class as obj

#‘Create an obstacle with id 1, 0.56 m away at a bearing of 11 degrees'
Obs1 = obj.Obstacle(1, 0.56, 11) #‘Create an obstacle with id 2, 1.6 m away at a bearing of -7 degrees'
Obs2 = obj.Obstacle(2, 1.6, -7)

#‘Call the display location method of Obs2'
Obs2.displayLocation()


#‘Destroying obs2 object’
del Obs2
