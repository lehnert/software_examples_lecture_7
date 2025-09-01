class Obstacle:
#'Common  base class for Obstacles'
    obsCount=0
    def __init__(self, ID, Dist, Bearing):
        self.Dist  =  Dist
        self.Bearing   =  Bearing
        self.ID   =  ID
        Obstacle.obsCount  +=  1

    def  displayCount(self):
        print ("There are {0} obstacles".format(Obstacle.obsCount))

    def  displayLocation(self):
        print  ("Id  :  {0}".format(self.ID))
        print  ("Distance to obstacle:  {0:2.2f}".format(self.Dist,2))
        print  ("Bearing to obstacle:  {0}".format(self.Bearing))

    def __str__(self):
        return "Object ID: {0}, distance: {1:2.2f}, bearing: {2}".format(self.ID, self.Dist, self.Bearing)

    def  __del__(self):
        Obstacle.obsCount  -=1



if __name__ == '__main__':
    print("called as main executable")
else:
    print("called as import")