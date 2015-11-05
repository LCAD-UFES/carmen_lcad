import pyCarmen
from pylab import *
from pyRobot import pyMessageHandler
from scipy import *
from sys import argv
from time import time, sleep
from random import random


ion()
X, Y = [], []
laser_plot, = plot(X, Y)

class myRobot(pyMessageHandler):
    def __init__(self):
        pyMessageHandler.__init__(self)
        print "starting"
        self.timeout = 0.5
        self.prev_time = time()

    def callback(self, the_type, message):
        global X, Y

        print "tk:", the_type
        print time() - self.prev_time 
        sleep(random()*0.1)
        if(time() - self.prev_time < self.timeout):

            return
        self.prev_time = time()


        
        #print "laser config", message["laser_config"]
        #print "data", message["range"]


        start_angle = message["laser_config"]["start_angle"]
        increment = message["laser_config"]["angular_resolution"]
        
        #print message["laser_config"]["start_angle"]
        th = arange(0.001,increment*len(message["range"]), increment)+(pi/180.0)*message["laser_config"]["start_angle"]
        
        print "len(th)", len(th)
        print "len(range)", len(message["range"])
        #print "setting message"
        X,Y = [],[]
        i = 0
        R = array(message["range"])
        
        for t in th:
            X.append(R[i]*cos(t))
            Y.append(R[i]*sin(t))
            i+=1
                
        #if((time() -self.prev_time) > self.timeout):                
            #print "plotting data"
        laser_plot.set_data(X, Y)
        max_range = message["laser_config"]["maximum_range"]
        axis([-max_range-5, max_range+5, -max_range-5, max_range+5])
        draw()
        
        #self.prev_time = time()



if __name__ == "__main__":
    

    # Create an callback
    robot = myRobot().__disown__()
    
    # Create callers
    # the exact syntax of these might change a bit
    if(not len(argv) == 2):
        print "usage: python viewlaser {frontlaser, rearlaser, laser5}"

    if(argv[1] == "frontlaser"):
        print "viewing front laser"
        fl = pyCarmen.front_laser(robot)
        

    elif(argv[1] == "rearlaser"):
        print "viewing rear laser"
        rl = pyCarmen.rear_laser(robot)
    
    elif(argv[1] == "laser5"):
        print "viewing laser 5"
        laser = pyCarmen.laser5(robot)
    
    # Dispatch and don't return

    robot.start()
    robot.connect()
