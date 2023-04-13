import numpy as np
import math
# Assumes obstacle is known:
def detection(location,obstacle,path):
    distance = np.sqrt((location[0]-obstacle[0])**2+(location[1]-obstacle[1])**2)

    if distance <= 10:
        cond = True # deviates
        evade(location,obstacle,15,1)
    else:
        cond = False # goes back to path
    
    return cond


def evade(location,obstacle,radius,step_size):
    dx,dy = location[0]-obstacle[0], location[1],obstacle[1]
    angle=math.atan2(dy,dx)
    angle_new=angle+step_size/radius
    location_new = obstacle[0]+radius*math.cos(angle_new), obstacle[1]+radius*math.sin(angle_new)
    return location_new

# returns new location as part of the new amended path