import numpy as np
import math
# Assumes obstacle is known:
def detection(location,obstacle,v,omega):
    
    omega_default = omega
    v_default = v
    distance = np.sqrt((location[0]-obstacle[0])**2+(location[1]-obstacle[1])**2)

    if distance <= 10:
        cond = True # deviates
        step_size = 1
        radius = 15

        dx,dy = location[0]-obstacle[0], location[1],obstacle[1]
        angle=math.atan2(dy,dx)
        omega=angle+step_size/radius    
        location = obstacle[0]+radius*math.cos(omega), obstacle[1]+radius*math.sin(omega)
        v = 0.5
    else:
        cond = False # goes back to path
        omega = omega_default
        v = v_default

    return cond, v, omega



