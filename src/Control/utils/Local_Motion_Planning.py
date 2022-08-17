import numpy as np
import matplotlib.pyplot as plt
import sys
# Input data : Target_Lane, Lane_changing
# Output data : alpha, beta
from utils.genWP import WP_X,WP_Y

def local_motion_planning(WP_X,WP_Y,Target_Lane):
    LANE_WIDTH=0.35
    tmp_shifting_x=np.zeros(len(WP_X)).reshape(len(WP_X),1)
    tmp_shifting_y=np.zeros(len(WP_X)).reshape(len(WP_X),1)
    tmp_Direction_vector=np.zeros(len(WP_X)*2).reshape(len(WP_X),2) # 197x2
    Direction_vector=np.zeros_like(tmp_Direction_vector) # 197x2
    normal_Direction_vector=np.zeros_like(Direction_vector)
    normal_Direction_vector=normal_Direction_vector.T # Transpose 2x197


    for i in range(1,len(WP_X)+1): # 1~197
        if (i<len(WP_X)):
            tmp_Direction_vector[i-1,:]=np.array([[WP_X[i]-WP_X[i-1]],[WP_Y[i]-WP_Y[i-1]]]).reshape(1,2)
        else:
            tmp_Direction_vector[i-1,:]=np.array([[WP_X[i-1]-WP_X[i-2]],[WP_Y[i-1]-WP_Y[i-2]]]).reshape(1,2)

    for i in range(len(tmp_Direction_vector)):
        Direction_vector[i,:] = tmp_Direction_vector[i,:]/np.linalg.norm(tmp_Direction_vector[i,:])

    counterclockwise_rotation_vector=np.array([[0,-1],[1,0]])
    Direction_vector=Direction_vector.T

    normal_Direction_vector = np.dot(counterclockwise_rotation_vector,Direction_vector)
    normal_Direction_vector=normal_Direction_vector.T

    for i in range(len(WP_X)):
        tmp_shifting_x[i] =  -Target_Lane*LANE_WIDTH*normal_Direction_vector[i, 0]
        tmp_shifting_y[i] =  -Target_Lane*LANE_WIDTH*normal_Direction_vector[i, 1]

    alpha=WP_X+tmp_shifting_x
    beta=WP_Y+tmp_shifting_y
    return alpha,beta # Waypoint shifting
    

