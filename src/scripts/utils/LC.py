import numpy as np

def lc(alpha,beta,WP_index):
    LANE_WIDTH=0.25
    vec_x,vec_y=alpha[WP_index]-alpha[WP_index-1],beta[WP_index]-beta[WP_index-1]
    sum=np.sqrt(vec_x**2+vec_y**2)
    dir_x=vec_x/sum
    dir_y=vec_y/sum
    dir_vec=np.array([dir_x,dir_y])
    counterclockwise_rotation_vector=np.array([[0,-1],[1,0]])
    normal_dir_vector=np.dot(counterclockwise_rotation_vector,dir_vec)
    LEFT_THRESHOLD_X=alpha[WP_index]-(LANE_WIDTH/2)*normal_dir_vector[0]
    LEFT_THRESHOLD_Y=beta[WP_index]-(LANE_WIDTH/2)*normal_dir_vector[1]
    RIGHT_THRESHOLD_X=alpha[WP_index]+(LANE_WIDTH/2)*normal_dir_vector[0]
    RIGHT_THRESHOLD_Y=beta[WP_index]+(LANE_WIDTH/2)*normal_dir_vector[1]

    return LEFT_THRESHOLD_X,LEFT_THRESHOLD_Y,RIGHT_THRESHOLD_X,RIGHT_THRESHOLD_Y
        