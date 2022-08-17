import math
def FVLTreatAss(Rear_Wheel_Velocity, FVL_Relative_Velocity, FVL_dv_y, FVL_yaw, FVL_Clearance, FVL_ds_y):
    
    v_ego = Rear_Wheel_Velocity
    dv_x = FVL_Relative_Velocity
    dv_y = FVL_dv_y
    rel_yaw = FVL_yaw
    ds_x = FVL_Clearance
    ds_y = FVL_ds_y 

    TTF_PARAMETER_TAU = 0.7 + 0.15; # system react time 0.3~1.2s, brake react time 0.15s
    TTF_PARAMETER_Sr = 1
    TTF_PARAMETER_decel_ego = 6
    TTF_PARAMETER_decel_sur = 6
    TTF_PARAMETER_width_ego = 1.8
    TTF_PARAMETER_width_lane = 3.5
    TTF_PARAMETER_LENGTH = 5


    ## Multi lane threat

    if ds_x < 50 and ds_x > 0:
        nearest_point = ds_y + (math.sqrt((TTF_PARAMETER_width_ego / 2)^2 + (TTF_PARAMETER_LENGTH)^2)) * math.sin(math.atan((TTF_PARAMETER_width_ego / 2) / TTF_PARAMETER_LENGTH) + rel_yaw)
        if abs(nearest_point) < TTF_PARAMETER_width_lane / 2:
            TTF_tmp = 0
        else:
            TTF_tmp = (nearest_point - TTF_PARAMETER_width_lane / 2) / dv_y
    
        if abs(TTF_tmp) > 10:
            TTF_tmp = 10
        

        D_es = ds_x + dv_x * TTF_tmp 


        D_safe_multi = v_ego * TTF_PARAMETER_TAU + (v_ego^2) / (2 * TTF_PARAMETER_decel_ego) - (dv_x + v_ego)^2 / (2 * TTF_PARAMETER_decel_sur)

        Threat_index_multi = D_safe_multi / D_es

        if rel_yaw > 0:
            Threat_index_multi = 0
        
    else: 
        Threat_index_multi = 0
    
    return Threat_index_multi
     

