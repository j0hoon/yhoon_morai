def FVIThreatAss(Rear_Wheel_Velocity, FVI_Relative_Velocity, FVL_Clearance):
    
    v_ego = Rear_Wheel_Velocity
    dv_x = FVI_Relative_Velocity
    ds_x = FVL_Clearance
    
    TTF_PARAMETER_TAU = 0.7 + 0.15; # system react time 0.3~1.2s, brake react time 0.15s
    TTF_PARAMETER_Sr = 1
    TTF_PARAMETER_decel_ego = 6
    TTF_PARAMETER_decel_sur = 6
    TTF_PARAMETER_width_ego = 1.8
    TTF_PARAMETER_width_lane = 3.5
    TTF_PARAMETER_LENGTH = 5

    # single lane threat
    if ds_x < 50 and ds_x > 0:

        D_safe_single = v_ego*TTF_PARAMETER_TAU + (v_ego^2) / (2*TTF_PARAMETER_decel_ego) - (dv_x+v_ego^2) / (2*TTF_PARAMETER_decel_sur)
        Threat_index = D_safe_single / ds_x
    else: 
        Threat_index = 0
    
    return Threat_index

