from typing import ForwardRef


def SurThreatAssessment(Ego_acc, Ego_vel, FVL_acc, FVL_rel_vel, FVL_rel_pos,FVR_acc, FVR_rel_vel, FVR_rel_pos, 
                        RVL_acc, RVL_rel_vel, RVL_rel_pos, RVR_acc, RVR_rel_vel, 
                        RVR_rel_pos,LANE_CHANGE_TIME,FOLLOWING_VEHICLE_MAX_DECEL, REACT_TIME, SAMPLE_TIME):
    # 시뮬 '0'으로 값을 받는 부분
    FVL_acc = 0
    FVR_acc = 0
    RVL_acc = 0
    RVR_acc = 0
    
    ##시뮬링크에서는 아래를 인풋으로 해서 변수 정의
#     Rear_Wheel_Velocity = Ego_vel
#     FVL_Relative_Velocity = FVL_rel_vel
#     FVL_Clearance = FVL_rel_pos
#     FVR_Relative_Velocity = FVR_rel_vel
#     FVR_Clearance = FVR_rel_pos
#     RVL_Relative_Velocity = RVL_rel_vel
#     RVL_Clearance = RVL_rel_pos
#     RVR_Relative_Velocity = RVR_rel_vel
    
#     Output %%%%%%%%%%%%%%%%%%%%%

# FVL_Risk : 전방 좌측 차량 위험도 0(safe) ~ 1(dangerous)
# FVR_Risk : 전방 우측 차량 위험도 0(safe) ~ 1(dangerous)
# RVL_Risk : 후방 좌측 차량 위험도 0(safe) ~ 1(dangerous)
# RVR_Risk : 후방 우측 차량 위험도 0(safe) ~ 1(dangerous)
    
    RVR_rel_pos = - RVR_rel_pos
    RVL_rel_pos = - RVL_rel_pos
    
    for local_time in range(0, LANE_CHANGE_TIME, 0.1):

        # FVL threat
        if FVL_rel_pos <= 0:
            FVL_Risk = 0
            FVL_MSS_min = 0
            FVL_MSS_safe = 0

        else:
            FVL_MSS_min = max(-(FVL_rel_vel)*local_time - 0.5 * (FVL_acc - Ego_acc) * local_time^2)
            FVL_MSS_safe = max(Ego_vel * local_time + 0.5 * Ego_acc*local_time^2)

            if FVL_MSS_safe < 35:
                FVL_MSS_safe = 35
            
            if FVL_rel_pos <= FVL_MSS_min:
                FVL_Risk = 1
            elif FVL_rel_pos > FVL_MSS_safe:
                FVL_Risk = 0
            else:
                FVL_Risk = 1 - (FVL_rel_pos - FVL_MSS_min) / (FVL_MSS_safe - FVL_MSS_min)

        # FVR threat
        if FVR_rel_pos <= 0:
            FVR_Risk = 0
            FVR_MSS_min = 0
            FVR_MSS_safe = 0

        else:
            FVR_MSS_min = max(-(FVR_rel_vel)*local_time - 0.5 * (FVR_acc - Ego_acc) * local_time^2)
            FVR_MSS_safe = max(Ego_vel * local_time + 0.5 * Ego_acc*local_time^2)

            if FVR_MSS_safe < 35:
                FVR_MSS_safe = 35
            
            if FVR_rel_pos <= FVR_MSS_min:
                FVR_Risk = 1
            elif FVR_rel_pos > FVR_MSS_safe:
                FVR_Risk = 0
            else:
                FVR_Risk = 1 - (FVR_rel_pos - FVR_MSS_min) / (FVR_MSS_safe - FVR_MSS_min)

        # RVL threat
        if RVL_rel_pos <= 0:
            RVL_Risk = 0
            RVL_MSS_min = 0
            RVL_MSS_safe = 0

        else:
            RVL_MSS_min = max(-(RVL_rel_vel)*local_time - 0.5 * (RVL_acc - Ego_acc) * local_time^2) + 3
            RVL_MSS_safe = RVL_MSS_min + REACT_TIME * (RVL_rel_vel + Ego_vel) - RVL_rel_vel^2 * LANE_CHANGE_TIME / (2 * FOLLOWING_VEHICLE_MAX_DECEL)

            if RVL_rel_pos <= RVL_MSS_min:
                RVL_Risk = 1
            elif RVL_rel_pos > RVL_MSS_safe:
                RVL_Risk = 0
            else:
                RVL_Risk = 1 - (RVL_rel_pos - RVL_MSS_min) / (RVL_MSS_safe - RVL_MSS_min)

        # RVR threat
        if RVR_rel_pos <= 0:
            RVR_Risk = 0
            RVR_MSS_min = 0
            RVR_MSS_safe = 0

        else:
            RVR_MSS_min = max(-(RVR_rel_vel)*local_time - 0.5 * (RVR_acc - Ego_acc) * local_time^2) + 3
            RVR_MSS_safe = RVR_MSS_min + REACT_TIME * (RVR_rel_vel + Ego_vel) - RVR_rel_vel^2 * LANE_CHANGE_TIME / (2 * FOLLOWING_VEHICLE_MAX_DECEL)

            if RVR_rel_pos <= RVR_MSS_min:
                RVR_Risk = 1
            elif RVR_rel_pos > RVR_MSS_safe:
                RVR_Risk = 0
            else:
                RVR_Risk = 1 - (RVR_rel_pos - RVR_MSS_min) / (RVR_MSS_safe - RVR_MSS_min)

    return FVL_Risk, FVR_Risk, RVL_Risk, RVR_Risk, FVL_MSS_min, FVL_rel_pos, FVL_MSS_safe, RVL_MSS_min, RVL_rel_pos, RVL_MSS_safe, FVR_MSS_min, FVR_rel_pos, FVR_MSS_safe , RVR_MSS_min, RVR_rel_pos, RVR_MSS_safe      

