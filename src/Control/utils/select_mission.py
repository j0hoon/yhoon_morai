# mission 1 : At stop line wait 5s -> go(only once)
# mission 2 : Rotary
# mission 3 : Traffic light signal recognize
# mission 4 : Static object --> lane change
# mission 5 : Dynamic objecy --> emergency stop -> go


def select_mission(WP_index,switch_WP): # specific range WP --> change road type

    if switch_WP==1 and WP_index==10: # mission 1
        current_mission=1
    elif (switch_WP==1 and WP_index>=30 and WP_index<=45) or (switch_WP==2 and WP_index>=30 and WP_index<=45):
        current_mission=2
    elif (switch_WP==1 and WP_index==60) or (switch_WP==2 and WP_index==50):
        current_mission=3
    elif (switch_WP==2 and WP_index>=100 and WP_index<=125) or (switch_WP==3 and WP_index>=40 and WP_index<=55):
        current_mission=4
    elif (switch_WP==2 and WP_index>=140 and WP_index<=155) or (switch_WP==3 and WP_index>=65 and WP_index<=80):
        current_mission=5
    else:
        current_mission=0

    return current_mission