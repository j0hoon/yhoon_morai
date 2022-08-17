# mission 1 : At stop line wait 5s -> go(only once)
# mission 2 : Rotary
# mission 3 : Traffic light signal recognize
# mission 4 : Static object --> lane change
# mission 5 : Dynamic objecy --> emergency stop -> go


def select_mission(WP_index,switch_WP): # specific range WP --> change road type

    if switch_WP==1 and WP_index ==15: # mission 1 init == 13
        current_mission=1
    elif (switch_WP==1 and WP_index>=35 and WP_index<=55) or (switch_WP==2 and WP_index>=34 and WP_index<=54): # init 29, 48 / 30 51 
        current_mission=2
    elif (switch_WP==1 and WP_index==78) or (switch_WP==2 and WP_index>=15 and WP_index<=16): #init 72 14
        current_mission=3
    elif (switch_WP==1 and WP_index>=75 and WP_index<=77) or (switch_WP==2 and WP_index>=12 and WP_index<=14): # for mission 3 dec
        current_mission=33    

    elif (switch_WP==2 and WP_index>=93 and WP_index<=114) or (switch_WP==3 and WP_index>=3 and WP_index<=25): # init 86 97 / 5 ??
        current_mission=4
    elif (switch_WP==3 and WP_index>=1 and WP_index<=2): # for mission 4 curve
        current_mission=44    
    
    elif (switch_WP==2 and WP_index>=161 and WP_index<=183) or (switch_WP==3 and WP_index>=74 and WP_index<=92): # init 151 164 / 75 85
        current_mission=5
    elif (switch_WP==1 and WP_index>=32 and WP_index<=34) or (switch_WP==2 and WP_index>=31 and WP_index<=33):
        current_mission=6 # mission 2-2
    elif (switch_WP==2 and WP_index>=116 and WP_index<=117) or (switch_WP==3 and WP_index>=27 and WP_index<=28):
        current_mission=99 # for 2nd lane back

    #elif target_spd = 9 for straight range
    
    else:
        current_mission=0

    return current_mission