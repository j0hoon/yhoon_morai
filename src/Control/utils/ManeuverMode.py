def ManeuverMode(self, risk, object_state, traffic_signal,mission): # + WP_index --> mode change

        # Traffic Light == 1 - red
        # Traffic Light == 4 - yellow
        # Traffic Light == 16 - green
        # Traffic Light == 33 - red and green(left)
        # Traffic Light == 5 - red and yellow

    # normal
    if mission == 0:
        maneuver_mode = 1

    # mission 1 : At stop line wait 5s -> go(only once)
    elif mission == 1:
        # delay 5 seconds
        maneuver_mode = 1

    # mission 2 : Rotary
    elif mission == 2:
        maneuver_mode = 1

    # mission 3 : Traffic light signal recognize
    elif mission == 3:
        if traffic_signal == 33:
            maneuver_mode = 1
        else:
            maneuver_mode = 4 

    # mission 4 : Static object --> lane change
    elif mission == 4:
        if risk == 1 and object_state == 0:
            maneuver_mode = 3 # lane change mode
        else:
            maneuver_mode = 1 

    # mission 5 : Dynamic object --> emergency stop -> go
    elif mission == 5:
        if risk == 1:
            maneuver_mode = 4 # stop mode
        else:
            maneuver_mode = 1
    
    return maneuver_mode