#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

class pidControl:
    def __init__(self , p_gain, i_gain, d_gain, prev_error, i_control, controlTime):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.prev_error = prev_error
        self.i_control = i_control
        self.controlTime = controlTime

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        #TODO: (5) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output
    
    
    
        def get_output(self, target_value, current_value):
        error = target_value-current_value
        self.integral_error += error*self.sampling_time
        derivative_error = (error-self.previous_error)/self.sampling_time

        output = self.p_gain*error + self.i_gain*self.integral_error + self.d_gain*derivative_error
        self.previous_error = error
        return output