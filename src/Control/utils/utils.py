
import numpy as np


class Purepursuit:
    def __init__(self, LW, K):

        self.wheelbase = LW
        self.preview_gain = K

    def pure_pursuit(self, ref_x, ref_y, V_x, ego_x, ego_y, yaw):
        # yaw: (rad/s)
        #


        Lp = self.preview_gain * V_x
        Lp_x = ego_x + Lp * np.cos(yaw)
        Lp_y = ego_y + Lp * np.sin(yaw)

        dis_P2 = np.sqrt((np.array(ref_x) - Lp_x)**2+(np.array(ref_y) - Lp_y)**2)

        min_index = np.argmin(dis_P2)

        Way_x = ref_x[min_index]
        Way_y = ref_y[min_index]

        x_2 =   (Way_x - ego_x) * np.cos(yaw) + (Way_y - ego_y) * np.sin(yaw)
        y_2 = - (Way_x - ego_x) * np.sin(yaw) + (Way_y - ego_y) * np.cos(yaw)

        L_bar = np.sqrt(x_2**2 +y_2**2)

        sin_alpha = y_2/L_bar

        self.k = 2 * sin_alpha/L_bar
        steer_angle = self.wheelbase * 2 * y_2 / (L_bar)**2

        return steer_angle, min_index


class pidController : 
    def __init__(self):
        self.p_gain= 0.5
        self.i_gain= 0.0
        self.d_gain= 0.1

        self.controlTime=0.033
        self.prev_error=0
        self.i_control=0

    def pid(self,target_vel,current_vel):
        
        error= target_vel-current_vel
        
        p_control=self.p_gain*error
        self.i_control+=self.i_gain*error*self.controlTime
        d_control=self.d_gain*(error-self.prev_error)/self.controlTime

        output=p_control+self.i_control+d_control
        self.prev_error=error
        
        return output

class ACC:
    def __init__(self, Clearance):

        self.clearance = Clearance   ##### clearance meter

    def acc(self, Ego_vehicle_speed, relative_velocity):

        if Ego_vehicle_speed > 40:
            k1 = 0.35
            k2 = -1.2

        elif Ego_vehicle_speed < 70 and Ego_vehicle_speed >= 40:
            k1 = 0.35 - 0.005 * Ego_vehicle_speed
            k2 = -1.2 + 0.01 * Ego_vehicle_speed

        else:
            k1 = 0.2
            k2 = -0.9

        c_desired = 1.98 + 1.36 * Ego_vehicle_speed

        clearance_error = c_desired - self.clearance
        a_desired = - k1 * Ego_vehicle_speed * clearance_error - \
                      k2 * Ego_vehicle_speed * relative_velocity

        return a_desired 

