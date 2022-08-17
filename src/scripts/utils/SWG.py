import numpy as np
#import matplotlib.pyplot as plt
import math
#from utils.genWP import WP_X, WP_Y

class SWG_make():
    def __init__(self,WP_X,WP_Y):
        self.WP_X=WP_X
        self.WP_Y=WP_Y

    def WAYPOINT_INITIALIZING(self,WP_X,WP_Y,X_OBS,Y_OBS):

        wpnumber=len(self.WP_X) # length of WP_X = 193
        distance=np.zeros(wpnumber)
        for i in range(len(distance)):
            distance[i]=np.sqrt((self.WP_X[i]-X_OBS)**2+(self.WP_Y[i]-Y_OBS)**2)

        min_index=np.argmin(distance)
        index=min_index+1
        if index >= len(self.WP_X):
            index=1
        return index

    def trajectory_make(self,WP_X,WP_Y,Waypoint_Index,X_OBS,Y_OBS,Yaw_OBS):
        long1=X_OBS
        lat1=Y_OBS
        long2=WP_X[Waypoint_Index]
        lat2=WP_Y[Waypoint_Index]

        if Waypoint_Index<=len(WP_Y)-3:
            long3=WP_X[Waypoint_Index+1]
            lat3=WP_Y[Waypoint_Index+1]
            long4=WP_X[Waypoint_Index+2]
            lat4=WP_Y[Waypoint_Index+2]
        elif Waypoint_Index<=len(WP_Y)-2:
            long3=WP_X[Waypoint_Index+1]
            lat3=WP_Y[Waypoint_Index+1]
            long4=WP_X[0]
            lat4=WP_Y[0]
        else:
            long3=WP_X[0]
            lat3=WP_Y[0]
            long4=WP_X[1]
            lat4=WP_Y[1]
        dist_1=np.sqrt((lat2-lat1)**2+(long2-long1)**2)
        dist_2=np.sqrt((lat3-lat2)**2+(long3-long2)**2)
        dist_3=np.sqrt((lat4-lat3)**2+(long4-long3)**2)

        target_point_x=long2-long1
        target_point_y=lat2-lat1

        target_point2_x=long3-long1
        target_point2_y=lat3-lat1     

        target_point3_x=long4-long1
        target_point3_y=lat4-lat1

        P0=[0,0]
        P3=[target_point_x,target_point_y]
        P6=[target_point2_x,target_point2_y]

        if Waypoint_Index<=len(WP_Y)-2:
            yaw3=math.atan2(target_point2_y-target_point_y,target_point2_x-target_point_x)
        else:
            yaw3=math.atan2(target_point_y,target_point_x)
        yaw6=math.atan2(target_point3_y-target_point2_y,target_point3_x-target_point2_x)

        if Waypoint_Index<=1:
            yaw3=math.atan2(WP_Y[1]-WP_Y[0],WP_X[1]-WP_X[0])

        yaw0=Yaw_OBS
        distFactor=0.25

        P1=[P0[0]+dist_1*distFactor*np.cos(yaw0),P0[1]+dist_1*distFactor*np.sin(yaw0)]
        P2=[P3[0]-dist_1*distFactor*np.cos(yaw3),P3[1]-dist_1*distFactor*np.sin(yaw3)]

        P4=[P3[0]+dist_2*distFactor*np.cos(yaw3),P3[1]+dist_2*distFactor*np.sin(yaw3)]
        P5=[P6[0]-dist_2*distFactor*np.cos(yaw6),P6[1]-dist_2*distFactor*np.sin(yaw6)]

        CP1_X=P1[0]
        CP1_Y=P1[1]
        CP2_X=P2[0]
        CP2_Y=P2[1]
        CP4_X=P4[0]
        CP4_Y=P4[1]
        CP5_X=P5[0]
        CP5_Y=P5[1]
        Bezier_Point_Control=[P0[0],P0[1],CP1_X[0],CP1_Y[0],CP2_X[0],CP2_Y[0],P3[0][0],P3[1][0],CP4_X[0],CP4_Y[0],CP5_X[0],CP5_Y[0],P6[0][0],P6[1][0]]        
        P0_X=Bezier_Point_Control[0]
        P0_Y=Bezier_Point_Control[1]
        CP1_X=Bezier_Point_Control[2]
        CP1_Y=Bezier_Point_Control[3]
        CP2_X=Bezier_Point_Control[4]
        CP2_Y=Bezier_Point_Control[5]
        P3_X=Bezier_Point_Control[6]
        P3_Y=Bezier_Point_Control[7]
        CP4_X=Bezier_Point_Control[8]
        CP4_Y=Bezier_Point_Control[9]
        CP5_X=Bezier_Point_Control[10]
        CP5_Y=Bezier_Point_Control[11]
        P6_X=Bezier_Point_Control[12]
        P6_Y=Bezier_Point_Control[13]    
        P0 = [P0_X, P0_Y]    # start position
        P1 = [CP1_X, CP1_Y]
        P2 = [CP2_X, CP2_Y]
        P3 = [P3_X, P3_Y]    # end position
        P4 = [CP4_X, CP4_Y]
        P5 = [CP5_X, CP5_Y]
        P6 = [P6_X, P6_Y]    # end position
        a_1=np.zeros(2)
        b_1=np.zeros(2)
        c_1=np.zeros(2)
        a_2=np.zeros(2)
        b_2=np.zeros(2)
        c_2=np.zeros(2)

        for i in range(2):   
            c_1[i]=3*(P1[i]-P0[i])
            b_1[i]=3*(P2[i]-P1[i])-c_1[i]
            a_1[i]=P3[i]-P0[i]-b_1[i]-c_1[i]
            c_2[i]=3*(P4[i]-P3[i])
            b_2[i]=3*(P5[i]-P4[i])-c_2[i]
            a_2[i]=P6[i]-P3[i]-b_2[i]-c_2[i]

        t_1=np.linspace(0,1,51)
        Trajectory=np.zeros(len(t_1)*4).reshape(len(t_1)*2,2)
        
        for i in range(0,51):
            for j in range(0,2):
                Trajectory[i,j]=a_1[j]*(t_1[i]**3)+b_1[j]*(t_1[i]**2)+c_1[j]*t_1[i]+P0[j]    

        t_2=np.linspace(0,1,51)
        for i in range(0,51):
            for j in range(0,2):
                    Trajectory[i+51,j]=a_2[j]*(t_2[i]**3)+b_2[j]*(t_2[i]**2)+c_2[j]*t_2[i]+P3[j] 

        Trajectory[:,0]=Trajectory[:,0]+X_OBS  
        Trajectory[:,1]=Trajectory[:,1]+Y_OBS 
        Trajectory_X = Trajectory[1:,0]        
        Trajectory_Y = Trajectory[1:,1] 
        Trajectory_X=Trajectory_X.tolist()
        Trajectory_Y=Trajectory_Y.tolist()
        

        return Trajectory_X, Trajectory_Y





