from lib2to3.pgen2.token import EQUAL
from turtle import end_fill
import numpy as np
import math
import matplotlib.pyplot as plt



x_ego = 0
y_ego = 0
Yaw_angle = math.atan2((407.1184-393.24),(-466.276-(456.26)))
refPoses = np.zeros(100,)
Waypoint_Index = np.zeros(len(refPoses),)

P0_X  =  407.184     #x_ego
P0_Y  = -466.276     #y_ego
P3_Y  = -480.737     #두번째 waypoint의 y좌표    사용방법 :  P3_Y = WP_Y(Waypoint_Index+1,1)
P3_X  = 407.574      #두번째 waypoint의 x좌표    사용방법 :  P3_X = WP_X(Waypoint_Index+1,1)
P6_Y  = -487.97      #세번째 waypoint의 y좌표    사용방법 :  P6_Y = WP_Y(Waypoint_Index+2,1)
P6_X  = 402.59       #세번째 waypoint의 x좌표    사용방법 :  P6_X = WP_X(Waypoint_Index+2,1)

    #Function : 경유점과 제어점을 생성하기 위한  코드
dist_1 = math.sqrt( ( P3_Y - P0_Y )**2 + ( P3_X - P0_X )**2)     # 자차와 첫번째 waypoint와의 거리
dist_2 = math.sqrt( ( P6_Y - P3_Y )**2 + ( P6_X - P3_X )**2)     # 두번째 waypoint와 첫번째 waypoint와의 거리
   

#     P3_Y = (P3_Y-P0_Y);      #자차 기준 첫번째 Waypoint의 y좌표
#     P3_X = (P3_X-P0_X);      #자차 기준 첫번째 Waypoint의 x좌표
#    
#     P6_Y = (P6_Y-P0_Y);      #자차 기준 두번째 Waypoint의 y좌표
#     P6_X = (P6_X-P0_X);      #자차 기준 두번째 Waypoint의 x좌표
   
     
P0=[P0_X ,P0_Y]        # start position
P3=[P3_X ,P3_Y]          # 첫번째 Waypoint
P6=[P6_X ,P6_Y]          # 두번째 Waypoint
   
distFactor = 0.375                        #경유점과 경유점 사이의 제어점을 만들기 위한 Parameter of control points
yaw6=math.atan2(P6_Y-P3_Y,P6_X-P3_X)
yaw3=math.atan2(P3_Y-P0_Y,P3_X-P0_X)  
yaw0=Yaw_angle*math.pi/180                                                      
   
   
P1=[P0[0]+dist_1*distFactor*math.cos(yaw0), P0[1]+dist_1*distFactor*math.sin(yaw0)]   # Control Point1
P2=[P3[0]-dist_1*distFactor*math.cos(yaw3), P3[1]-dist_1*distFactor*math.sin(yaw3)]   # Control Point2
   
P4=[P3[0]+dist_2*distFactor*math.cos(yaw3), P3[1]+dist_2*distFactor*math.sin(yaw3)]       # Control Point3
P5=[P6[0]-dist_2*distFactor*math.cos(yaw6), P6[1]-dist_2*distFactor*math.sin(yaw6)]       # Control Point4


P0_X    = P0[0]            #x_ego
P0_Y    = P0[1]            #y_ego          
CP1_X   = P1[0]            #첫번째 제어점 x좌표
CP1_Y   = P1[1]            #첫번째 제어점 y좌표
CP2_X   = P2[0]            #두번째 제어점 x좌표
CP2_Y   = P2[1]            #두번째 제어점 y좌표
P3_X    = P3[0]            #첫번째 경유점 x좌표
P3_Y    = P3[1]            #첫번째 경유점 y좌표  
CP4_X   = P4[0]            #세번째 제어점 x좌표
CP4_Y   = P4[1]            #세번째 제어점 y좌표
CP5_X   = P5[0]            #네번째 제어점 x좌표
CP5_Y   = P5[1]            #네번째 제어점 y좌표
P6_X    = P6[0]            #두번째 경유점 x좌표
P6_Y    = P6[1]            #두번째 경유점 y좌표
   
#Bezier_Point_Control = [P0_X, P0_Y, CP1_X, CP1_Y, CP2_X, CP2_Y, P3_X, P3_Y, CP4_X, CP4_Y, CP5_X, CP5_Y, P6_X, P6_Y ] 
    # end            function사용시 주석 해제

 #따라서 경유점과 제어점을 그려보면 다음과 같다.

#Plot
plt.figure(figsize=(8,8))
plt.plot(P0_X,P0_Y,'bo' )
plt.text(P0_X,P0_Y,'P_0')
plt.plot(CP1_X,CP1_Y,'ro')
plt.text(CP1_X,CP1_Y,'CP_1')
plt.plot(CP2_X,CP2_Y,'ro')
plt.text(CP2_X,CP2_Y,'CP_2')
plt.plot(P3_X, P3_Y,'bo')
plt.text(P3_X,P3_Y,'P_3')
plt.plot(CP4_X,CP4_Y,'ro')
plt.text(CP4_X,CP4_Y,'CP_3')
plt.plot(CP5_X,CP5_Y,'ro')
plt.text(CP5_X,CP5_Y,'CP_4')
plt.plot(P6_X, P6_Y,'bo')
plt.text(P6_X,P6_Y,'P_6')
plt.grid
plt.legend(('P0', 'CP1', 'CP2', 'P3','CP3','CP4','P6','Location','Northwest','NumColumns'))
    #axis equal
#     xlim([-1 15])
#     ylim([-1 5])
plt.axis()
plt.title('Waypoint & Control point')
plt.show()



############################################################################################
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

 
Trajectory_X = Trajectory[:,0]        #궤적의 x좌표
Trajectory_Y = Trajectory[:,1]         #궤적의 y좌표
        
plt.figure(figsize=(8,8))

plt.plot(Trajectory_X,Trajectory_Y,'b')

plt.plot(P0_X,P0_Y,'bo')
plt.text(P0_X,P0_Y,'P_0')
plt.plot(CP1_X,CP1_Y,'ro')
plt.text(CP1_X,CP1_Y,'CP_1')
plt.plot(CP2_X,CP2_Y,'ro')
plt.text(CP2_X,CP2_Y,'CP_2')
plt.plot(P3_X, P3_Y,'bo')
plt.text(P3_X,P3_Y,'P_3')
plt.plot(CP4_X,CP4_Y,'ro')
plt.text(CP4_X,CP4_Y,'CP_3')
plt.plot(CP5_X,CP5_Y,'ro')
plt.text(CP5_X,CP5_Y,'CP_4')
plt.plot(P6_X, P6_Y,'bo')
plt.text(P6_X,P6_Y,'P_6')
plt.grid()
plt.legend(loc='best')
plt.title('Distance factor 0.375')
plt.show()
