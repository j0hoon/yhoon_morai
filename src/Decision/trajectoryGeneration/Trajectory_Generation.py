import numpy as np


def Trajectory_Generation(Bezier_Point_Control,X_OBS,Y_OBS):
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