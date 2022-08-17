import numpy as np
import matplotlib.pyplot as plt
import os

DISTANCE_WP2WP_STRAIGHT=0.4 # init=0.8
DISTANCE_WP2WP_CURVE=0.2 # init=0.3
plot_switch=1

# 1st WP

load_data=np.loadtxt('/home/aclsim2/catkin_ws/src/Morai_KM/scripts/path_1.csv',delimiter=',',dtype=str) # Input GPS csv file

longitude=load_data[1:,[0]] # x
latitude=load_data[1:,[1]] # y

longitude=longitude.reshape(len(longitude),1)
latitude=latitude.reshape(len(longitude),1)

longitude=longitude.astype(np.float)
latitude=latitude.astype(np.float)

DESLONG_tmp=np.zeros(len(longitude)).reshape(len(longitude),1)
DESLAT_tmp=np.zeros(len(latitude)).reshape(len(longitude),1)

DESLONG_tmp[0] = longitude[0]
DESLAT_tmp[0] = latitude[0]

a=0
b=2
number_tmp=[1]

for i in range(0,len(longitude)):
    lat1=latitude[a] 
    long1=longitude[a]
    lat2=latitude[i] 
    long2=longitude[i]
    dist=np.sqrt(((lat1-lat2)**2)+((long1-long2)**2))

    if dist >=0.01: 
        DESLONG_tmp[i] = longitude[i]
        DESLAT_tmp[i] = latitude[i]
        a=i

        number_tmp.append(i+1)

index=[]
AA=np.zeros(len(DESLAT_tmp)).reshape(len(DESLAT_tmp),1)
for i in range(len(AA)):
    if DESLAT_tmp[i]==0:
        index.append(i)

DESLONG=np.delete(DESLONG_tmp,index)
DESLAT=np.delete(DESLAT_tmp,index)
DESLONG_tmp=np.delete(DESLONG_tmp,index)
DESLAT_tmp=np.delete(DESLAT_tmp,index)

WP_X_1=DESLONG.reshape(len(DESLONG))
WP_Y_1=DESLAT.reshape(len(DESLAT))

CURVATURE_STRAIGHT=1/1000000
CURVATURE_CURVE=0.00005

x=DESLONG 
y=DESLAT 

WP_Curvature=np.zeros(len(DESLAT)).reshape(len(DESLAT),1) # 193x1
for i in range(1,len(DESLAT)+1):
    if i<=len(DESLAT_tmp)-2:
        x1=DESLONG_tmp[i-1]
        x2=DESLONG_tmp[i]
        x3=DESLONG_tmp[i+1]

        y1=DESLAT_tmp[i-1]
        y2=DESLAT_tmp[i]
        y3=DESLAT_tmp[i+1]
    elif i==len(DESLAT_tmp)-1:
        x1=DESLONG_tmp[i-2]
        x2=DESLONG_tmp[i-1]
        x3=DESLONG_tmp[i]

        y1=DESLAT_tmp[i-2]
        y2=DESLAT_tmp[i-1]
        y3=DESLAT_tmp[i]
    else:
        x1=DESLONG_tmp[i-3]
        x2=DESLONG_tmp[i-2]
        x3=DESLONG_tmp[i-1]

        y1=DESLAT_tmp[i-3]
        y2=DESLAT_tmp[i-2]
        y3=DESLAT_tmp[i-1]        
    rd_1=(y2-y1)/(x2-x1)
    rd_2=(y3-y2)/(x3-x2)
    d_1=1/rd_1
    d_2=1/rd_2

    x=((y3-y1)+(x2+x3)*d_2-(x1+x2)*d_1)/(2*(d_2-d_1))
    y=-d_1*(x-(x1+x2)/2)+(y1+y2)/2

    r=np.sqrt((x1-x)**2+(y1-y)**2)
    WP_Curvature[i-1]=1/r

    if((x1==x2 and x2==x3)or(y1==y2 and y2==y3)):
        WP_Curvature[i-1]=CURVATURE_STRAIGHT

WP_X_1=np.zeros(len(DESLAT_tmp)).reshape(len(DESLAT_tmp),1) # 193x1
WP_Y_1=np.zeros(len(DESLAT_tmp)).reshape(len(DESLAT_tmp),1)

WP_Curvature_index_straight=[] 
WP_Curvature_index_curve=[]

for i in range(len(WP_Curvature)):
    if(WP_Curvature[i]<CURVATURE_CURVE):
        WP_Curvature_index_straight.append(i+1) # list 97
    else:
        WP_Curvature_index_curve.append(i+1) # list 96

tmp_start_index = 1
tmp_end_index = 1

tmp_start_end_info_straight = np.zeros(len(WP_Curvature)*2).reshape(len(WP_Curvature),2)
tmp_start_end_info_curve = np.zeros(len(WP_Curvature)*2).reshape(len(WP_Curvature),2)

row_index = 0 

# WP_Curvature_index_straight
if (len(WP_Curvature_index_straight)!=0):
    for i in range(2,len(WP_Curvature_index_straight)+1): # i=2~97
        if ((WP_Curvature_index_straight[i-1] - WP_Curvature_index_straight[i-2])>1):

            tmp_end_index=i-1
            tmp_start_end_info_straight[row_index,:] = [tmp_start_index,tmp_end_index] # [86. 87.]

            if (i!=len(WP_Curvature_index_straight)): # 97
                tmp_start_index=i
                row_index+=1
            else:
                tmp_start_index=0
                row_index=0
        else:
            if (i==len(WP_Curvature_index_straight) and tmp_start_index!=0):
                tmp_start_end_info_straight[row_index,0] = tmp_start_index
                tmp_start_end_info_straight[row_index,1] = i   

    for i in range(len(tmp_start_end_info_straight)):
        if (tmp_start_end_info_straight[i,0]==0):
            idx=i
            break
    start_end_info_straight=np.zeros(idx*2).reshape(idx,2)    
    for i in range(idx):
        start_end_info_straight[i,0]=tmp_start_end_info_straight[i,0]
        start_end_info_straight[i,1]=tmp_start_end_info_straight[i,1]

    for i in range(len(start_end_info_straight)):
        i_st_range=np.arange(start_end_info_straight[i,0],(start_end_info_straight[i,1]+1),1)
        start_index=int(start_end_info_straight[i,0])
        
        tmp_WP_first_index=WP_Curvature_index_straight[int(i_st_range[0])-1]
        tmp_WP_end_index=WP_Curvature_index_straight[int(i_st_range[-1])-1]
        

        WP_X_1[tmp_WP_first_index-1]=DESLONG_tmp[tmp_WP_first_index-1]
        WP_X_1[tmp_WP_end_index-1]=DESLONG_tmp[tmp_WP_end_index-1]

        WP_Y_1[tmp_WP_first_index-1]=DESLAT_tmp[tmp_WP_first_index-1]
        WP_Y_1[tmp_WP_end_index-1]=DESLAT_tmp[tmp_WP_end_index-1]
   
        for j in range(int(i_st_range[0]),int(i_st_range[-1]+1)): 
            tmp_WP_index=WP_Curvature_index_straight[j-1] 

            WP_X_start=DESLONG_tmp[start_index-1] 
            WP_Y_start=DESLAT_tmp[start_index-1]

            WP_X_end=DESLONG_tmp[tmp_WP_index-1]
            WP_Y_end=DESLAT_tmp[tmp_WP_index-1]

            tmp_distance=np.sqrt(((WP_X_start-WP_X_end)**2)+((WP_Y_start-WP_Y_end)**2))

            if (tmp_distance >= DISTANCE_WP2WP_STRAIGHT):
                WP_X_1[tmp_WP_index-1]=WP_X_end
                WP_Y_1[tmp_WP_index-1]=WP_Y_end

                start_index=tmp_WP_index

tmp_start_index=1
row_index=0

# WP_Curvature_index_curve
if (len(WP_Curvature_index_curve)!=0):
    for i in range(2,len(WP_Curvature_index_curve)+1): # 2~97
        if ((WP_Curvature_index_curve[i-1] - WP_Curvature_index_curve[i-2])>1):
            tmp_end_index=i-1
            tmp_start_end_info_curve[row_index,:] = [tmp_start_index,tmp_end_index]

            if (i!=len(WP_Curvature_index_curve)): # 120
                tmp_start_index=i
                row_index+=1
            else:
                tmp_start_index=0
                row_index=0
        else:
            if (i==len(WP_Curvature_index_curve) and tmp_start_index!=0):
                tmp_start_end_info_curve[row_index,0] = tmp_start_index
                tmp_start_end_info_curve[row_index,1] = i   

    for i in range(len(tmp_start_end_info_curve)): # 120
        if (tmp_start_end_info_curve[i,0]==0):
            idx=i
            break
    start_end_info_curve=np.zeros(idx*2).reshape(idx,2)    
    for i in range(idx): # idx=36
        start_end_info_curve[i,0]=tmp_start_end_info_curve[i,0]
        start_end_info_curve[i,1]=tmp_start_end_info_curve[i,1]

    for i in range(len(start_end_info_curve)):
        i_cv_range=np.arange(start_end_info_curve[i,0],start_end_info_curve[i,1]+1,1)
        start_index=int(start_end_info_curve[i,0])

        tmp_WP_first_index=WP_Curvature_index_curve[int(i_cv_range[0])-1]
        tmp_WP_end_index=WP_Curvature_index_curve[int(i_cv_range[-1])-1]

        WP_X_1[tmp_WP_first_index-1]=DESLONG_tmp[tmp_WP_first_index-1]
        WP_X_1[tmp_WP_end_index-1]=DESLONG_tmp[tmp_WP_end_index-1]

        WP_Y_1[tmp_WP_first_index-1]=DESLAT_tmp[tmp_WP_first_index-1]
        WP_Y_1[tmp_WP_end_index-1]=DESLAT_tmp[tmp_WP_end_index-1]

        for j in range(int(i_cv_range[0]),int(i_cv_range[-1]+1)):
            tmp_WP_index=WP_Curvature_index_curve[j-1]

            WP_X_start=DESLONG_tmp[start_index-1]
            WP_Y_start=DESLAT_tmp[start_index-1]

            WP_X_end=DESLONG_tmp[tmp_WP_index-1]
            WP_Y_end=DESLAT_tmp[tmp_WP_index-1]

            tmp_distance=np.sqrt(((WP_X_start-WP_X_end)**2)+((WP_Y_start-WP_Y_end)**2))

            if (tmp_distance >= DISTANCE_WP2WP_CURVE):
                WP_X_1[tmp_WP_index-1]=WP_X_end
                WP_Y_1[tmp_WP_index-1]=WP_Y_end

                start_index=tmp_WP_index

del_indexes=[]
for i in range(len(WP_X_1)):
    if (WP_X_1[i]==0):
        del_indexes.append(i)
WP_X_1=np.delete(WP_X_1,del_indexes)
WP_Y_1=np.delete(WP_Y_1,del_indexes)
WP_X_1=WP_X_1.reshape(len(WP_X_1),1)
WP_Y_1=WP_Y_1.reshape(len(WP_Y_1),1)
WP=np.zeros(len(WP_X_1)*2)
WP=WP.reshape(len(WP_X_1),2)

for i in range(len(WP_X_1)):
    WP[i,0]=WP_X_1[i]
    WP[i,1]=WP_Y_1[i]


# 2nd WP

load_data_2=np.loadtxt('/home/aclsim2/catkin_ws/src/Morai_KM/scripts/path_2.csv',delimiter=',',dtype=str) # Input GPS csv file

longitude_2=load_data_2[1:,[0]] # x
latitude_2=load_data_2[1:,[1]] # y

longitude_2=longitude_2.reshape(len(longitude_2),1)
latitude_2=latitude_2.reshape(len(longitude_2),1)

longitude_2=longitude_2.astype(np.float)
latitude_2=latitude_2.astype(np.float)

DESLONG_tmp_2=np.zeros(len(longitude_2)).reshape(len(longitude_2),1)
DESLAT_tmp_2=np.zeros(len(latitude_2)).reshape(len(longitude_2),1)

DESLONG_tmp_2[0] = longitude_2[0]
DESLAT_tmp_2[0] = latitude_2[0]

a_2=0
b_2=2
number_tmp_2=[1]

for i in range(0,len(longitude_2)):
    lat1=latitude_2[a_2] 
    long1=longitude_2[a_2]
    lat2=latitude_2[i] 
    long2=longitude_2[i]
    dist=np.sqrt(((lat1-lat2)**2)+((long1-long2)**2))

    if dist >=0.01: 
        DESLONG_tmp_2[i] = longitude_2[i]
        DESLAT_tmp_2[i] = latitude_2[i]
        a_2=i

        number_tmp_2.append(i+1)

index_2=[]
AA_2=np.zeros(len(DESLAT_tmp_2)).reshape(len(DESLAT_tmp_2),1)
for i in range(len(AA_2)):
    if DESLAT_tmp_2[i]==0:
        index_2.append(i)

DESLONG_2=np.delete(DESLONG_tmp_2,index_2)
DESLAT_2=np.delete(DESLAT_tmp_2,index_2)
DESLONG_tmp_2=np.delete(DESLONG_tmp_2,index_2)
DESLAT_tmp_2=np.delete(DESLAT_tmp_2,index_2)

WP_X_1_2=DESLONG_2.reshape(len(DESLONG_2))
WP_Y_1_2=DESLAT_2.reshape(len(DESLAT_2))

x=DESLONG_2 
y=DESLAT_2 

WP_Curvature_2=np.zeros(len(DESLAT_2)).reshape(len(DESLAT_2),1) # 193x1
for i in range(1,len(DESLAT_2)+1):
    if i<=len(DESLAT_tmp_2)-2:
        x1=DESLONG_tmp_2[i-1]
        x2=DESLONG_tmp_2[i]
        x3=DESLONG_tmp_2[i+1]

        y1=DESLAT_tmp_2[i-1]
        y2=DESLAT_tmp_2[i]
        y3=DESLAT_tmp_2[i+1]
    elif i==len(DESLAT_tmp_2)-1:
        x1=DESLONG_tmp_2[i-2]
        x2=DESLONG_tmp_2[i-1]
        x3=DESLONG_tmp_2[i]

        y1=DESLAT_tmp_2[i-2]
        y2=DESLAT_tmp_2[i-1]
        y3=DESLAT_tmp_2[i]
    else:
        x1=DESLONG_tmp_2[i-3]
        x2=DESLONG_tmp_2[i-2]
        x3=DESLONG_tmp_2[i-1]

        y1=DESLAT_tmp_2[i-3]
        y2=DESLAT_tmp_2[i-2]
        y3=DESLAT_tmp_2[i-1]        
    rd_1=(y2-y1)/(x2-x1)
    rd_2=(y3-y2)/(x3-x2)
    d_1=1/rd_1
    d_2=1/rd_2

    x=((y3-y1)+(x2+x3)*d_2-(x1+x2)*d_1)/(2*(d_2-d_1))
    y=-d_1*(x-(x1+x2)/2)+(y1+y2)/2

    r=np.sqrt((x1-x)**2+(y1-y)**2)
    WP_Curvature_2[i-1]=1/r

    if((x1==x2 and x2==x3)or(y1==y2 and y2==y3)):
        WP_Curvature_2[i-1]=CURVATURE_STRAIGHT

WP_X_2=np.zeros(len(DESLAT_tmp_2)).reshape(len(DESLAT_tmp_2),1) # 193x1
WP_Y_2=np.zeros(len(DESLAT_tmp_2)).reshape(len(DESLAT_tmp_2),1)

WP_Curvature_index_straight_2=[] 
WP_Curvature_index_curve_2=[]

for i in range(len(WP_Curvature_2)):
    if(WP_Curvature_2[i]<CURVATURE_CURVE):
        WP_Curvature_index_straight_2.append(i+1) # list 97
    else:
        WP_Curvature_index_curve_2.append(i+1) # list 96

tmp_start_index = 1
tmp_end_index = 1

tmp_start_end_info_straight_2 = np.zeros(len(WP_Curvature_2)*2).reshape(len(WP_Curvature_2),2)
tmp_start_end_info_curve_2 = np.zeros(len(WP_Curvature_2)*2).reshape(len(WP_Curvature_2),2)

row_index = 0 

# WP_Curvature_index_straight_2
if (len(WP_Curvature_index_straight_2)!=0):
    for i in range(2,len(WP_Curvature_index_straight_2)+1): # i=2~97
        if ((WP_Curvature_index_straight_2[i-1] - WP_Curvature_index_straight_2[i-2])>1):

            tmp_end_index=i-1
            tmp_start_end_info_straight_2[row_index,:] = [tmp_start_index,tmp_end_index] # [86. 87.]

            if (i!=len(WP_Curvature_index_straight_2)): # 97
                tmp_start_index=i
                row_index+=1
            else:
                tmp_start_index=0
                row_index=0
        else:
            if (i==len(WP_Curvature_index_straight_2) and tmp_start_index!=0):
                tmp_start_end_info_straight_2[row_index,0] = tmp_start_index
                tmp_start_end_info_straight_2[row_index,1] = i   

    for i in range(len(tmp_start_end_info_straight_2)):
        if (tmp_start_end_info_straight_2[i,0]==0):
            idx=i
            break
    start_end_info_straight_2=np.zeros(idx*2).reshape(idx,2)    
    for i in range(idx):
        start_end_info_straight_2[i,0]=tmp_start_end_info_straight_2[i,0]
        start_end_info_straight_2[i,1]=tmp_start_end_info_straight_2[i,1]

    for i in range(len(start_end_info_straight_2)):
        i_st_range=np.arange(start_end_info_straight_2[i,0],(start_end_info_straight_2[i,1]+1),1)
        start_index=int(start_end_info_straight_2[i,0])
        
        tmp_WP_first_index_2=WP_Curvature_index_straight_2[int(i_st_range[0])-1]
        tmp_WP_end_index_2=WP_Curvature_index_straight_2[int(i_st_range[-1])-1]
        

        WP_X_2[tmp_WP_first_index_2-1]=DESLONG_tmp_2[tmp_WP_first_index_2-1]
        WP_X_2[tmp_WP_end_index_2-1]=DESLONG_tmp_2[tmp_WP_end_index_2-1]

        WP_Y_2[tmp_WP_first_index_2-1]=DESLAT_tmp_2[tmp_WP_first_index_2-1]
        WP_Y_2[tmp_WP_end_index_2-1]=DESLAT_tmp_2[tmp_WP_end_index_2-1]
   
        for j in range(int(i_st_range[0]),int(i_st_range[-1]+1)): 
            tmp_WP_index=WP_Curvature_index_straight_2[j-1] 

            WP_X_start=DESLONG_tmp_2[start_index-1] 
            WP_Y_start=DESLAT_tmp_2[start_index-1]

            WP_X_end=DESLONG_tmp_2[tmp_WP_index-1]
            WP_Y_end=DESLAT_tmp_2[tmp_WP_index-1]

            tmp_distance=np.sqrt(((WP_X_start-WP_X_end)**2)+((WP_Y_start-WP_Y_end)**2))

            if (tmp_distance >= DISTANCE_WP2WP_STRAIGHT):
                WP_X_2[tmp_WP_index-1]=WP_X_end
                WP_Y_2[tmp_WP_index-1]=WP_Y_end

                start_index=tmp_WP_index

tmp_start_index=1
row_index=0

# WP_Curvature_index_curve_2
if (len(WP_Curvature_index_curve_2)!=0):
    for i in range(2,len(WP_Curvature_index_curve_2)+1): # 2~97
        if ((WP_Curvature_index_curve_2[i-1] - WP_Curvature_index_curve_2[i-2])>1):
            tmp_end_index=i-1
            tmp_start_end_info_curve_2[row_index,:] = [tmp_start_index,tmp_end_index]

            if (i!=len(WP_Curvature_index_curve_2)): # 120
                tmp_start_index=i
                row_index+=1
            else:
                tmp_start_index=0
                row_index=0
        else:
            if (i==len(WP_Curvature_index_curve_2) and tmp_start_index!=0):
                tmp_start_end_info_curve_2[row_index,0] = tmp_start_index
                tmp_start_end_info_curve_2[row_index,1] = i   

    for i in range(len(tmp_start_end_info_curve_2)): # 120
        if (tmp_start_end_info_curve_2[i,0]==0):
            idx=i
            break
    start_end_info_curve_2=np.zeros(idx*2).reshape(idx,2)    
    for i in range(idx): # idx=36
        start_end_info_curve_2[i,0]=tmp_start_end_info_curve_2[i,0]
        start_end_info_curve_2[i,1]=tmp_start_end_info_curve_2[i,1]

    for i in range(len(start_end_info_curve_2)):
        i_cv_range=np.arange(start_end_info_curve_2[i,0],start_end_info_curve_2[i,1]+1,1)
        start_index=int(start_end_info_curve_2[i,0])

        tmp_WP_first_index_2=WP_Curvature_index_curve_2[int(i_cv_range[0])-1]
        tmp_WP_end_index_2=WP_Curvature_index_curve_2[int(i_cv_range[-1])-1]

        WP_X_2[tmp_WP_first_index_2-1]=DESLONG_tmp_2[tmp_WP_first_index_2-1]
        WP_X_2[tmp_WP_end_index_2-1]=DESLONG_tmp_2[tmp_WP_end_index_2-1]

        WP_Y_2[tmp_WP_first_index_2-1]=DESLAT_tmp_2[tmp_WP_first_index_2-1]
        WP_Y_2[tmp_WP_end_index_2-1]=DESLAT_tmp_2[tmp_WP_end_index_2-1]

        for j in range(int(i_cv_range[0]),int(i_cv_range[-1]+1)):
            tmp_WP_index=WP_Curvature_index_curve_2[j-1]

            WP_X_start=DESLONG_tmp_2[start_index-1]
            WP_Y_start=DESLAT_tmp_2[start_index-1]

            WP_X_end=DESLONG_tmp_2[tmp_WP_index-1]
            WP_Y_end=DESLAT_tmp_2[tmp_WP_index-1]

            tmp_distance=np.sqrt(((WP_X_start-WP_X_end)**2)+((WP_Y_start-WP_Y_end)**2))

            if (tmp_distance >= DISTANCE_WP2WP_CURVE):
                WP_X_2[tmp_WP_index-1]=WP_X_end
                WP_Y_2[tmp_WP_index-1]=WP_Y_end

                start_index=tmp_WP_index

del_indexes=[]
for i in range(len(WP_X_2)):
    if (WP_X_2[i]==0):
        del_indexes.append(i)
WP_X_2=np.delete(WP_X_2,del_indexes)
WP_Y_2=np.delete(WP_Y_2,del_indexes)
WP_X_2=WP_X_2.reshape(len(WP_X_2),1)
WP_Y_2=WP_Y_2.reshape(len(WP_Y_2),1)


# 3rd WP



load_data_3=np.loadtxt('/home/aclsim2/catkin_ws/src/Morai_KM/scripts/path_3.csv',delimiter=',',dtype=str) # Input GPS csv file

longitude_3=load_data_3[1:,[0]] # x
latitude_3=load_data_3[1:,[1]] # y

longitude_3=longitude_3.reshape(len(longitude_3),1)
latitude_3=latitude_3.reshape(len(longitude_3),1)

longitude_3=longitude_3.astype(np.float)
latitude_3=latitude_3.astype(np.float)

DESLONG_tmp_3=np.zeros(len(longitude_3)).reshape(len(longitude_3),1)
DESLAT_tmp_3=np.zeros(len(latitude_3)).reshape(len(longitude_3),1)

DESLONG_tmp_3[0] = longitude_3[0]
DESLAT_tmp_3[0] = latitude_3[0]

a_3=0
b_3=2
number_tmp_3=[1]

for i in range(0,len(longitude_3)):
    lat1=latitude_3[a_3] 
    long1=longitude_3[a_3]
    lat2=latitude_3[i] 
    long2=longitude_3[i]
    dist=np.sqrt(((lat1-lat2)**2)+((long1-long2)**2))

    if dist >=0.01: 
        DESLONG_tmp_3[i] = longitude_3[i]
        DESLAT_tmp_3[i] = latitude_3[i]
        a_3=i

        number_tmp_3.append(i+1)

index_3=[]
AA_3=np.zeros(len(DESLAT_tmp_3)).reshape(len(DESLAT_tmp_3),1)
for i in range(len(AA_3)):
    if DESLAT_tmp_3[i]==0:
        index_3.append(i)

DESLONG_3=np.delete(DESLONG_tmp_3,index_3)
DESLAT_3=np.delete(DESLAT_tmp_3,index_3)
DESLONG_tmp_3=np.delete(DESLONG_tmp_3,index_3)
DESLAT_tmp_3=np.delete(DESLAT_tmp_3,index_3)

WP_X_1_3=DESLONG_3.reshape(len(DESLONG_3))
WP_Y_1_3=DESLAT_3.reshape(len(DESLAT_3))

x=DESLONG_3 
y=DESLAT_3 

WP_Curvature_3=np.zeros(len(DESLAT_3)).reshape(len(DESLAT_3),1) # 193x1
for i in range(1,len(DESLAT_3)+1):
    if i<=len(DESLAT_tmp_3)-2:
        x1=DESLONG_tmp_3[i-1]
        x2=DESLONG_tmp_3[i]
        x3=DESLONG_tmp_3[i+1]

        y1=DESLAT_tmp_3[i-1]
        y2=DESLAT_tmp_3[i]
        y3=DESLAT_tmp_3[i+1]
    elif i==len(DESLAT_tmp_3)-1:
        x1=DESLONG_tmp_3[i-2]
        x2=DESLONG_tmp_3[i-1]
        x3=DESLONG_tmp_3[i]

        y1=DESLAT_tmp_3[i-2]
        y2=DESLAT_tmp_3[i-1]
        y3=DESLAT_tmp_3[i]
    else:
        x1=DESLONG_tmp_3[i-3]
        x2=DESLONG_tmp_3[i-2]
        x3=DESLONG_tmp_3[i-1]

        y1=DESLAT_tmp_3[i-3]
        y2=DESLAT_tmp_3[i-2]
        y3=DESLAT_tmp_3[i-1]        
    rd_1=(y2-y1)/(x2-x1)
    rd_2=(y3-y2)/(x3-x2)
    d_1=1/rd_1
    d_2=1/rd_2

    x=((y3-y1)+(x2+x3)*d_2-(x1+x2)*d_1)/(2*(d_2-d_1))
    y=-d_1*(x-(x1+x2)/2)+(y1+y2)/2

    r=np.sqrt((x1-x)**2+(y1-y)**2)
    WP_Curvature_3[i-1]=1/r

    if((x1==x2 and x2==x3)or(y1==y2 and y2==y3)):
        WP_Curvature_3[i-1]=CURVATURE_STRAIGHT

WP_X_3=np.zeros(len(DESLAT_tmp_3)).reshape(len(DESLAT_tmp_3),1) # 193x1
WP_Y_3=np.zeros(len(DESLAT_tmp_3)).reshape(len(DESLAT_tmp_3),1)

WP_Curvature_index_straight_3=[] 
WP_Curvature_index_curve_3=[]

for i in range(len(WP_Curvature_3)):
    if(WP_Curvature_3[i]<CURVATURE_CURVE):
        WP_Curvature_index_straight_3.append(i+1) # list 97
    else:
        WP_Curvature_index_curve_3.append(i+1) # list 96

tmp_start_index = 1
tmp_end_index = 1

tmp_start_end_info_straight_3 = np.zeros(len(WP_Curvature_3)*2).reshape(len(WP_Curvature_3),2)
tmp_start_end_info_curve_3 = np.zeros(len(WP_Curvature_3)*2).reshape(len(WP_Curvature_3),2)

row_index = 0 

# WP_Curvature_index_straight_3
if (len(WP_Curvature_index_straight_3)!=0):
    for i in range(2,len(WP_Curvature_index_straight_3)+1): # i=2~97
        if ((WP_Curvature_index_straight_3[i-1] - WP_Curvature_index_straight_3[i-2])>1):

            tmp_end_index=i-1
            tmp_start_end_info_straight_3[row_index,:] = [tmp_start_index,tmp_end_index] # [86. 87.]

            if (i!=len(WP_Curvature_index_straight_3)): # 97
                tmp_start_index=i
                row_index+=1
            else:
                tmp_start_index=0
                row_index=0
        else:
            if (i==len(WP_Curvature_index_straight_3) and tmp_start_index!=0):
                tmp_start_end_info_straight_3[row_index,0] = tmp_start_index
                tmp_start_end_info_straight_3[row_index,1] = i   

    for i in range(len(tmp_start_end_info_straight_3)):
        if (tmp_start_end_info_straight_3[i,0]==0):
            idx=i
            break
    start_end_info_straight_3=np.zeros(idx*2).reshape(idx,2)    
    for i in range(idx):
        start_end_info_straight_3[i,0]=tmp_start_end_info_straight_3[i,0]
        start_end_info_straight_3[i,1]=tmp_start_end_info_straight_3[i,1]

    for i in range(len(start_end_info_straight_3)):
        i_st_range=np.arange(start_end_info_straight_3[i,0],(start_end_info_straight_3[i,1]+1),1)
        start_index=int(start_end_info_straight_3[i,0])
        
        tmp_WP_first_index_3=WP_Curvature_index_straight_3[int(i_st_range[0])-1]
        tmp_WP_end_index_3=WP_Curvature_index_straight_3[int(i_st_range[-1])-1]
        

        WP_X_3[tmp_WP_first_index_3-1]=DESLONG_tmp_3[tmp_WP_first_index_3-1]
        WP_X_3[tmp_WP_end_index_3-1]=DESLONG_tmp_3[tmp_WP_end_index_3-1]

        WP_Y_3[tmp_WP_first_index_3-1]=DESLAT_tmp_3[tmp_WP_first_index_3-1]
        WP_Y_3[tmp_WP_end_index_3-1]=DESLAT_tmp_3[tmp_WP_end_index_3-1]
   
        for j in range(int(i_st_range[0]),int(i_st_range[-1]+1)): 
            tmp_WP_index=WP_Curvature_index_straight_3[j-1] 

            WP_X_start=DESLONG_tmp_3[start_index-1] 
            WP_Y_start=DESLAT_tmp_3[start_index-1]

            WP_X_end=DESLONG_tmp_3[tmp_WP_index-1]
            WP_Y_end=DESLAT_tmp_3[tmp_WP_index-1]

            tmp_distance=np.sqrt(((WP_X_start-WP_X_end)**2)+((WP_Y_start-WP_Y_end)**2))

            if (tmp_distance >= DISTANCE_WP2WP_STRAIGHT):
                WP_X_3[tmp_WP_index-1]=WP_X_end
                WP_Y_3[tmp_WP_index-1]=WP_Y_end

                start_index=tmp_WP_index

tmp_start_index=1
row_index=0

# WP_Curvature_index_curve_3
if (len(WP_Curvature_index_curve_3)!=0):
    for i in range(2,len(WP_Curvature_index_curve_3)+1): # 2~97
        if ((WP_Curvature_index_curve_3[i-1] - WP_Curvature_index_curve_3[i-2])>1):
            tmp_end_index=i-1
            tmp_start_end_info_curve_3[row_index,:] = [tmp_start_index,tmp_end_index]

            if (i!=len(WP_Curvature_index_curve_3)): # 120
                tmp_start_index=i
                row_index+=1
            else:
                tmp_start_index=0
                row_index=0
        else:
            if (i==len(WP_Curvature_index_curve_3) and tmp_start_index!=0):
                tmp_start_end_info_curve_3[row_index,0] = tmp_start_index
                tmp_start_end_info_curve_3[row_index,1] = i   

    for i in range(len(tmp_start_end_info_curve_3)): # 120
        if (tmp_start_end_info_curve_3[i,0]==0):
            idx=i
            break
    start_end_info_curve_3=np.zeros(idx*2).reshape(idx,2)    
    for i in range(idx): # idx=36
        start_end_info_curve_3[i,0]=tmp_start_end_info_curve_3[i,0]
        start_end_info_curve_3[i,1]=tmp_start_end_info_curve_3[i,1]

    for i in range(len(start_end_info_curve_3)):
        i_cv_range=np.arange(start_end_info_curve_3[i,0],start_end_info_curve_3[i,1]+1,1)
        start_index=int(start_end_info_curve_3[i,0])

        tmp_WP_first_index_3=WP_Curvature_index_curve_3[int(i_cv_range[0])-1]
        tmp_WP_end_index_3=WP_Curvature_index_curve_3[int(i_cv_range[-1])-1]

        WP_X_3[tmp_WP_first_index_3-1]=DESLONG_tmp_3[tmp_WP_first_index_3-1]
        WP_X_3[tmp_WP_end_index_3-1]=DESLONG_tmp_3[tmp_WP_end_index_3-1]

        WP_Y_3[tmp_WP_first_index_3-1]=DESLAT_tmp_3[tmp_WP_first_index_3-1]
        WP_Y_3[tmp_WP_end_index_3-1]=DESLAT_tmp_3[tmp_WP_end_index_3-1]

        for j in range(int(i_cv_range[0]),int(i_cv_range[-1]+1)):
            tmp_WP_index=WP_Curvature_index_curve_3[j-1]

            WP_X_start=DESLONG_tmp_3[start_index-1]
            WP_Y_start=DESLAT_tmp_3[start_index-1]

            WP_X_end=DESLONG_tmp_3[tmp_WP_index-1]
            WP_Y_end=DESLAT_tmp_3[tmp_WP_index-1]

            tmp_distance=np.sqrt(((WP_X_start-WP_X_end)**2)+((WP_Y_start-WP_Y_end)**2))

            if (tmp_distance >= DISTANCE_WP2WP_CURVE):
                WP_X_3[tmp_WP_index-1]=WP_X_end
                WP_Y_3[tmp_WP_index-1]=WP_Y_end

                start_index=tmp_WP_index

del_indexes=[]
for i in range(len(WP_X_3)):
    if (WP_X_3[i]==0):
        del_indexes.append(i)
WP_X_3=np.delete(WP_X_3,del_indexes)
WP_Y_3=np.delete(WP_Y_3,del_indexes)
WP_X_3=WP_X_3.reshape(len(WP_X_3),1)
WP_Y_3=WP_Y_3.reshape(len(WP_Y_3),1)


# temp
WP_X_3[108] = WP_X_3[108] - (WP_X_3[107]-WP_X_3[106])/1.2
WP_Y_3[108] = WP_Y_3[108] - (WP_Y_3[107]-WP_Y_3[106])/1.2

# for WP 15
WP_X_1[14] = WP_X_1[14] - (0.25*(WP_X_1[14]-WP_X_1[13]))/(np.sqrt((WP_X_1[15]-WP_X_1[14])**2+(WP_Y_1[15]-WP_Y_1[14])**2))
WP_Y_1[14] = WP_Y_1[14] - (0.25*(WP_Y_1[14]-WP_Y_1[13]))/(np.sqrt((WP_X_1[15]-WP_X_1[14])**2+(WP_Y_1[15]-WP_Y_1[14])**2))

# for WP 33~35
#WP_X_1[32] = WP_X_1[32] - (0.05*(WP_X_1[32]-WP_X_1[31]))/(np.sqrt((WP_X_1[32]-WP_X_1[31])**2+(WP_Y_1[32]-WP_Y_1[31])**2))
#WP_Y_1[32] = WP_Y_1[32] - (0.05*(WP_Y_1[32]-WP_Y_1[31]))/(np.sqrt((WP_X_1[32]-WP_X_1[31])**2+(WP_Y_1[32]-WP_Y_1[31])**2))
#WP_X_1[33] = WP_X_1[33] - (0.05*(WP_X_1[33]-WP_X_1[32]))/(np.sqrt((WP_X_1[33]-WP_X_1[32])**2+(WP_Y_1[33]-WP_Y_1[32])**2))
#WP_Y_1[33] = WP_Y_1[33] - (0.05*(WP_Y_1[33]-WP_Y_1[32]))/(np.sqrt((WP_X_1[33]-WP_X_1[32])**2+(WP_Y_1[33]-WP_Y_1[32])**2))
WP_X_1[34] = WP_X_1[34] + (0.125*(WP_X_1[35]-WP_X_1[34]))/(np.sqrt((WP_X_1[35]-WP_X_1[34])**2+(WP_Y_1[35]-WP_Y_1[34])**2))
WP_Y_1[34] = WP_Y_1[34] + (0.125*(WP_Y_1[35]-WP_Y_1[34]))/(np.sqrt((WP_X_1[35]-WP_X_1[34])**2+(WP_Y_1[35]-WP_Y_1[34])**2))


WP_X_2[14] = WP_X_2[14] - (0.2*(WP_X_2[14]-WP_X_2[13]))/(np.sqrt((WP_X_2[15]-WP_X_2[14])**2+(WP_Y_2[15]-WP_Y_2[14])**2))
WP_Y_2[14] = WP_Y_2[14] - (0.2*(WP_Y_2[14]-WP_Y_2[13]))/(np.sqrt((WP_X_2[15]-WP_X_2[14])**2+(WP_Y_2[15]-WP_Y_2[14])**2))

#WP_X_2[32] = WP_X_2[32] - (0.2*(WP_X_2[32]-WP_X_2[31]))/(np.sqrt((WP_X_2[32]-WP_X_2[31])**2+(WP_Y_2[32]-WP_Y_2[31])**2))
#WP_Y_2[32] = WP_Y_2[32] - (0.2*(WP_Y_2[32]-WP_Y_2[31]))/(np.sqrt((WP_X_2[32]-WP_X_2[31])**2+(WP_Y_2[32]-WP_Y_2[31])**2))
#WP_X_2[31] = WP_X_2[31] - (0.2*(WP_X_2[32]-WP_X_2[31]))/(np.sqrt((WP_X_2[32]-WP_X_2[31])**2+(WP_Y_2[32]-WP_Y_2[31])**2))
#WP_Y_2[31] = WP_Y_2[31] - (0.2*(WP_Y_2[32]-WP_Y_2[31]))/(np.sqrt((WP_X_2[32]-WP_X_2[31])**2+(WP_Y_2[32]-WP_Y_2[31])**2))
#WP_X_2[30] = WP_X_2[30] - (0.2*(WP_X_2[32]-WP_X_2[31]))/(np.sqrt((WP_X_2[32]-WP_X_2[31])**2+(WP_Y_2[32]-WP_Y_2[31])**2))
#WP_Y_2[30] = WP_Y_2[30] - (0.2*(WP_Y_2[32]-WP_Y_2[31]))/(np.sqrt((WP_X_2[32]-WP_X_2[31])**2+(WP_Y_2[32]-WP_Y_2[31])**2))


if (plot_switch==1):
    target=34
    # Plot GPS Point and WP Point
    plt.figure()
    #plt.plot(longitude_3,latitude_3)
    plt.plot(longitude,latitude)
    #plt.scatter(longitude[target-1],latitude[target-1],s=10,c='red')
    #plt.scatter(WP_X_3,WP_Y_3,s=5,c='blue')    
    #plt.scatter(WP_X_2,WP_Y_2,s=5,c='black')
    plt.scatter(WP_X_1,WP_Y_1,s=4,c='black')
    plt.scatter(WP_X_1[target],WP_Y_1[target],c='red',s=15)

    plt.scatter(14.63,0.3,s=1,c='black')
    plt.scatter(14.63,0.4,s=1,c='black')
    plt.scatter(14.63,0.5,s=1,c='black')
    plt.scatter(14.63,0.6,s=1,c='black')
    plt.scatter(14.63,0.7,s=1,c='black')
    plt.scatter(14.63,0.8,s=1,c='black')        
    plt.grid()
    plt.show()


#np.savetxt('kmtrackWP.csv',WP, fmt='%lf', delimiter=',')