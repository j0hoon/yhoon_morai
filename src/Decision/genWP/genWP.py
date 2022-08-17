import numpy as np
import matplotlib.pyplot as plt
import os

DISTANCE_WP2WP_STRAIGHT=1 # init=50
DISTANCE_WP2WP_CURVE=0.5 # init=5
plot_switch=0

load_data=np.loadtxt('path_name_1.csv',delimiter=',',dtype=str) # Input GPS csv file

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

    if dist >=0.05: 
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
CURVATURE_CURVE=0.001

x=DESLONG 
y=DESLAT 

WP_Curvature=np.zeros(len(DESLAT)).reshape(len(DESLAT),1) # 193x1
delta=1e-4
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
    d_1=1/rd_1+delta
    d_2=1/rd_2+delta

    x=((y3-y1)+(x2+x3)*d_2-(x1+x2)*d_1)/(2*(d_2-d_1))
    y=-d_1*(x-(x1+x2)/2)+(y1+y2)/2

    r=np.sqrt((x1-x)**2+(y1-y)**2)
    WP_Curvature[i-1]=1/r

    if((x1==x2 and x2==x3)or(y1==y2 and y2==y3)):
        WP_Curvature[i-1]=CURVATURE_STRAIGHT

WP_X=np.zeros(len(DESLAT_tmp)).reshape(len(DESLAT_tmp),1) # 193x1
WP_Y=np.zeros(len(DESLAT_tmp)).reshape(len(DESLAT_tmp),1)

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
        

        WP_X[tmp_WP_first_index-1]=DESLONG_tmp[tmp_WP_first_index-1]
        WP_X[tmp_WP_end_index-1]=DESLONG_tmp[tmp_WP_end_index-1]

        WP_Y[tmp_WP_first_index-1]=DESLAT_tmp[tmp_WP_first_index-1]
        WP_Y[tmp_WP_end_index-1]=DESLAT_tmp[tmp_WP_end_index-1]
   
        for j in range(int(i_st_range[0]),int(i_st_range[-1]+1)): 
            tmp_WP_index=WP_Curvature_index_straight[j-1] 

            WP_X_start=DESLONG_tmp[start_index-1] 
            WP_Y_start=DESLAT_tmp[start_index-1]

            WP_X_end=DESLONG_tmp[tmp_WP_index-1]
            WP_Y_end=DESLAT_tmp[tmp_WP_index-1]

            tmp_distance=np.sqrt(((WP_X_start-WP_X_end)**2)+((WP_Y_start-WP_Y_end)**2))

            if (tmp_distance >= DISTANCE_WP2WP_STRAIGHT):
                WP_X[tmp_WP_index-1]=WP_X_end
                WP_Y[tmp_WP_index-1]=WP_Y_end

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

        WP_X[tmp_WP_first_index-1]=DESLONG_tmp[tmp_WP_first_index-1]
        WP_X[tmp_WP_end_index-1]=DESLONG_tmp[tmp_WP_end_index-1]

        WP_Y[tmp_WP_first_index-1]=DESLAT_tmp[tmp_WP_first_index-1]
        WP_Y[tmp_WP_end_index-1]=DESLAT_tmp[tmp_WP_end_index-1]

        for j in range(int(i_cv_range[0]),int(i_cv_range[-1]+1)):
            tmp_WP_index=WP_Curvature_index_curve[j-1]

            WP_X_start=DESLONG_tmp[start_index-1]
            WP_Y_start=DESLAT_tmp[start_index-1]

            WP_X_end=DESLONG_tmp[tmp_WP_index-1]
            WP_Y_end=DESLAT_tmp[tmp_WP_index-1]

            tmp_distance=np.sqrt(((WP_X_start-WP_X_end)**2)+((WP_Y_start-WP_Y_end)**2))

            if (tmp_distance >= DISTANCE_WP2WP_CURVE):
                WP_X[tmp_WP_index-1]=WP_X_end
                WP_Y[tmp_WP_index-1]=WP_Y_end

                start_index=tmp_WP_index

del_indexes=[]
for i in range(len(WP_X)):
    if (WP_X[i]==0):
        del_indexes.append(i)
WP_X=np.delete(WP_X,del_indexes)
WP_Y=np.delete(WP_Y,del_indexes)
WP_X=WP_X.reshape(len(WP_X),1)
WP_Y=WP_Y.reshape(len(WP_Y),1)
WP=np.zeros(len(WP_X)*2)
WP=WP.reshape(len(WP_X),2)

for i in range(len(WP_X)):
    WP[i,0]=WP_X[i]
    WP[i,1]=WP_Y[i]

if (plot_switch==1):
    # Plot GPS Point and WP Point
    plt.figure()
    plt.scatter(longitude,latitude,s=3)
    plt.scatter(WP_X,WP_Y,s=10)
    plt.grid()
    plt.show()

'''
np.savetxt('kmtrackWP.csv',WP, fmt='%lf', delimiter=',')
'''

