import numpy as np
import os
import math

def replace_in_file(file_path, old_str, new_str):
    fr = open(file_path,'r')
    lines = fr.readlines()
    fr.close()
    
    fw = open(file_path,'w')
    for line in lines:
        fw.write(line.replace(old_str, new_str))
    fw.close()


def morai2kitti(file_path, save_path, num):
    
    file = np.loadtxt(file_path, dtype = str, delimiter = ' ')
    row = file.shape[0]
    if len(file) == 0 :
        print('file is empty')
    else:

        Class = file[:,0].reshape(row,1)
        truncated = np.zeros(row).reshape(row,1)
        occluded = np.zeros(row).reshape(row,1)
        alpha = np.zeros(row).reshape(row,1)
        bbox = np.zeros(row).reshape(row,1)
        length = file[:,5].astype(dtype='float32').reshape(row,1)
        width = file[:,6].astype(dtype='float32').reshape(row,1)
        height = file[:,7].astype(dtype='float32').reshape(row,1)
        loc_x = file[:,2].astype(dtype='float32').reshape(row,1)
        loc_y = file[:,3].astype(dtype='float32').reshape(row,1)
        loc_z = file[:,4].astype(dtype='float32').reshape(row,1) - height/2
        yaw = file[:,8].astype(dtype='float32').reshape(row,1) - math.pi/2

        # loc_x = file[:,2].reshape(row,1)
        # loc_y = file[:,3].reshape(row,1)
        # loc_z = file[:,4].reshape(row,1)
        # length = file[:,5].reshape(row,1)
        # width = file[:,6].reshape(row,1)
        # height = file[:,7].reshape(row,1)
        # yaw = file[:,8].reshape(row,1)

    
    kitti_txt = np.hstack((Class, truncated, occluded, alpha, bbox, bbox, bbox, bbox,
                   height, width, length, -loc_y, -loc_z, loc_x, -yaw))
    kitti_path = save_path + '/' + num + '.txt'

    np.savetxt(kitti_path, kitti_txt, fmt ='%s', delimiter = ' ')


load_path_label = '/home/bomint/Desktop/MORAI/txt'
load_path_pcd = '/home/bomint/Desktop/MORAI/lidar'
save_path = '/home/bomint/Desktop/MORAI/save'

file_list = os.listdir(load_path_label)
file_list_txt = [file for file in file_list if file.endswith(".txt")]

for i in range(len(file_list_txt)):
    label_path = load_path_label + '/' + file_list_txt[i]
    pcd_num = file_list_txt[i].replace('txt','bin')
    pcd_path = load_path_pcd + '/' + pcd_num

    print(label_path)
    print(pcd_path)
    replace_in_file(label_path, ",", " ") # morai txt에 있는 ,를 띄어쓰기로 다 치환 후 저장
    replace_in_file(label_path, "Vehicle", "Car")
    replace_in_file(label_path, "Sedan", "Car")
    replace_in_file(label_path, "SUV", "Car")
    num = '%06i' % i
    morai2kitti(label_path, save_path, num)
    # num2 = '%06i' % i + '.bin'
    # os.rename(pcd_path, num2)
