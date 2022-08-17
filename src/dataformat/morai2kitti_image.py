import numpy as np
import os
import math
################ select_blank_file ####################
def select_blank_file(name, label_path, image_path):
    label_path = label_path + '/' + name 
    image = name.replace('txt','png')
    image = image.replace('Instance','Intensity')
    image_path = image_path + '/' + image
    
    a= os.stat(label_path).st_size == 0
    if a == True:        
        os.remove(label_path)
        os.remove(image_path)
        print(f'remove {name}')
    else:
        pass

################ replace string ####################
def replace_in_file(file_path, old_str, new_str):
    fr = open(file_path,'r')
    lines = fr.readlines()
    fr.close()
    
    fw = open(file_path,'w')

    for line in lines:
        fw.write(line.replace(old_str, new_str))  
    fw.close()


######### morai data conver to kitti format data ######

def morai2kitti(file_path, save_path, num):
    
    file = np.loadtxt(file_path, dtype = str, delimiter = ' ')
    row = file.shape[0]
    if len(file) >= 17 :
        Class = file[0]
        alpha = 2.57

        bbox_x1 = file[4].astype(dtype='float32')
        bbox_y1 = file[5].astype(dtype='float32')
        bbox_x2 = file[6].astype(dtype='float32')
        bbox_y2 = file[7].astype(dtype='float32')
        minus_one = -1
        minus_1000 = -1000
        minus_8 = -8.43
        kitti_txt = np.hstack((Class, minus_one, minus_one, alpha, bbox_x1, bbox_y1, bbox_x2, bbox_y2,
                        minus_one, minus_one, minus_one, minus_1000, minus_1000, minus_1000, minus_8)).reshape(1,15)
        kitti_path = save_path + '/' + num + '.txt'
        
    else:
        
        Class = file[:,0].reshape(row,1)
        alpha = np.ones(row).reshape(row,1) * (2.57)

        bbox_x1 = file[:,4].astype(dtype='float32').reshape(row,1)
        bbox_y1 = file[:,5].astype(dtype='float32').reshape(row,1)
        bbox_x2 = file[:,6].astype(dtype='float32').reshape(row,1)
        bbox_y2 = file[:,7].astype(dtype='float32').reshape(row,1)
        minus_one = np.ones(row).reshape(row,1) * (-1)
        minus_1000 = np.ones(row).reshape(row,1) * (-1000)
        minus_8 = np.ones(row).reshape(row,1) * (-8.43)


        kitti_txt = np.hstack((Class, minus_one, minus_one, alpha, bbox_x1, bbox_y1, bbox_x2, bbox_y2,
                        minus_one, minus_one, minus_one, minus_1000, minus_1000, minus_1000, minus_8))
        kitti_path = save_path + '/' + num + '.txt'

    np.savetxt(kitti_path, kitti_txt, fmt ='%s', delimiter = ' ')

########## 실행 코드 ###########


save_label_path = 'C:/Users/tthh1/Desktop/yolo_data/label'
save_image_path = 'C:/Users/tthh1/Desktop/yolo_data/image'
load_label_path = 'C:/Users/tthh1/Desktop/SensorData/bike_label'
load_image_path = 'C:/Users/tthh1/Desktop/SensorData/bike_image'


file_list = os.listdir(load_label_path)
file_list_txt = [file for file in file_list if file.endswith(".txt")]
for i in range(len(file_list_txt)):
    name = file_list_txt[i]
    select_blank_file(name, load_label_path, load_image_path)


file_list = os.listdir(load_label_path)
file_list_txt = [file for file in file_list if file.endswith(".txt")]
for i in range(len(file_list_txt)):
    label_path = load_label_path + '/' + file_list_txt[i]
    replace_in_file(label_path, ",", " ")     
    replace_in_file(label_path, "Obstacle", "bike")  #bike 부분 해당 클래스로 이름 바꿔줘야함


file_list = os.listdir(load_label_path)
file_list_txt = [file for file in file_list if file.endswith(".txt")]
for i in range(len(file_list_txt)):
    name = file_list_txt[i]
    label_path = load_label_path + '/' + name
    
    image = name.replace('txt','png')
    image = image.replace('Instance','Intensity')
    image_path = load_label_path + '/' + image
    
    num = '%06i' % (i+1809)
    image_save_path = save_image_path +'/'+ num +'.png'
    print(num)
    morai2kitti(label_path, save_label_path, num)
    os.replace(image_path,image_save_path)