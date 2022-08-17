import csv
import numpy as np
import matplotlib.pyplot as plt
def read_csv(file_name):
    f = open(file_name, "r")
    reader = csv.reader(f)
    WP_x, WP_y  = [], []
    for row in reader:
        WP_x.append(float(row[0]))
        WP_y.append(float(row[1]))
        
    return WP_x, WP_y




def get_WP_index(ego_x, ego_y, WP_x, WP_y):
    WP_x = np.array(WP_x)
    WP_y = np.array(WP_y)
    distance = np.sqrt((WP_x-ego_x)**2 + (WP_y -ego_y) **2)
    
    index = np.argmin(distance)

    return index+1

def read_txt(file_name):
    openFile = open(file_name, 'r')
    line=openFile.readlines()
    WP_x, WP_y  = [], []
    for i in line :
        tmp=i.split()
        WP_x.append(float(tmp[0]))
        WP_y.append(float(tmp[1]))
        
        
    openFile.close()
    return  WP_x, WP_y
# WP_x, WP_y =read_txt('kcity.txt')

# print(WP_x)