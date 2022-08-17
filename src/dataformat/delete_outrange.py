import numpy as np
import os


filepath = '/home/bomint/Desktop/MORAI/save'

savepath = '/home/bomint/Desktop/MORAI/final_label'

for i in range(len(os.listdir(filepath))):
    num = '%06i' % i
    label_path = filepath + '/' + num +'.txt'
    label_file = np.loadtxt(label_path, dtype = str, delimiter = ' ')
    index = []
    for idx in range(label_file.shape[0]):
        if float(label_file[idx,13]) > 0 and float(label_file[idx,13]) < 0.6:
            index.append(idx)

        if label_file[idx,0] == 'Car' or 'Truck':
            if float(label_file[idx,13]) > 50 or float(label_file[idx,13]) < -50 or -float(label_file[idx,11]) > 50 or -float(label_file[idx,11])< -50:
                index.append(idx)
        
        elif label_file[idx,0] == 'Pedestrian':
            if float(label_file[idx,13]) > 25 or float(label_file[idx,13]) < -25 or -float(label_file[idx,11]) > 25 or -float(label_file[idx,11])< -25:
                index.append(idx)
            # print(index)
        else:
            continue
            

        # else:
        #     continue
    index =np.array(index)
    label_file = np.delete(label_file,index.T, axis=0)

    final_savepath = savepath + '/' + num + '.txt'
    print(num)
    np.savetxt(final_savepath, label_file, fmt ='%s', delimiter = ' ')




