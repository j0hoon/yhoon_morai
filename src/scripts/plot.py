import csv
import numpy as np
import matplotlib.pyplot as plt
def read_txt(file_name):
    x, y, time  = [], [] ,[]
    f = open(file_name,'r')
    lines = f.readlines()
    for line in lines:
        line = line.split()

        x.append(float(line[0]))
        y.append(float(line[1]))
        time.append(float(line[2])- 0.5013883113861084)
    f.close()  
    return x, y, time 



a, b, time = read_txt('/home/aclsim2/catkin_ws/src/velocity.txt')
print(a)
plt.plot(time, a, label = 'actual_vel')
plt.ylim([-1,9.5])
plt.ylabel('velocity (km/h)')
plt.plot(time, b, label = 'target_vel')
plt.xlim([-1 ,4])
plt.xlabel('time (sec)')
plt.legend()
plt.grid(True)
plt.show()
# import numpy as np



# def cyclodial_curve(target_V, max_acc ,t):

#     A = (target_V) ** 2 / (4 * max_acc)
#     B = 2 * max_acc/(target_V)
    
#     time_interval = target_V * np.pi / (2 * max_acc)

#     if t < time_interval:

#         return A * B *( 1 - np.cos(B * t))

#     else:

#         return target_V




# for t in range(100):

#     t = 0.1 * t
#     v = cyclodial_curve(9, 3, t)

#     print(v)



