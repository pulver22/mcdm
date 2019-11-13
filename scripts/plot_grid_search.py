import numpy as np
import matplotlib
from matplotlib import pyplot as plt
import matplotlib.pylab as pl
from mpl_toolkits.mplot3d import Axes3D



def build_matrix(input_array, max_value):
    max_index = np.max(input_array[:,0])
    matrix_size = int(max_index*100)
    matrix = np.ones(shape=(matrix_size, matrix_size))
    matrix *= max_value

    for w_info_gain in range(0, matrix_size):
        for w_travel_distance in range(matrix_size, 0, -1):
            # print("\n[{},{}]".format(w_info_gain/100.0, w_travel_distance/100.0))
            if (w_info_gain + w_travel_distance == matrix_size):
                index_arr = np.where(input_array[:,0] == (w_info_gain/100.0))
                # print(index_arr[0][0])
                value = input_array[index_arr, 2][0][0]
                # print(value)
                matrix[w_info_gain, w_travel_distance-1] = value

    return matrix


wAVG = np.genfromtxt('/home/pulver/Desktop/mcdm_wAVG/result_gs_wAVG.csv', skip_header=True, delimiter=',')  
mcdm = np.genfromtxt('/home/pulver/Desktop/mcdm_wAVG/result_gs_mcdm.csv', skip_header=True, delimiter=',')  
dtype = [('w_info_gain', float),
        ('w_travel_distance', float),
        ('w_sensing_time', float),
        ('w_rfid_gain', float),
        ('coverage', float),
        ('numConfiguration', float),
        ('travelledDistance', float),
        ('totalScanTime', float)]

# Order the two arrays based on w_info_gain
wAVG_sorted = wAVG[wAVG[:,0].argsort()]
mcdm_sorted = mcdm[mcdm[:,0].argsort()]
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(mcdm_sorted[:,0], mcdm_sorted[:,1], mcdm_sorted[:,6], c='r', marker='o')
# ax.set_xlabel('w_info_gain')
# ax.set_ylabel('w_travel_distance')
# ax.set_zlabel('TravelledDistance')
# plt.show()

# Keep only [w_info_gain, w_travel_distance, travelledDistance]
wAVG_sorted = np.delete(wAVG_sorted, [2,3,4,5,7,8], axis=1)
mcdm_sorted = np.delete(mcdm_sorted, [2,3,4,5,7,8], axis=1)

max_wAVG = np.max(wAVG_sorted[:,2])
max_mcdm = np.max(mcdm_sorted[:,2])
print("{}, {}".format(max_wAVG, max_mcdm))
max_value = max_wAVG if max_wAVG >= max_mcdm else max_mcdm

wAVG_matrix = build_matrix(wAVG_sorted, max_value)
mcdm_matrix = build_matrix(mcdm_sorted, max_value)
# Plot the matrix
fig = plt.figure()
ax2 = fig.add_subplot(1,2,1)
ax2.set_aspect('equal')
plt.imshow(wAVG_matrix, interpolation='nearest', cmap=plt.cm.ocean)
plt.colorbar()
plt.ylabel("w_info_gain[*100]")
plt.xlabel("w_travel_distance[*100]")
plt.title("TravelledDistance [wAVG]")
# plt.show()

# fig = plt.figure()
ax2 = fig.add_subplot(1,2,2)
ax2.set_aspect('equal')
plt.imshow(mcdm_matrix, interpolation='nearest', cmap=plt.cm.ocean)
plt.colorbar()
plt.ylabel("w_info_gain[*100]")
plt.xlabel("w_travel_distance[*100]")
plt.title("TravelledDistance [MCDM]")
plt.show()

