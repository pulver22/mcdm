import numpy as np
import matplotlib as mpl
from matplotlib import pyplot as plt
import matplotlib.pylab as pl
from mpl_toolkits.mplot3d import Axes3D

param_list=[0.0, 0.01, 0.1, 0.5, 1.0, 5.0, 10.0, 50.0, 100.0]

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


def mjrFormatter(x, pos):
    return "${{{0}}}$".format(param_list[pos])

def mjrFormatter_no_TeX(x, pos):
    return "2^{0}".format(x)

wAVG_r1 = np.genfromtxt('/tmp/result_gs_wAVG_r1.csv', skip_header=True, delimiter=',')  
wAVG_r2 = np.genfromtxt('/tmp/result_gs_wAVG_r2.csv', skip_header=True, delimiter=',')  
wAVG_r3 = np.genfromtxt('/tmp/result_gs_wAVG_r3.csv', skip_header=True, delimiter=',')  
mcdm_r1 = np.genfromtxt('/tmp/result_gs_mcdm_r1.csv', skip_header=True, delimiter=',')  
mcdm_r2 = np.genfromtxt('/tmp/result_gs_mcdm_r2.csv', skip_header=True, delimiter=',')  
mcdm_r3 = np.genfromtxt('/tmp/result_gs_mcdm_r3.csv', skip_header=True, delimiter=',')  

mcdm_list = [mcdm_r1, mcdm_r2, mcdm_r3]
wAVG_list = [wAVG_r1, wAVG_r2, wAVG_r3]

final_mcdm_matrix = np.zeros(shape=(len(param_list), len(param_list)))
final_wAVG_matrix = np.zeros(shape=(len(param_list), len(param_list)))
for mcdm in mcdm_list:
    # Order the two arrays based on w_info_gain
    mcdm_sorted = mcdm[mcdm[:,0].argsort()]
    # Keep only [w_info_gain, w_travel_distance, travelledDistance]
    mcdm_sorted = np.delete(mcdm_sorted, [2,3,4,5,7], axis=1)
    # Remove also the criteria weight
    mcdm_matrix = np.delete(mcdm_sorted, [0,1], axis=1)
    # Reshape into a 2D matrix
    mcdm_matrix = np.reshape(mcdm_matrix, (-1, len(param_list)))
    final_mcdm_matrix = final_mcdm_matrix +  mcdm_matrix

for wAVG in wAVG_list:
    # Order the two arrays based on w_info_gain
    wAVG_sorted = wAVG[wAVG[:,0].argsort()]
    # Keep only [w_info_gain, w_travel_distance, travelledDistance]
    wAVG_sorted = np.delete(wAVG_sorted, [2,3,4,5,7], axis=1)
    # Remove also the criteria weight
    wAVG_matrix = np.delete(wAVG_sorted, [0,1], axis=1)
    # Reshape into a 2D matrix
    wAVG_matrix = np.reshape(wAVG_matrix, (-1, len(param_list)))
    final_wAVG_matrix = final_wAVG_matrix + wAVG_matrix


# Normalize the matrix
final_mcdm_matrix /= len(mcdm_list)
final_wAVG_matrix /= len(wAVG_list)

np.savetxt('/tmp/clean_wAVG.txt', final_wAVG_matrix)
np.savetxt('/tmp/clean_mcdm.txt', final_mcdm_matrix)
# # Remove also the criteria weight
# wAVG_matrix = np.delete(wAVG_sorted, [0,1], axis=1)
# mcdm_matrix = np.delete(mcdm_sorted, [0,1], axis=1)

# max_wAVG = np.max(wAVG_matrix[:,0])
# max_mcdm = np.max(mcdm_matrix[:,0])
# wAVG_matrix = np.reshape(wAVG_matrix, (-1, 9))
# mcdm_matrix = np.reshape(mcdm_matrix, (-1, 9))
# print(final_wAVG_matrix.shape)
# print(wAVG)
# print(final_mcdm_matrix.shape)

# # print("{}, {}".format(max_wAVG, max_mcdm))
# max_value = max_wAVG if max_wAVG >= max_mcdm else max_mcdm

# Plot the matrix
fig = plt.figure()
ax1 = fig.add_subplot(1,2,1)
ax1.set_aspect('equal')
plt.imshow(final_wAVG_matrix, interpolation='nearest', cmap=plt.cm.ocean)
plt.colorbar()
plt.ylabel("w_info_gain")
plt.xlabel("w_travel_distance")
# a = plt.gca()
plt.yticks(np.arange(0, len(param_list), 1.0))
ax1.yaxis.set_major_formatter(mpl.ticker.FuncFormatter(mjrFormatter))
plt.xticks(np.arange(0, len(param_list), 1.0))
ax1.xaxis.set_major_formatter(mpl.ticker.FuncFormatter(mjrFormatter))
plt.title("TravelledDistance [wAVG]")
# plt.show()

ax2 = fig.add_subplot(1,2,2)
ax2.set_aspect('equal')
plt.imshow(final_mcdm_matrix, interpolation='nearest', cmap=plt.cm.ocean)
plt.colorbar()
plt.ylabel("w_info_gain")
plt.xlabel("w_travel_distance")
plt.title("TravelledDistance [MCDM]")
plt.yticks(np.arange(0, len(param_list), 1.0))
ax2.yaxis.set_major_formatter(mpl.ticker.FuncFormatter(mjrFormatter))
plt.xticks(np.arange(0, len(param_list), 1.0))
ax2.xaxis.set_major_formatter(mpl.ticker.FuncFormatter(mjrFormatter))
plt.show()


# plt.bar(x=wAVG_sorted[:,0]-0.0025, height=wAVG_sorted[:,2], width=0.005, label='wAVG')
# plt.bar(x=mcdm_sorted[:,0]+0.0025, height=mcdm_sorted[:,2], width=0.005, label='mcdm')
# plt.legend()
# plt.xlabel("w_info_gain")
# plt.ylabel("TravelledDistance[m]")
# plt.title("Travelled distance: MCDM vs wAVG")
# plt.show()
