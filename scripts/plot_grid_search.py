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


def sortData(input_array):
    input_array = input_array[input_array[:, 0].argsort()]
    num_mini_batch = int(input_array.shape[0] / len(param_list)) + 1
    for i in range(0, num_mini_batch):
        # Create the minimatch to sort
        mini_batch = input_array[i*len(param_list):len(param_list)+i*len(param_list)]
        # Sort it first based on the first column and the on the second
        mini_batch = mini_batch[mini_batch[:, 1].argsort()]
        # mini_batch = mini_batch[mini_batch[:, 1].argsort()
        # Update the original array
        input_array[i*len(param_list):len(param_list)+i*len(param_list)] = mini_batch 
        np.savetxt('/tmp/sorted_' + str(i) + '.txt', mini_batch)
    np.savetxt('/tmp/sorted_array.txt', input_array)
    return input_array

def mjrFormatter(x, pos):
    return "${{{0}}}$".format(param_list[pos])

def mjrFormatter_no_TeX(x, pos):
    return "2^{0}".format(x)

wAVG_r1 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_wAVG_r0.csv', skip_header=True, delimiter=',')  
wAVG_r2 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_wAVG_r2.csv', skip_header=True, delimiter=',')  
wAVG_r3 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_wAVG_r3.csv', skip_header=True, delimiter=',')  
wAVG_r4 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_wAVG_r4.csv', skip_header=True, delimiter=',')  
wAVG_r5 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_wAVG_r5.csv', skip_header=True, delimiter=',')  
wAVG_r6 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_wAVG_r6.csv', skip_header=True, delimiter=',')  
wAVG_r7 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_wAVG_r7.csv', skip_header=True, delimiter=',')  
wAVG_r8 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_wAVG_r8.csv', skip_header=True, delimiter=',')  
wAVG_r9 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_wAVG_r9.csv', skip_header=True, delimiter=',')  
wAVG_r10 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_wAVG_r0.csv', skip_header=True, delimiter=',')  

mcdm_r1 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_mcdm_r1.csv', skip_header=True, delimiter=',')  
mcdm_r2 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_mcdm_r2.csv', skip_header=True, delimiter=',')  
mcdm_r3 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_mcdm_r3.csv', skip_header=True, delimiter=',')  
mcdm_r4 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_mcdm_r4.csv', skip_header=True, delimiter=',')  
mcdm_r5 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_mcdm_r5.csv', skip_header=True, delimiter=',')  
mcdm_r6 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_mcdm_r6.csv', skip_header=True, delimiter=',')  
mcdm_r7 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_mcdm_r7.csv', skip_header=True, delimiter=',')  
mcdm_r8 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_mcdm_r8.csv', skip_header=True, delimiter=',')  
mcdm_r9 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_mcdm_r9.csv', skip_header=True, delimiter=',')  
mcdm_r10 = np.genfromtxt('/home/pulver/Desktop/mcdm_avg_r/result_gs_mcdm_r0.csv', skip_header=True, delimiter=',')  

mcdm_list = [mcdm_r1, mcdm_r2, mcdm_r3, mcdm_r4, mcdm_r5, mcdm_r6, mcdm_r7, mcdm_r8, mcdm_r9, mcdm_r10]
wAVG_list = [wAVG_r1, wAVG_r2, wAVG_r3, wAVG_r4, wAVG_r5, wAVG_r6, wAVG_r7, wAVG_r8, wAVG_r9, wAVG_r10]

final_mcdm_matrix = np.zeros(shape=(len(param_list), len(param_list)))
final_wAVG_matrix = np.zeros(shape=(len(param_list), len(param_list)))
comparison_matrix = np.zeros(shape=(len(param_list), len(param_list)))
counter = 0
wAVG_mean_list = []
wAVG_best_list = []
mcdm_mean_list = []
mcdm_best_list = []
for mcdm in mcdm_list:
    # print("Original: ", mcdm.shape)
    # Keep only [w_info_gain, w_travel_distance, travelledDistance]
    mcdm = np.delete(mcdm, [2,3,4,5,6,7,8,9,11], axis=1)
    # print("Slim: ", mcdm.shape)
    # Order the two arrays based on w_info_gain
    mcdm_sorted = sortData(mcdm)
    # print("Sorted: ", mcdm_sorted.shape)
    # Remove also the criteria weight
    mcdm_matrix = np.delete(mcdm_sorted, [0,1], axis=1)
    # print("Matrix: ", mcdm_matrix.shape)
    # Reshape into a 2D matrix
    mcdm_matrix = np.reshape(mcdm_matrix, (len(param_list), len(param_list) ))
    # print("Reshaped: ", mcdm_matrix.shape)
    final_mcdm_matrix = final_mcdm_matrix +  mcdm_matrix
    mcdm_best_list.append(np.min(mcdm_matrix))
    mcdm_mean_list.append(np.mean(mcdm_matrix))
    # np.savetxt('/tmp/clean_mcdm' + str(counter) + '.txt', mcdm_matrix)
    counter += 1
# exit(0)

for wAVG in wAVG_list:
    # Keep only [w_info_gain, w_travel_distance, travelledDistance]
    wAVG = np.delete(wAVG, [2,3,4,5,6,7,8,9,11], axis=1)
    # Order the two arrays based on w_info_gain
    wAVG_sorted = sortData(wAVG)
    # Remove also the criteria weight
    wAVG_matrix = np.delete(wAVG_sorted, [0,1], axis=1)
    # Reshape into a 2D matrix
    wAVG_matrix = np.reshape(wAVG_matrix, (len(param_list), len(param_list)))
    final_wAVG_matrix = final_wAVG_matrix + wAVG_matrix
    wAVG_best_list.append(np.min(wAVG_matrix))
    wAVG_mean_list.append(np.mean(wAVG_matrix))


# Normalize the matrix
final_mcdm_matrix /= len(mcdm_list)
final_wAVG_matrix /= len(wAVG_list)

# for i in range(0, len(param_list)):
#     for j in range(0, len(param_list)):
#         value = 0 if final_wAVG_matrix[i,j] < final_mcdm_matrix[i,j] else 1
#         comparison_matrix[i, j] = value
comparison_matrix = final_wAVG_matrix - final_mcdm_matrix
comparison_matrix[comparison_matrix < 0] = -1
comparison_matrix[comparison_matrix > 0] = 1
# comparison_matrix = (comparison_matrix - np.min(comparison_matrix)) / (np.max(comparison_matrix) - np.min(comparison_matrix))
np.savetxt('/tmp/clean_wAVG.txt', final_wAVG_matrix)
np.savetxt('/tmp/clean_mcdm.txt', final_mcdm_matrix)
np.savetxt('/tmp/comparison.txt', comparison_matrix)

# Print some statistics data
print("[wAVG] Best mean: ", np.min(wAVG_mean_list))
print("[wAVG] mean[std]: {}[{}]".format(np.mean(wAVG_mean_list), np.std(wAVG_mean_list)))
print("[wAVG] Top10 - mean[std]: {}[{}]".format(np.mean(wAVG_best_list), np.std(wAVG_best_list)))
print("[mcdm] Best mean: ", np.min(mcdm_mean_list))
print("[mcdm] mean[std]: {}[{}]".format(np.mean(mcdm_mean_list), np.std(mcdm_mean_list)))
print("[mcdm] Top10 - mean[std]: {}[{}]".format(np.mean(mcdm_best_list), np.std(mcdm_best_list)))


# Plot the matrix
fig = plt.figure()
ax1 = fig.add_subplot(1,3,1)
ax1.set_aspect('equal')
plt.imshow(np.log(final_wAVG_matrix), interpolation='nearest', cmap=plt.cm.ocean)
plt.colorbar()
plt.ylabel("w_info_gain")
plt.xlabel("w_travel_distance")
# a = plt.gca()
plt.yticks(np.arange(0, len(param_list), 1.0))
ax1.yaxis.set_major_formatter(mpl.ticker.FuncFormatter(mjrFormatter))
plt.xticks(np.arange(0, len(param_list), 1.0))
ax1.xaxis.set_major_formatter(mpl.ticker.FuncFormatter(mjrFormatter))
plt.title("TravelledDistance [wAVG] \n best:{}({})".format(np.min(final_wAVG_matrix), np.std(final_wAVG_matrix)))
# plt.show()

ax2 = fig.add_subplot(1,3,2)
ax2.set_aspect('equal')
plt.imshow(np.log(final_mcdm_matrix), interpolation='nearest', cmap=plt.cm.ocean)
plt.colorbar()
plt.ylabel("w_info_gain")
plt.xlabel("w_travel_distance")
plt.title("TravelledDistance [MCDM] \n best:{}({})".format(np.min(final_mcdm_matrix), np.std(final_mcdm_matrix)))
plt.yticks(np.arange(0, len(param_list), 1.0))
ax2.yaxis.set_major_formatter(mpl.ticker.FuncFormatter(mjrFormatter))
plt.xticks(np.arange(0, len(param_list), 1.0))
ax2.xaxis.set_major_formatter(mpl.ticker.FuncFormatter(mjrFormatter))
# plt.show()


ax3 = fig.add_subplot(1,3,3)
ax3.set_aspect('equal')
plt.imshow(comparison_matrix, interpolation='nearest', cmap=plt.cm.ocean)
plt.colorbar()
plt.ylabel("w_info_gain")
plt.xlabel("w_travel_distance")
plt.title("wAVG[green] - MCDM[white]")
plt.yticks(np.arange(0, len(param_list), 1.0))
ax3.yaxis.set_major_formatter(mpl.ticker.FuncFormatter(mjrFormatter))
plt.xticks(np.arange(0, len(param_list), 1.0))
ax3.xaxis.set_major_formatter(mpl.ticker.FuncFormatter(mjrFormatter))
plt.show()

# new_fig = plt.figure()
# ax3 = new_fig.add_subplot(1,1,1)
# ax3.set_aspect('equal')
# plt.imshow(comparison_matrix, interpolation='nearest', cmap=plt.cm.ocean)
# plt.colorbar()
# plt.ylabel("w_info_gain")
# plt.xlabel("w_travel_distance")
# # a = plt.gca()
# plt.yticks(np.arange(0, len(param_list), 1.0))
# ax3.yaxis.set_major_formatter(mpl.ticker.FuncFormatter(mjrFormatter))
# plt.xticks(np.arange(0, len(param_list), 1.0))
# ax3.xaxis.set_major_formatter(mpl.ticker.FuncFormatter(mjrFormatter))
# plt.title("Comparison wAVG[green] vs MCDM[white]")
# plt.show()



# plt.bar(x=wAVG_sorted[:,0]-0.0025, height=wAVG_sorted[:,2], width=0.005, label='wAVG')
# plt.bar(x=mcdm_sorted[:,0]+0.0025, height=mcdm_sorted[:,2], width=0.005, label='mcdm')
# plt.legend()
# plt.xlabel("w_info_gain")
# plt.ylabel("TravelledDistance[m]")
# plt.title("Travelled distance: MCDM vs wAVG")
# plt.show()
