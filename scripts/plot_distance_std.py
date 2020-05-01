import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker


# Normalization factors
res_inb3123 = 0.5
res_inbeng = 0.25
res_inbatrium = 0.33
res_ncfm = 0.25
res_orebro = 0.5

res = res_inbatrium

# 1) Load all the data for the 5 environments
# Orebro
# mcdm_data_1            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/orebro/coverage_orebro_mcdm_7_r1.csv", "rb"), delimiter=",", skiprows=1)
# mcdm_data_2            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/orebro/coverage_orebro_mcdm_7_r2.csv", "rb"), delimiter=",", skiprows=1)
# mcdm_data_3            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/orebro/coverage_orebro_mcdm_7_r0.csv", "rb"), delimiter=",", skiprows=1)
# # stachniss_data_1            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/coverage_stachniss_orebro_1.csv", "rb"), delimiter=",", skiprows=1)
# # stachniss_data_2            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/coverage_stachniss_orebro_2.csv", "rb"), delimiter=",", skiprows=1)
# # stachniss_data_3            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/coverage_stachniss_orebro_3.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_1 = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_frontier/orebro/coverage_orebro_mcdm_0_r1.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_2 = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_frontier/orebro/coverage_orebro_mcdm_0_r2.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_3 = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_frontier/orebro/coverage_orebro_mcdm_0_r0.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_1     = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_walk/orebro/coverage_orebro_mcdm_0_r1.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_2     = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_walk/orebro/coverage_orebro_mcdm_0_r2.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_3     = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_walk/orebro/coverage_orebro_mcdm_0_r0.csv", "rb"), delimiter=",", skiprows=1)
# NCFM
# mcdm_data_1            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/ncfm/coverage_ncfm_mcdm_7_r0.csv", "rb"), delimiter=",", skiprows=1)
# mcdm_data_2            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/ncfm/coverage_ncfm_mcdm_7_r1.csv", "rb"), delimiter=",", skiprows=1)
# mcdm_data_3            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/ncfm/coverage_ncfm_mcdm_7_r2.csv", "rb"), delimiter=",", skiprows=1)
# # stachniss_data_1       = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/coverage_stachniss_ncfm_1.csv", "rb"), delimiter=",", skiprows=1)
# # stachniss_data_2       = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/coverage_stachniss_ncfm_2.csv", "rb"), delimiter=",", skiprows=1)
# # stachniss_data_3       = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/coverage_stachniss_ncfm_3.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_1 = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_frontier/ncfm/coverage_ncfm_mcdm_0_r0.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_2 = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_frontier/ncfm/coverage_ncfm_mcdm_0_r1.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_3 = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_frontier/ncfm/coverage_ncfm_mcdm_0_r2.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_1     = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_walk/ncfm/coverage_ncfm_mcdm_0_r0.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_2     = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_walk/ncfm/coverage_ncfm_mcdm_0_r2.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_3     = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_walk/ncfm/coverage_ncfm_mcdm_0_r2.csv", "rb"), delimiter=",", skiprows=1)
# INB3123
# mcdm_data_1            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/inb3123/coverage_inb3123_mcdm_7_r0.csv", "rb"), delimiter=",", skiprows=1)
# mcdm_data_2            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/inb3123/coverage_inb3123_mcdm_7_r1.csv", "rb"), delimiter=",", skiprows=1)
# mcdm_data_3            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/inb3123/coverage_inb3123_mcdm_7_r2.csv", "rb"), delimiter=",", skiprows=1)
# # stachniss_data_1            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/coverage_stachniss_inb_3123_1.csv", "rb"), delimiter=",", skiprows=1)
# # stachniss_data_2            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/coverage_stachniss_inb_3123_2.csv", "rb"), delimiter=",", skiprows=1)
# # stachniss_data_3            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/coverage_stachniss_inb_3123_3.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_1 = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_frontier/inb3123/coverage_inb3123_mcdm_0_r0.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_2 = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_frontier/inb3123/coverage_inb3123_mcdm_0_r1.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_3 = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_frontier/inb3123/coverage_inb3123_mcdm_0_r2.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_1     = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_walk/inb3123/coverage_inb3123_mcdm_0_r0.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_2     = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_walk/inb3123/coverage_inb3123_mcdm_0_r1.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_3     = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_walk/inb3123/coverage_inb3123_mcdm_0_r2.csv", "rb"), delimiter=",", skiprows=1)
# # # INB_ENG
# mcdm_data_1            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/inbeng/coverage_inbeng_mcdm_7_r1.csv", "rb"), delimiter=",", skiprows=1)
# mcdm_data_2            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/inbeng/coverage_inbeng_mcdm_7_r2.csv", "rb"), delimiter=",", skiprows=1)
# mcdm_data_3            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/inbeng/coverage_inbeng_mcdm_7_r0.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_1 = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_frontier/inbeng/coverage_inbeng_mcdm_0_r1.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_2 = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_frontier/inbeng/coverage_inbeng_mcdm_0_r2.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_3 = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_frontier/inbeng/coverage_inbeng_mcdm_0_r0.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_1     = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_walk/inbeng/coverage_inbeng_mcdm_0_r0.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_2     = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_walk/inbeng/coverage_inbeng_mcdm_0_r1.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_3     = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_walk/inbeng/coverage_inbeng_mcdm_0_r2.csv", "rb"), delimiter=",", skiprows=1)
# # INB_Atrium
mcdm_data_1            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/inbatrium/coverage_inbatrium_mcdm_7_r0.csv", "rb"), delimiter=",", skiprows=1)
mcdm_data_2            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/inbatrium/coverage_inbatrium_mcdm_7_r1.csv", "rb"), delimiter=",", skiprows=1)
mcdm_data_3            = np.loadtxt(open("/home/pulver/Desktop/mcdm_ral_experiments/inbatrium/coverage_inbatrium_mcdm_7_r2.csv", "rb"), delimiter=",", skiprows=1)
random_frontier_data_1 = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_frontier/inbatrium/coverage_inbatrium_mcdm_0_r1.csv", "rb"), delimiter=",", skiprows=1)
random_frontier_data_2 = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_frontier/inbatrium/coverage_inbatrium_mcdm_0_r2.csv", "rb"), delimiter=",", skiprows=1)
random_frontier_data_3 = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_frontier/inbatrium/coverage_inbatrium_mcdm_0_r0.csv", "rb"), delimiter=",", skiprows=1)
random_walk_data_1     = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_walk/inbatrium/coverage_inbatrium_mcdm_0_r1.csv", "rb"), delimiter=",", skiprows=1)
random_walk_data_2     = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_walk/inbatrium/coverage_inbatrium_mcdm_0_r2.csv", "rb"), delimiter=",", skiprows=1)
random_walk_data_3     = np.loadtxt(open("/home/pulver/Desktop/[std=1]mcdm_ral_experiments/random_walk/inbatrium/coverage_inbatrium_mcdm_0_r0.csv", "rb"), delimiter=",", skiprows=1)


# Remove the first 4 columns containing the weights value leaving <configuration, coverage, TravelledDistance>
mcdm_data_1 = mcdm_data_1[:, -3:]
mcdm_data_2 = mcdm_data_2[:, -3:]
mcdm_data_3 = mcdm_data_3[:, -3:]
# stachniss_data_1 = stachniss_data_1[:, -3:]
# stachniss_data_2 = stachniss_data_2[:, -3:]
# stachniss_data_3 = stachniss_data_3[:, -3:]
random_frontier_data_1 = random_frontier_data_1[:, -3:]
random_frontier_data_2 = random_frontier_data_2[:, -3:]
random_frontier_data_3 = random_frontier_data_3[:, -3:]
random_walk_data_1 = random_walk_data_1[:, -3:]
random_walk_data_2 = random_walk_data_2[:, -3:]
random_walk_data_3 = random_walk_data_3[:, -3:]

# Calculate only unique <config, coverage, distance> entries
# print("Before: ", len(mcdm_data_1), len(mcdm_data_2), len(mcdm_data_3))
unique_1 = np.unique(mcdm_data_1, axis=0)
unique_2 = np.unique(mcdm_data_2, axis=0)
unique_3 = np.unique(mcdm_data_3, axis=0)
unique_rf_1 = np.unique(random_frontier_data_1, axis=0)
unique_rf_2 = np.unique(random_frontier_data_2, axis=0)
unique_rf_3 = np.unique(random_frontier_data_3, axis=0)
unique_rw_1 = np.unique(random_walk_data_1, axis=0)
unique_rw_2 = np.unique(random_walk_data_2, axis=0)
unique_rw_3 = np.unique(random_walk_data_3, axis=0)
# print("After: ", len(unique_1), len(unique_2), len(unique_3))
# print("Last element: ", unique_1[-1,0], unique_2[-1,0], unique_3[-1,0])

# # Remove the second colum (coverage) leaving <configuration, TravelledDistance>
unique_1 = np.delete(unique_1, 1, 1)
unique_2 = np.delete(unique_2, 1, 1)
unique_3 = np.delete(unique_3, 1, 1)
# stachniss_data_1 = np.delete(stachniss_data_1, 1, 1)
# stachniss_data_2 = np.delete(stachniss_data_2, 1, 1)
# stachniss_data_3 = np.delete(stachniss_data_3, 1, 1)
unique_rf_1 = np.delete(unique_rf_1, 1, 1)
unique_rf_2 = np.delete(unique_rf_2, 1, 1)
unique_rf_3 = np.delete(unique_rf_3, 1, 1)
unique_rw_1 = np.delete(unique_rw_1, 1, 1)
unique_rw_2 = np.delete(unique_rw_2, 1, 1)
unique_rw_3 = np.delete(unique_rw_3, 1, 1)


for element in [unique_1, unique_2, unique_3, unique_rf_1, unique_rf_2, unique_rf_3, unique_rw_1, unique_rw_2, unique_rw_3]:
    for i in range(element.shape[0]):
        element[i,0] = i+0


print(len(mcdm_data_1), len(mcdm_data_2), len(mcdm_data_3))
print(len(unique_1), len(unique_2), len(unique_3))


# Make the trajectories long the same length and store it in a single array
len_arr = [unique_1.shape[0], unique_2.shape[0], unique_3.shape[0]]
print(len_arr)
print(np.mean(len_arr), np.std(len_arr))
print(unique_1[-1], unique_2[-1], unique_3[-1])
max_len_index = np.argmax(len_arr)
final = np.zeros(shape=(np.max(len_arr), 5))
arr_list = [unique_1, unique_2, unique_3]
for i in range(3):
    while arr_list[i].shape[0] < np.max(len_arr):
        arr_list[i] = np.vstack([arr_list[i], arr_list[i][-1,:]])
    final[:,i] = arr_list[i][:,1]

# len_stachniss_arr = [unique_stachniss_1.shape[0], unique_stachniss_2.shape[0], unique_stachniss_3.shape[0]]
# max_len_index = np.argmax(len_stachniss_arr)
# final_stachniss = np.zeros(shape=(np.max(len_stachniss_arr), 5))
# arr_stachniss_list = [unique_stachniss_1, unique_stachniss_2, unique_stachniss_3]
# for i in range(3):
#     while arr_stachniss_list[i].shape[0] < np.max(len_stachniss_arr):
#         arr_stachniss_list[i] = np.vstack([arr_stachniss_list[i], arr_stachniss_list[i][-1,:]])
#     final_stachniss[:,i] = arr_stachniss_list[i][:,1]

len_rf_arr = [unique_rf_1.shape[0], unique_rf_2.shape[0], unique_rf_3.shape[0]]
# print(len_rf_arr)
# print(np.mean(len_rf_arr), np.std(len_rf_arr))
# print(unique_rf_1[-1], unique_rf_2[-1], unique_rf_3[-1])
max_len_index = np.argmax(len_rf_arr)
final_rf = np.zeros(shape=(np.max(len_rf_arr), 5))
arr_rf_list = [unique_rf_1, unique_rf_2, unique_rf_3]
for i in range(3):
    while arr_rf_list[i].shape[0] < np.max(len_rf_arr):
        arr_rf_list[i] = np.vstack([arr_rf_list[i], arr_rf_list[i][-1,:]])
    final_rf[:,i] = arr_rf_list[i][:,1]

len_rw_arr = [unique_rw_1.shape[0], unique_rw_2.shape[0], unique_rw_3.shape[0]]
# print(len_rw_arr)
# print(np.mean(len_rw_arr), np.std(len_rw_arr))
max_len_index = np.argmax(len_rw_arr)
final_rw = np.zeros(shape=(np.max(len_rw_arr), 5))
arr_rw_list = [unique_rw_1, unique_rw_2, unique_rw_3]
for i in range(3):
    while arr_rw_list[i].shape[0] < np.max(len_rw_arr):
        arr_rw_list[i] = np.vstack([arr_rw_list[i], arr_rw_list[i][-1,:]])
    final_rw[:,i] = arr_rw_list[i][:,1]

# Normalize in meters
final *= res
final_rf *= res
final_rw *= res

# Calculate average and std dev from the trajectories
final[:,3] = np.average(final[:,0:3], axis=1)
final[:,4] = np.std(final[:,0:3], axis=1)
# final_stachniss[:,3] = np.average(final_stachniss[:,0:3], axis=1)
# final_stachniss[:,4] = np.std(final_stachniss[:,0:3], axis=1)
final_rf[:,3] = np.average(final_rf[:,0:3], axis=1)
final_rf[:,4] = np.std(final_rf[:,0:3], axis=1)
final_rw[:,3] = np.average(final_rw[:,0:3], axis=1)
final_rw[:,4] = np.std(final_rw[:,0:3], axis=1)
# print(final[:,4])

# Truncate the array at the average value
final = final[:int(np.mean(len_arr))]
final_rf = final_rf[:int(np.mean(len_rf_arr))]
final_rw = final_rw[:int(np.mean(len_rw_arr))]

# Plot the trajectories
x = np.arange(1, final.shape[0] + 1, 1)
# print("x: ", len(x))
# print(final.shape[0], final[-1,3], final[-1, 4])
plt.plot(x, final[:, 3],  color='#CC4F1B', label='NBS')
plt.fill_between(x, final[:, 3]-final[:, 4], final[:, 3]+final[:, 4], alpha=0.5, edgecolor='#CC4F1B', facecolor='#FF9848')

# x = np.arange(1, final_stachniss.shape[0] + 1, 1)
# print("x: ", len(x))
# print(final_stachniss.shape[0])
# plt.plot(x, final_stachniss[:, 3],  color='#410093', label='Stachniss et al.')
# plt.fill_between(x, final_stachniss[:, 3]-final_stachniss[:, 4], final_stachniss[:, 3]+final_stachniss[:, 4], alpha=0.5, edgecolor='#5000ac', facecolor='#7201ff')

x = np.arange(1, final_rf.shape[0] + 1, 1)
# print("x: ", len(x))
# print(final_rf.shape[0], final_rf[-1,3], final_rf[-1, 4])
plt.plot(x, final_rf[:, 3],  color='#1B2ACC', label='RandomFrontier')
plt.fill_between(x, final_rf[:, 3]-final_rf[:, 4], final_rf[:, 3]+final_rf[:, 4], alpha=0.5, edgecolor='#1B2ACC', facecolor='#089FFF')

x = np.arange(1, final_rw.shape[0] + 1, 1)
# print("x: ", len(x))
# print(final_rw.shape[0], final_rw[-1,3], final_rw[-1, 4])
plt.plot(x, final_rw[:, 3],  color='#3F7F4C', label='RandomWalk')
plt.fill_between(x, final_rw[:, 3]-final_rw[:, 4], final_rw[:, 3]+final_rw[:, 4], alpha=0.5, edgecolor='#3F7F4C', facecolor='#7EFF99')

# Draw vertical lines at the enf of the plot

plt.axhline(y=final[-1,3], color='#CC4F1B', linestyle="--")
plt.axhline(y=final_rf[-1,3], color='#1B2ACC', linestyle="--")
plt.axhline(y=final_rw[-1,3], color='#3F7F4C', linestyle="--")

# plt.legend( ncol=3, bbox_to_anchor=(1, 1.10))
axes = plt.gca()
axes.set_xlim([0, 700])
axes.set_ylim([0, 1200])
# axes.set_xlabel("Robot Configuration")
# axes.set_ylabel("Travelled Distance")
plt.show()