import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker


# 1) Load all the data for the 5 environments
# Orebro
# mcdm_data_1            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_mcdm_orebro_35_1.csv", "rb"), delimiter=",", skiprows=1)
# mcdm_data_2            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_mcdm_orebro_35_2.csv", "rb"), delimiter=",", skiprows=1)
# mcdm_data_3            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_mcdm_orebro_35_3.csv", "rb"), delimiter=",", skiprows=1)
# stachniss_data_1            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_stachniss_orebro_1.csv", "rb"), delimiter=",", skiprows=1)
# stachniss_data_2            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_stachniss_orebro_2.csv", "rb"), delimiter=",", skiprows=1)
# stachniss_data_3            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_stachniss_orebro_3.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_1 = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_frontier_orebro_1.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_2 = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_frontier_orebro_2.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_3 = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_frontier_orebro_3.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_1     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_walk_orebro_1.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_2     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_walk_orebro_2.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_3     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_walk_orebro_3.csv", "rb"), delimiter=",", skiprows=1)
# NCFM
mcdm_data_1            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_mcdm_ncfm_28_1.csv", "rb"), delimiter=",", skiprows=1)
mcdm_data_2            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_mcdm_ncfm_28_2.csv", "rb"), delimiter=",", skiprows=1)
mcdm_data_3            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_mcdm_ncfm_28_3.csv", "rb"), delimiter=",", skiprows=1)
stachniss_data_1       = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_stachniss_ncfm_1.csv", "rb"), delimiter=",", skiprows=1)
stachniss_data_2       = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_stachniss_ncfm_2.csv", "rb"), delimiter=",", skiprows=1)
stachniss_data_3       = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_stachniss_ncfm_3.csv", "rb"), delimiter=",", skiprows=1)
random_frontier_data_1 = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_frontier_ncfm_1.csv", "rb"), delimiter=",", skiprows=1)
random_frontier_data_2 = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_frontier_ncfm_2.csv", "rb"), delimiter=",", skiprows=1)
random_frontier_data_3 = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_frontier_ncfm_3.csv", "rb"), delimiter=",", skiprows=1)
random_walk_data_1     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_walk_ncfm_1.csv", "rb"), delimiter=",", skiprows=1)
random_walk_data_2     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_walk_ncfm_2.csv", "rb"), delimiter=",", skiprows=1)
random_walk_data_3     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_walk_ncfm_3.csv", "rb"), delimiter=",", skiprows=1)
# INB3123
# mcdm_data_1            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_mcdm_inb_atrium_28_1.csv", "rb"), delimiter=",", skiprows=1)
# mcdm_data_2            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_mcdm_inb_atrium_28_2.csv", "rb"), delimiter=",", skiprows=1)
# mcdm_data_3            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_mcdm_inb_atrium_28_3.csv", "rb"), delimiter=",", skiprows=1)
# stachniss_data_1            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_stachniss_inb_atrium_1.csv", "rb"), delimiter=",", skiprows=1)
# stachniss_data_2            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_stachniss_inb_atrium_2.csv", "rb"), delimiter=",", skiprows=1)
# stachniss_data_3            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_stachniss_inb_atrium_3.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_1 = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_frontier_inb_atrium_1.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_2 = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_frontier_inb_atrium_2.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_3 = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_frontier_inb_atrium_3.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_1     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_walk_inb_atrium_1.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_2     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_walk_inb_atrium_2.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_3     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_walk_inb_atrium_3.csv", "rb"), delimiter=",", skiprows=1)
# # # INB_ENG
# stachniss_data_1            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_mcdm_inb_eng_1.csv", "rb"), delimiter=",", skiprows=1)
# stachniss_data_2            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_mcdm_inb_eng_2.csv", "rb"), delimiter=",", skiprows=1)
# stachniss_data_3            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_mcdm_inb_eng_3.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_1 = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_frontier_inb_eng_1.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_2 = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_frontier_inb_eng_2.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_3 = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_frontier_inb_eng_3.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_1     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_walk_inb_eng_1.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_2     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_walk_inb_eng_2.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_3     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_walk_inb_eng_3.csv", "rb"), delimiter=",", skiprows=1)
# # INB_Atrium
# stachniss_data_1            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_mcdm_inb_3123_1.csv", "rb"), delimiter=",", skiprows=1)
# stachniss_data_2            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_mcdm_inb_3123_2.csv", "rb"), delimiter=",", skiprows=1)
# stachniss_data_3            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_mcdm_inb_3123_3.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_1 = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_frontier_inb3123_1.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_2 = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_frontier_inb3123_2.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier_data_3 = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_frontier_inb3123_3.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_1     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_walk_inb3123_1.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_2     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_walk_inb3123_2.csv", "rb"), delimiter=",", skiprows=1)
# random_walk_data_3     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/coverage_travel/coverage_random_walk_inb3123_3.csv", "rb"), delimiter=",", skiprows=1)

# Remove the first 4 columns containing the weights value leaving <configuration, coverage, TravelledDistance>
mcdm_data_1 = mcdm_data_1[:, -3:]
mcdm_data_2 = mcdm_data_2[:, -3:]
mcdm_data_3 = mcdm_data_3[:, -3:]
stachniss_data_1 = stachniss_data_1[:, -3:]
stachniss_data_2 = stachniss_data_2[:, -3:]
stachniss_data_3 = stachniss_data_3[:, -3:]
random_frontier_data_1 = random_frontier_data_1[:, -3:]
random_frontier_data_2 = random_frontier_data_2[:, -3:]
random_frontier_data_3 = random_frontier_data_3[:, -3:]
random_walk_data_1 = random_walk_data_1[:, -3:]
random_walk_data_2 = random_walk_data_2[:, -3:]
random_walk_data_3 = random_walk_data_3[:, -3:]
# Remove the second colum (coverage) leaving <configuration, TravelledDistance>
mcdm_data_1 = np.delete(mcdm_data_1, 1, 1)
mcdm_data_2 = np.delete(mcdm_data_2, 1, 1)
mcdm_data_3 = np.delete(mcdm_data_3, 1, 1)
stachniss_data_1 = np.delete(stachniss_data_1, 1, 1)
stachniss_data_2 = np.delete(stachniss_data_2, 1, 1)
stachniss_data_3 = np.delete(stachniss_data_3, 1, 1)
random_frontier_data_1 = np.delete(random_frontier_data_1, 1, 1)
random_frontier_data_2 = np.delete(random_frontier_data_2, 1, 1)
random_frontier_data_3 = np.delete(random_frontier_data_3, 1, 1)
random_walk_data_1 = np.delete(random_walk_data_1, 1, 1)
random_walk_data_2 = np.delete(random_walk_data_2, 1, 1)
random_walk_data_3 = np.delete(random_walk_data_3, 1, 1)

# Remove duplicare rows
unique_1 = np.unique(mcdm_data_1, axis=0)
unique_2 = np.unique(mcdm_data_2, axis=0)
unique_3 = np.unique(mcdm_data_3, axis=0)
unique_stachniss_1 = np.unique(stachniss_data_1, axis=0)
unique_stachniss_2 = np.unique(stachniss_data_2, axis=0)
unique_stachniss_3 = np.unique(stachniss_data_3, axis=0)
unique_rf_1 = np.unique(random_frontier_data_1, axis=0)
unique_rf_2 = np.unique(random_frontier_data_2, axis=0)
unique_rf_3 = np.unique(random_frontier_data_3, axis=0)
unique_rw_1 = np.unique(random_walk_data_1, axis=0)
unique_rw_2 = np.unique(random_walk_data_2, axis=0)
unique_rw_3 = np.unique(random_walk_data_3, axis=0)

# print(mcdm_data_1.shape)
# print(random_frontier_data_1.shape)
# print(random_walk_data_1.shape)

# Make the trajectories long the same length and store it in a single array
len_arr = [unique_1.shape[0], unique_2.shape[0], unique_3.shape[0]]
max_len_index = np.argmax(len_arr)
final = np.zeros(shape=(np.max(len_arr), 5))
arr_list = [unique_1, unique_2, unique_3]
for i in range(3):
    while arr_list[i].shape[0] < np.max(len_arr):
        arr_list[i] = np.vstack([arr_list[i], arr_list[i][-1,:]])
    final[:,i] = arr_list[i][:,1]

len_stachniss_arr = [unique_stachniss_1.shape[0], unique_stachniss_2.shape[0], unique_stachniss_3.shape[0]]
max_len_index = np.argmax(len_stachniss_arr)
final_stachniss = np.zeros(shape=(np.max(len_stachniss_arr), 5))
arr_stachniss_list = [unique_stachniss_1, unique_stachniss_2, unique_stachniss_3]
for i in range(3):
    while arr_stachniss_list[i].shape[0] < np.max(len_stachniss_arr):
        arr_stachniss_list[i] = np.vstack([arr_stachniss_list[i], arr_stachniss_list[i][-1,:]])
    final_stachniss[:,i] = arr_stachniss_list[i][:,1]

len_rf_arr = [unique_rf_1.shape[0], unique_rf_2.shape[0], unique_rf_3.shape[0]]
max_len_index = np.argmax(len_rf_arr)
final_rf = np.zeros(shape=(np.max(len_rf_arr), 5))
arr_rf_list = [unique_rf_1, unique_rf_2, unique_rf_3]
for i in range(3):
    while arr_rf_list[i].shape[0] < np.max(len_rf_arr):
        arr_rf_list[i] = np.vstack([arr_rf_list[i], arr_rf_list[i][-1,:]])
    final_rf[:,i] = arr_rf_list[i][:,1]

len_rw_arr = [unique_rw_1.shape[0], unique_rw_2.shape[0], unique_rw_3.shape[0]]
max_len_index = np.argmax(len_rw_arr)
final_rw = np.zeros(shape=(np.max(len_rw_arr), 5))
arr_rw_list = [unique_rw_1, unique_rw_2, unique_rw_3]
for i in range(3):
    while arr_rw_list[i].shape[0] < np.max(len_rw_arr):
        arr_rw_list[i] = np.vstack([arr_rw_list[i], arr_rw_list[i][-1,:]])
    final_rw[:,i] = arr_rw_list[i][:,1]

# Calculate average and std dev from the trajectories
final[:,3] = np.average(final[:,0:3], axis=1)
final[:,4] = np.std(final[:,0:3], axis=1)
final_stachniss[:,3] = np.average(final_stachniss[:,0:3], axis=1)
final_stachniss[:,4] = np.std(final_stachniss[:,0:3], axis=1)
final_rf[:,3] = np.average(final_rf[:,0:3], axis=1)
final_rf[:,4] = np.std(final_rf[:,0:3], axis=1)
final_rw[:,3] = np.average(final_rw[:,0:3], axis=1)
final_rw[:,4] = np.std(final_rw[:,0:3], axis=1)
# print(final[:,4])

# Plot the trajectories
x = np.arange(1, final.shape[0] + 1, 1)
print("x: ", len(x))
print(final.shape[0])
plt.plot(x, final[:, 3],  color='#CC4F1B', label='NBS')
plt.fill_between(x, final[:, 3]-final[:, 4], final[:, 3]+final[:, 4], alpha=0.5, edgecolor='#CC4F1B', facecolor='#FF9848')

x = np.arange(1, final_stachniss.shape[0] + 1, 1)
print("x: ", len(x))
print(final_stachniss.shape[0])
plt.plot(x, final_stachniss[:, 3],  color='#410093', label='Stachniss et al.')
plt.fill_between(x, final_stachniss[:, 3]-final_stachniss[:, 4], final_stachniss[:, 3]+final_stachniss[:, 4], alpha=0.5, edgecolor='#5000ac', facecolor='#7201ff')

x = np.arange(1, final_rf.shape[0] + 1, 1)
print("x: ", len(x))
print(final_rf.shape[0])
plt.plot(x, final_rf[:, 3],  color='#1B2ACC', label='RandomFrontier')
plt.fill_between(x, final_rf[:, 3]-final_rf[:, 4], final_rf[:, 3]+final_rf[:, 4], alpha=0.5, edgecolor='#1B2ACC', facecolor='#089FFF')

x = np.arange(1, final_rw.shape[0] + 1, 1)
print("x: ", len(x))
print(final_rw.shape[0])
plt.plot(x, final_rw[:, 3],  color='#3F7F4C', label='RandomWalk')
plt.fill_between(x, final_rw[:, 3]-final_rw[:, 4], final_rw[:, 3]+final_rw[:, 4], alpha=0.5, edgecolor='#3F7F4C', facecolor='#7EFF99')

# Draw vertical lines at the enf of the plot
print(final[-1,3])
plt.axhline(y=final[-1,3], color='#CC4F1B', linestyle="--")
plt.axhline(y=final_rf[-1,3], color='#1B2ACC', linestyle="--")

plt.legend( ncol=3, bbox_to_anchor=(1, 1.10))
axes = plt.gca()
axes.set_xlim([0, 700])
axes.set_ylim([0, 5000])
axes.set_xlabel("Robot Configuration")
axes.set_ylabel("Travelled Distance")
plt.show()