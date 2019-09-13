import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker

# INB_3123
# mcdm_data       = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/sim_logs/success_gazebo_INB3123_mcdm/coverage_mcdm.csv", "rb"), delimiter=",", skiprows=1)
# random_frontier = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/sim_logs/success_gazebo_INB3123_rf/gazebo_inb3123_coverage_rf_plot.csv", "rb"), delimiter=",", skiprows=1)
# random_walk     = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/sim_logs/success_gazebo_INB3123_rw_5m/gazebo_inb3123_coverage_v2.csv", "rb"), delimiter=",", skiprows=1)
# real            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/sim_logs/success_INB3123/coverage_mcdm.csv", "rb"), delimiter=",", skiprows=1)

# INB_ENG
mcdm_data       = np.loadtxt(open("/home/pulver/Desktop/MCDM/mcdm/inb_eng_coverage_mcdm_final_td06_20cm.csv", "rb"), delimiter=",", skiprows=1)
random_frontier = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/sim_logs/success_gazebo_INB3ENG_rf/gazebo_inb3eng_coverage_v2.csv", "rb"), delimiter=",", skiprows=1)
random_walk     = np.loadtxt(open("/home/pulver/Desktop/MCDM/random_walk/gazebo_ingeng_coverage_v10.csv", "rb"), delimiter=",", skiprows=1)
real            = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/sim_logs/success_INB3ENG/coverage_final.csv", "rb"), delimiter=",", skiprows=1)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8,3.0))
# fig.suptitle('Map coverage in Gazebo')
# Labels to use for each line
line_labels = ["NBS", "RandomFrontier", "RandomWalk", "Real"]


# Remove the first 4 columns containing the weights value
# mcdm_data = mcdm_data[:, -3:]
# random_frontier = random_frontier[:, -3:]
# random_walk = random_walk[:, -3:]
# real = real[:, -2:]

mcdm_data = mcdm_data[:, 1:3]
random_frontier = random_frontier[:, 1:3]
random_walk = random_walk[:, 1:3]
real = real[:, 1:3]

# Remove duplicare rows
unique_1 = np.unique(mcdm_data, axis=0)
unique_2 = np.unique(random_frontier, axis=0)
# unique_3 = np.unique(random_walk, axis=0)
unique_4 = np.unique(real, axis=0)


# unique_1 = mcdm_data
# unique_2 = random_frontier
unique_3 = random_walk
# unique_4 = real

# x = np.arange(1, unique_1.shape[0] + 1, 1)
# # Coverage
l1 = ax1.plot(unique_1[:,0], color='r')[0]
l2 = ax1.plot(unique_2[:,0], color='b')[0]
l3 = ax1.plot(unique_3[:,0], color='g')[0]
l4 = ax1.plot(unique_4[:,0], color='purple')[0]
#RFID Coverage
l1 = ax2.plot(unique_1[:,1], color='r')[0]
l2 = ax2.plot(unique_2[:,1], color='b')[0]
l3 = ax2.plot(unique_3[:,1], color='g')[0]
l4 = ax2.plot(unique_4[:,1], color='purple')[0]

# # # Coverage
# l1 = ax1.plot(mcdm_data[:,0], mcdm_data[:,1], color='b')[0]
# l2 = ax1.plot(random_frontier[:,0], random_frontier[:,1], color='g')[0]
# l3 = ax1.plot(random_walk[:,0], random_walk[:,1], color='r')[0]
# l4 = ax1.plot(real[:,0], real[:,1], color='purple')[0]
# #RFID Coverage
# l1 = ax2.plot(mcdm_data[:,0], mcdm_data[:,2], color='b')[0]
# l2 = ax2.plot(random_frontier[:,0], random_frontier[:,2], color='g')[0]
# l3 = ax2.plot(random_walk[:,0], random_walk[:,2], color='r')[0]
# l4 = ax2.plot(real[:,0], real[:,2], color='purple')[0]
# Add legend
# plt.legend(loc=9, ncol=3)
fig.legend([l1, l2, l3, l4],              # List of the line objects
           labels= line_labels,       # The labels for each line
           loc="upper center",        # Position of the legend
           borderaxespad=0.1,         # Add little spacing around the legend box
           ncol=4)      # Title for the legend
ax1.axvline(x=unique_1.shape[0], color='r', linestyle="--")
ax1.axvline(x=unique_2.shape[0], color='b', linestyle="--")
ax1.axvline(x=unique_3.shape[0], color='g', linestyle="--")
ax1.axvline(x=unique_4.shape[0], color='purple', linestyle="--")

ax2.axvline(x=unique_1.shape[0], color='r', linestyle="--")
ax2.axvline(x=unique_2.shape[0], color='b', linestyle="--")
ax2.axvline(x=unique_3.shape[0], color='g', linestyle="--")
ax2.axvline(x=unique_4.shape[0], color='purple', linestyle="--")

# Add titles
# plt.title("Map exploration in Gazebo", loc='center', fontsize=12, fontweight=0, color='black')
ax1.set_xlabel("Robot configuration")
ax1.set_ylabel("Coverage [%]")

ax2.set_xlabel("Robot configuration")
ax2.set_ylabel("RFID Coverage [%]")

ax1.set_ylim([0, 100])
ax2.set_ylim([0, 1])

plt.show()