import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker

# INB_3123
inb3123_mcdm_data = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/gazebo_simulations/mcdm/coverage_mcdm_2.csv", "rb"), delimiter=",", skiprows=1)
inb3123_random_frontier = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/gazebo_simulations/random_frontier/gazebo_inb3123_coverage_v2.csv", "rb"), delimiter=",", skiprows=1)
inb3123_random_walk = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/gazebo_simulations/random_walk/gazebo_inb3123_coverage_v2.csv", "rb"), delimiter=",", skiprows=1)
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10,6))
# fig.suptitle('Map coverage in Gazebo')
# Labels to use for each line
line_labels = ["NBS", "RandomFrontier", "RandomWalk"]
# # Coverage
l1 = ax1.plot(inb3123_mcdm_data[:,0], inb3123_mcdm_data[:,1], color='b')[0]
l2 = ax1.plot(inb3123_random_frontier[:,0], inb3123_random_frontier[:,1], color='g')[0]
l3 = ax1.plot(inb3123_random_walk[:,0], inb3123_random_walk[:,1], color='r')[0]
#RFID Coverage
l1 = ax2.plot(inb3123_mcdm_data[:,0], inb3123_mcdm_data[:,2], color='b')[0]
l2 = ax2.plot(inb3123_random_frontier[:,0], inb3123_random_frontier[:,2], color='g')[0]
l3 = ax2.plot(inb3123_random_walk[:,0], inb3123_random_walk[:,2], color='r')[0]
# Add legend
# plt.legend(loc=9, ncol=3)
fig.legend([l1, l2, l3],              # List of the line objects
           labels= line_labels,       # The labels for each line
           loc="upper center",        # Position of the legend
           borderaxespad=0.1,         # Add little spacing around the legend box
           ncol=3)      # Title for the legend

# Add titles
# plt.title("Map exploration in Gazebo", loc='center', fontsize=12, fontweight=0, color='black')
ax1.set_xlabel("Robot configuration")
ax1.set_ylabel("Coverage [%]")

ax2.set_xlabel("Robot configuration")
ax2.set_ylabel("RFID Coverage [%]")


plt.show()