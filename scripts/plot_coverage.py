import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker


# 1) Load all the data for the 5 environments
# Orebro
orebro_mcdm_data = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/mcdm/logic_exps/orebro/coverage_mcdm_orebro_33.csv", "rb"), delimiter=",", skiprows=1)
orebro_random_frontier = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/random_frontier/orebro/coverage_random_frontier_orebro_1.csv", "rb"), delimiter=",", skiprows=1)
orebro_random_walk = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/random_walk/orebro/coverage_random_walk_orebro_1.csv", "rb"), delimiter=",", skiprows=1)
# ILIAD
iliad_mcdm_data = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/mcdm/logic_exps/ILIAD/coverage_mcdm_ILIAD_11.csv", "rb"), delimiter=",", skiprows=1)
iliad_random_frontier = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/random_frontier/ILIAD/coverage_random_frontier_iliad_1.csv", "rb"), delimiter=",", skiprows=1)
iliad_random_walk = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/random_walk/ILIAD/coverage_random_walk_iliad_1.csv", "rb"), delimiter=",", skiprows=1)
# INB_3123
inb3123_mcdm_data = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/mcdm/logic_exps/inb_3123/coverage_mcdm_inb_3123_0.csv", "rb"), delimiter=",", skiprows=1)
inb3123_random_frontier = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/random_frontier/inb_3123/coverage_random_frontier_inb3123_1.csv", "rb"), delimiter=",", skiprows=1)
inb3123_random_walk = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/random_walk/inb_3123/coverage_random_walk_inb3123_1.csv", "rb"), delimiter=",", skiprows=1)
# INB_ATRIUM
inb_atrium_mcdm_data = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/mcdm/logic_exps/inb_atrium/coverage_mcdm_inb_atrium_14.csv", "rb"), delimiter=",", skiprows=1)
inb_atrium_random_frontier = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/random_frontier/inb_atrium/coverage_random_frontier_inb_atrium_1.csv", "rb"), delimiter=",", skiprows=1)
inb_atrium_random_walk = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/random_walk/inb_atrium/coverage_random_walk_inb_atrium_1.csv", "rb"), delimiter=",", skiprows=1)
# INB_eng
inb_eng_mcdm_data = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/mcdm/logic_exps/inb_eng/coverage_mcdm_inb_eng_19.csv", "rb"), delimiter=",", skiprows=1)
inb_eng_random_frontier = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/random_frontier/inb_eng/coverage_random_frontier_inb_eng_1.csv", "rb"), delimiter=",", skiprows=1)
inb_eng_random_walk = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/random_walk/inb_eng/coverage_random_walk_inb_eng_1.csv", "rb"), delimiter=",", skiprows=1)


# Remove the first 4 columns containing the weights value
orebro_mcdm_data = orebro_mcdm_data[:, -2:]
orebro_random_frontier = orebro_random_frontier[:, -2:]
orebro_random_walk = orebro_random_walk[:, -2:]
iliad_mcdm_data = iliad_mcdm_data[:, -2:]
iliad_random_frontier = iliad_random_frontier[:, -2:]
iliad_random_walk = iliad_random_walk[:, -2:]
inb3123_mcdm_data = inb3123_mcdm_data[:, -2:]
inb3123_random_frontier = inb3123_random_frontier[:, -2:]
inb3123_random_walk = inb3123_random_walk[:, -2:]
inb_atrium_mcdm_data = inb_atrium_mcdm_data[:, -2:]
inb_atrium_random_frontier = inb_atrium_random_frontier[:, -2:]
inb_atrium_random_walk = inb_atrium_random_walk[:, -2:]
inb_eng_mcdm_data = inb_eng_mcdm_data[:, -2:]
inb_eng_random_frontier = inb_eng_random_frontier[:, -2:]
inb_eng_random_walk = inb_eng_random_walk[:, -2:]


# fig = plt.figure(figsize=(5, 2), dpi=300)
# ax = fig.add_subplot(111)
#
# # plt.plot(orebro_mcdm_data[:, 0], orebro_mcdm_data[:, 1], 'r',
# #          orebro_random_frontier[:, 0], orebro_random_frontier[:, 1], 's',
# #          orebro_random_walk[:, 0], orebro_random_walk[:, 1], 'g')
#
# ax.plot(orebro_mcdm_data[:, 0], orebro_mcdm_data[:, 1], label='MCDM')
# ax.plot(orebro_random_frontier[:, 0], orebro_random_frontier[:, 1], label='RandomFrontier')
# ax.plot(orebro_random_walk[:, 0], orebro_random_walk[:, 1], label='RandomWalk')
#
# plt.legend(loc='best', fontsize='small')
# plt.show()

# plt.figure()
fig = plt.figure(figsize=(15, 2.0))#, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(1, 5, sharey=True)
ax = fig.add_subplot(111)  # The big subplot
ax1 = fig.add_subplot(151)
ax2 = fig.add_subplot(152)
ax3 = fig.add_subplot(153)
ax4 = fig.add_subplot(154)
ax5 = fig.add_subplot(155)
ax1.get_shared_y_axes().join(ax1, ax2)
ax1.get_shared_y_axes().join(ax1, ax3)
ax1.get_shared_y_axes().join(ax1, ax4)
ax1.get_shared_y_axes().join(ax1, ax5)
ax2.set_yticklabels([])
ax3.set_yticklabels([])
ax4.set_yticklabels([])
ax5.set_yticklabels([])
# Turn off axis lines and ticks of the big subplot
ax.spines['top'].set_color('none')
ax.spines['bottom'].set_color('none')
ax.spines['left'].set_color('none')
ax.spines['right'].set_color('none')
ax.tick_params(direction='in', labelsize=0.0, labelcolor='w', top='off', bottom='off', left='off', right='off')
# Set common labels
ax.set_ylabel('Coverage [%]')#, fontsize='x-large')
ax.set_xlabel('Robot Configurations')#, fontsize='x-large')
ax.yaxis.set_label_coords(-0.05, 0.5)
ax.xaxis.set_label_coords(0.5, -0.3)


# ax1 = plt.subplot(151)
ax1.plot(orebro_mcdm_data[:, 0], orebro_mcdm_data[:, 1], 'r', label='NBS')
ax1.plot(orebro_random_frontier[:, 0], orebro_random_frontier[:, 1], 'b', label='RandomFrontier')
ax1.plot(orebro_random_walk[:, 0], orebro_random_walk[:, 1], 'g', label='RandomWalk')
ax1.set_title('Orebro', fontdict={'fontsize': 'small'})
# ax1.set_xticklabels(['0','','100','','200','','300','','400'])
loc = plticker.MultipleLocator(base=100.0) # this locator puts ticks at regular intervals
ax1.xaxis.set_major_locator(loc)

# ax2 = plt.subplot(152, sharey=ax1)
ax2.plot(iliad_mcdm_data[:, 0], iliad_mcdm_data[:, 1], 'r',
         iliad_random_frontier[:, 0], iliad_random_frontier[:, 1], 'b',
         iliad_random_walk[:, 0], iliad_random_walk[:, 1], 'g')
ax2.set_title('MCFM', fontdict={'fontsize': 'small'})
loc = plticker.MultipleLocator(base=100.0) # this locator puts ticks at regular intervals
ax2.xaxis.set_major_locator(loc)

# ax3 = plt.subplot(153, sharey=ax1)
ax3.plot(inb3123_mcdm_data[:, 0], inb3123_mcdm_data[:, 1], 'r',
         inb3123_random_frontier[:, 0], inb3123_random_frontier[:, 1], 'b',
         inb3123_random_walk[:, 0], inb3123_random_walk[:, 1], 'g')
ax3.set_title('INB3123', fontdict={'fontsize': 'small'})
loc = plticker.MultipleLocator(base=400.0) # this locator puts ticks at regular intervals
ax3.xaxis.set_major_locator(loc)

# ax4 = plt.subplot(154, sharey=ax1)
ax4.plot(inb_eng_mcdm_data[:, 0], inb_eng_mcdm_data[:, 1], 'r',
         inb_eng_random_frontier[:, 0], inb_eng_random_frontier[:, 1], 'b',
         inb_eng_random_walk[:, 0], inb_eng_random_walk[:, 1], 'g')
ax4.set_title('INB_Eng', fontdict={'fontsize': 'small'})
loc = plticker.MultipleLocator(base=500.0) # this locator puts ticks at regular intervals
ax4.xaxis.set_major_locator(loc)

# ax5 = plt.subplot(155, sharey=ax1)
ax5.plot(inb_atrium_mcdm_data[:, 0], inb_atrium_mcdm_data[:, 1], 'r',
         inb_atrium_random_frontier[:, 0], inb_atrium_random_frontier[:, 1], 'b',
         inb_atrium_random_walk[:, 0], inb_atrium_random_walk[:, 1], 'g')
ax5.set_title('INB_Atrium', fontdict={'fontsize': 'small'})
loc = plticker.MultipleLocator(base=400.0) # this locator puts ticks at regular intervals
ax5.xaxis.set_major_locator(loc)

# fig.legend(["MCDM", "RandomFrontier", "RandomWalk"], loc = (0.5, 0), ncol=5,)
ax1.legend(loc=(2.15, 1.2), fontsize='x-small', ncol=3)
plt.show()

# plt.savefig('common_labels.png', dpi=300)
