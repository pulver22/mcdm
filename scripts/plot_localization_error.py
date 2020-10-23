import numpy as np
import matplotlib.pyplot as plt

root = "/home/pulver/Desktop/mcdm_ral_experiments/inbeng/"
out_path = root + "localization_error.png"
resolution = 0.25

run = 3
total_tag = 10
localization_errors = []


rows = 5
cols = 2
parameters = {'axes.labelsize': 8,
                'ytick.labelsize': 8, 
                'xtick.labelsize': 8,
                'legend.fontsize': 8}
plt.rcParams.update(parameters)
fig = plt.figure(figsize=(6,6))
axs = fig.subplots(rows, cols, sharex=True, sharey=True)
fig.suptitle("Tags localization error")

def plotData(data, row, col, axes):
    # Normalise lenght
    max_len_index = np.max([x.shape[0] for x in tag_single])
    final = np.zeros(shape=(max_len_index, run + 2))
    for i in range(run):
        while tag_single[i].shape[0] < max_len_index:
            tag_single[i] = np.vstack([tag_single[i], tag_single[i][-1,:]])
        final[:,i] = tag_single[i][:,0]
        # Calculate average and std dev 
    final[:,-2] = np.average(final[:,0:run], axis=1)
    final[:,-1] = np.std(final[:,0:run], axis=1)
    #  Plot the trajectories
    x = np.arange(1, final.shape[0] + 1, 1)
    axs[row, col].plot(x, final[:,3], label="Tag-"+str(tag_id), color='#CC4F1B')
    axs[row, col].fill_between(x, final[:, -2]-final[:, -1], final[:, -2]+final[:, -1], alpha=0.5, edgecolor='#CC4F1B', facecolor='#FF9848')
    axs[row, col].legend(loc='upper right')#, fontsize='x-small')
    axs[row, col].set_xlim(left=0, right=max(x))
    axs[row, col].set_ylim(bottom=0, top=10)
    if (col == 1):
        axs[row,col].yaxis.label.set_color(color='white')

for ax in axs.flat:
    ax.set(xlabel='Robot configuration', ylabel='Localization\nerror [m]')
    ax.yaxis.set_label_position('left')
    ax.yaxis.set_ticks([0, 2, 4, 6, 8, 10])
    ax.grid(axis='y', alpha=0.2)
# Hide x labels and tick labels for top plots and y ticks for right plots.
for ax in axs.flat:
    ax.label_outer()

tag_single = []
for row in range(rows):
    for col in range(cols):
        for i in range(run):
            tag_id = 2*row + col;
            tmp = np.loadtxt(open(root + "entropy_" + str(i) + "/distance_" + str(tag_id) + '.csv', "rb"), delimiter=",", skiprows=0)
            tmp = np.expand_dims(tmp, axis=1)  # Add one column needed later for stacking
            tmp *= resolution  # Multiple distance by resolution to obtain data in meters
            tag_single.append(tmp)
        
        plotData(tag_single, row, col, axs)
        tag_single.clear()
        
    
        # x = np.arange(1, tmp.shape[0] + 1, 1)
        # axs[row, col].plot(x, tmp, label="Tag-"+str(tag_id))
        # axs[row, col].legend(loc='upper left')
        # axs[row, col].set_xlim(left=0, right=max(x))
        # axs[row, col].set_ylim(bottom=0, top=10)
        # if (col == 1):
        #     axs[row,col].yaxis.label.set_color(color='white')

plt.tight_layout()
# plt.show()
fig.savefig(out_path, dpi=300)