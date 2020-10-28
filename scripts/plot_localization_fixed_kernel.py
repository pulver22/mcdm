import numpy as np
import matplotlib.pyplot as plt

root = "/home/pulver/Desktop/mcdm_ral_experiments/static/"
kernel_list = [3, 5, 9]
resolution = 0.25
motion_list = [2, 3, 5]
run = 3
total_tag = 10
localization_errors = []
color_list = ['red', 'blue', 'green']

for kernel in kernel_list:
    path = root + "kernel_" + str(kernel) + "x" + str(kernel) + "/"
    
    rows = 5
    cols = 2
    parameters = {'axes.labelsize': 8,
                    'ytick.labelsize': 8, 
                    'xtick.labelsize': 8,
                    'legend.fontsize': 8}
    plt.rcParams.update(parameters)
    fig = plt.figure(figsize=(6,8.5))
    axs = fig.subplots(rows, cols, sharex=True, sharey=True)

    def plotData(data, row, col, axes, color):
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
        axs[row, col].plot(x, final[:,3], label=str(motion), color=color, linewidth=0.8)
        axs[row, col].fill_between(x, final[:, -2]-final[:, -1], final[:, -2]+final[:, -1], alpha=0.1, edgecolor=color, facecolor=color)
        # axs[row, col].legend(loc='upper left')#, fontsize='x-small')
        axs[row, col].set_xlim(left=0, right=max(x))
        axs[row, col].set_ylim(bottom=0, top=10)
        axs[row, col].set_title("Tag-"+str(tag_id), fontsize=8)
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
    for motion_index in range(len(motion_list)):
        motion = motion_list[motion_index]
        tag_single = []
        for row in range(rows):
            for col in range(cols):
                for i in range(run):
                    tag_id = 2*row + col;
                    tmp = np.loadtxt(open(path + str(motion) +"/entropy_" + str(i) + "/distance_" + str(tag_id) + '.csv', "rb"), delimiter=",", skiprows=0)
                    tmp = np.expand_dims(tmp, axis=1)  # Add one column needed later for stacking
                    tmp *= resolution  # Multiple distance by resolution to obtain data in meters
                    tag_single.append(tmp)
                
                plotData(tag_single, row, col, axs, color_list[motion_index])
                tag_single.clear()
            
        
            # x = np.arange(1, tmp.shape[0] + 1, 1)
            # axs[row, col].plot(x, tmp, label="Tag-"+str(tag_id))
            # axs[row, col].legend(loc='upper left')
            # axs[row, col].set_xlim(left=0, right=max(x))
            # axs[row, col].set_ylim(bottom=0, top=10)
            # if (col == 1):
            #     axs[row,col].yaxis.label.set_color(color='white')
    #place legend above plot
    fig.legend(labels= motion_list,       # The labels for each line
                loc="upper center",        # Position of the legend
                borderaxespad=0.1,         # Add little spacing around the legend box
                title="Normal distribution std-dev", 
                title_fontsize=8,
                ncol=3,
                bbox_to_anchor=(0.5, 0.98),
                fontsize=8)      
    plt.subplots_adjust(left=0.07, right=0.93, wspace=0.25, hspace=0.95)
    
    fig.suptitle("Tags localization error with kernel "+str(kernel) + "x" + str(kernel),  y=1.0)#, color='white')
    plt.tight_layout()
    # plt.show()
    fig.savefig(root + "localization_error_kernel" + str(kernel) + ".png", dpi=300)