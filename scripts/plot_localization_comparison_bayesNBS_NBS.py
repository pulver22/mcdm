import numpy as np
import matplotlib.pyplot as plt 

root = '/home/pulver/Desktop/nbs_ecmr/'
experiment = 'moving'
coverage_log = '/coverage_inbeng_mcdm_0_r'
out_path = root + experiment + "_localization_error_motion.png"

resolution = 0.25
std_dev_list = [ 1]
run = 50
total_tag = 10
localization_errors = []
agent_list = ['nbs', 'bayes_nbs']
legend_list = ["NBS[1]", "$\it{bayes}$NBS"]
color_list = ['red', 'blue']

rows = 5
cols = 2
yticks_list =  [0, 3, 6, 9, 12, 15]
parameters = {'axes.labelsize': 8,
                'ytick.labelsize': 8, 
                'xtick.labelsize': 8,
                'legend.fontsize': 8}
plt.rcParams.update(parameters)
fig = plt.figure(figsize=(6,7.5))
axs = fig.subplots(rows, cols, sharex=True, sharey=True)
for ax in axs.flat:
    ax.set(xlabel='Robot configuration', ylabel='Localization\nerror [m]')
    ax.yaxis.set_label_position('left')
    ax.yaxis.set_ticks(yticks_list)
    ax.grid(axis='y', alpha=0.2)
# Hide x labels and tick labels for top plots and y ticks for right plots.
for ax in axs.flat:
    ax.label_outer()

def plotData(data, row, col, axes, color):
        # Normalise lenght
        max_len_index = np.max([x.shape[0] for x in tag_single])
        final = np.zeros(shape=(max_len_index, len(data) + 2))
        for i in range(len(data)):
            while tag_single[i].shape[0] < max_len_index:
                tag_single[i] = np.vstack([tag_single[i], tag_single[i][-1,:]])
            final[:,i] = tag_single[i][:,0]
            # Calculate average and std dev 
        final[:,-2] = np.average(final[:,0:-3], axis=1)
        final[:,-1] = np.std(final[:,0:-3], axis=1)
        #  Plot the trajectories
        x = np.arange(1, final.shape[0] + 1, 1)
        axs[row, col].plot(x, final[:,-2], label=str(agent), color=color, linewidth=0.8)
        axs[row, col].fill_between(x, final[:, -2]-final[:, -1], final[:, -2]+final[:, -1], alpha=0.1, edgecolor=color, facecolor=color)
        # axs[row, col].legend(loc='upper left')#, fontsize='x-small')
        axs[row, col].set_xlim(left=0, right=1000)
        axs[row, col].set_ylim(bottom=0, top=max(yticks_list))
        axs[row, col].set_title("Tag-"+str(tag_id), fontsize=8)
        if (col == 1):
            axs[row,col].yaxis.label.set_color(color='white')

for agent_index in range(len(agent_list)):
    agent = agent_list[agent_index]
    for motion_index in range(len(std_dev_list)):
        motion = std_dev_list[motion_index]
        path = root + str(agent) + "/" + str(experiment) + '/std_' + str(motion) + "/"
        tag_single = []
        for row in range(rows):
            for col in range(cols):
                for i in range(run):
                    tag_id = 2*row + col;
                    tmp = np.loadtxt(open(path +"entropy_" + str(i) + "/distance_" + str(tag_id) + '.csv', "rb"), delimiter=",", skiprows=0)
                    if (tmp.size > 2):
                        tmp = np.expand_dims(tmp, axis=1)  # Add one column needed later for stacking
                        tmp *= resolution  # Multiple distance by resolution to obtain data in meters
                        tag_single.append(tmp)
                
                plotData(tag_single, row, col, axs, color_list[agent_index])
                tag_single.clear()
    
#place legend above plot
fig.legend(labels=legend_list,       # The labels for each line
            loc="upper center",        # Position of the legend
            borderaxespad=0.1,         # Add little spacing around the legend box
            title="Agents", 
            title_fontsize=8,
            ncol=2,
            bbox_to_anchor=(0.5, 0.99),
            fontsize=8)      
# plt.subplots_adjust(left=0.07, right=0.93, wspace=0.25, hspace=0.2)

fig.suptitle("Tags localization error with {} tags".format(experiment), color="white")
plt.tight_layout(h_pad=0)
# plt.show()
fig.savefig(out_path, dpi=300)
