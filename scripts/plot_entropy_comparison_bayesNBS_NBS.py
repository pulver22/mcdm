import numpy as np
import matplotlib.pyplot as plt 

def normaliseDataAndPlot(ax, total_entropies, color, label):
    # Normalise lenght
    max_len_index = np.max([x.shape[0] for x in total_entropies])
    final = np.zeros(shape=(max_len_index, len(total_entropies) + 2))
    for i in range(len(total_entropies)):
        while total_entropies[i].shape[0] < max_len_index:
            total_entropies[i] = np.vstack([total_entropies[i], total_entropies[i][-1,:]])
        final[:,i] = total_entropies[i][:,0]

    # Normalise between 0 and 1
    max_entropy = np.max(final[..., :])
    # print(max_entropy)
    final /= max_entropy
    # exit(0)
    # Calculate average and std dev 
    final[:,-2] = np.average(final[:,0:-3], axis=1)
    final[:,-1] = np.std(final[:,0:-3], axis=1)
    #  Plot the trajectories
    x = np.arange(1, final.shape[0] + 1, 1)
    plt.xlim(xmin=0, xmax=1000)
    plt.plot(x, final[:, -2],  color=color, label=label)
    plt.fill_between(x, final[:, -2]-final[:, -1], final[:, -2]+final[:, -1], alpha=0.2, edgecolor=color, facecolor=color)
    # plt.show()
    
root = '/home/pulver/Desktop/nbs_ecmr/'
experiment = "static"
coverage_log = '/coverage_inbeng_mcdm_'

std_dev_list = [1]
agent_list = ['nbs', 'bayes_nbs']
color_list = ['red', 'blue']
run = 50
total_tag = 10


# root = path

total_entropies = []
tag_all_run = []
tag_single_run = [] 
len_arr = []
full_coverage_index = []

f = plt.figure()
ax = f.add_subplot(111)
ax.yaxis.tick_right()
ax.yaxis.set_label_position("right")

for agent_index in range(len(agent_list)):
    agent = agent_list[agent_index]
    color  = color_list[agent_index]
    for motion_index in range(len(std_dev_list)):
        motion = std_dev_list[motion_index]
        for i in range(0,run):
            tmp = np.loadtxt(open(root + str(agent) + "/" + str(experiment) + '/std_' + str(motion) + '/entropy_' + str(i) + '/entropy.csv', "rb"), delimiter=",")
            if (tmp.size > 2):
                # coverage = np.loadtxt(open(root + 'std__' + str(motion) + coverage_log + str(int(motion)) + "_r" + str(i) + '.csv', "rb"), delimiter=",", skiprows=1)
                # full_coverage_index.append(coverage[-1,-3])
                # Remove duplicate lines without sorting
                indexes = np.unique(tmp, return_index=True)[1]
                [tmp[index] for index in sorted(indexes)]
                tmp = np.expand_dims(tmp, axis=1)  # Add one column needed later for stacking
                total_entropies.append(tmp)
                len_arr.append(tmp.shape[0])
        # exp_completed = int(np.mean(full_coverage_index))
        normaliseDataAndPlot(ax, total_entropies, color=color, label='{}'.format(agent))
        total_entropies.clear()

plt.legend(title="Agent")
plt.xlabel('Robot configurations')
plt.ylabel('Entropy' )
plt.title('Map Entropy with {} tags'.format(experiment))
# plt.show()
plt.tight_layout()
plt.savefig(root + experiment + '_entropy_vs_agent.png', dpi=300)

# Now I must collect, for every tag, its entropy from all the runs
# single_tag = []
# for j in range(total_tag):
#     for i in range(run):
#         single_tag.append(tag_all_run[i][j])
#     normaliseDataAndPlot(single_tag, exp_completed,
#                         out_path=root + 'entropy_' + str(run) + 'runs_tag' + str(j) + '.png',
#                         title='Tag-' + str(j) + ' entropy')
#     single_tag = []

