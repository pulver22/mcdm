import numpy as np
import matplotlib.pyplot as plt 

root = '/home/pulver/Desktop/mcdm_ral_experiments/inbeng/'
coverage_log = 'coverage_inbeng_mcdm_0_r'

run = 3
total_tag = 10

def normaliseDataAndPlot(total_entropies, exp_completed, out_path, title):
    # Normalise lenght
    max_len_index = np.max([x.shape[0] for x in total_entropies])
    final = np.zeros(shape=(max_len_index, run + 2))
    for i in range(run):
        while total_entropies[i].shape[0] < max_len_index:
            total_entropies[i] = np.vstack([total_entropies[i], total_entropies[i][-1,:]])
        final[:,i] = total_entropies[i][:,0]

    # Normalise between 0 and 1
    max_entropy = np.max(final[..., :])
    # print(max_entropy)
    final /= max_entropy
    # exit(0)
    # Calculate average and std dev 
    final[:,-2] = np.average(final[:,0:run], axis=1)
    final[:,-1] = np.std(final[:,0:run], axis=1)
    #  Plot the trajectories
    x = np.arange(1, final.shape[0] + 1, 1)

    f = plt.figure()
    ax = f.add_subplot(111)
    ax.yaxis.tick_right()
    ax.yaxis.set_label_position("right")

    plt.axvline(x=exp_completed, label='Exploration completed', linestyle="--", color='black', alpha=0.2 )
    plt.axhline(y=final[exp_completed,3], label='Entropy at exploration completed', linestyle=":", color='black', alpha=0.2 )
    plt.plot(x, final[:, 3],  color='#CC4F1B', label='Entropy at runtime')
    plt.fill_between(x, final[:, -2]-final[:, -1], final[:, -2]+final[:, -1], alpha=0.5, edgecolor='#CC4F1B', facecolor='#FF9848')
    
    plt.title(title)
    plt.legend()
    plt.xlim(xmin=0, xmax=x[-1])
    # move ticks
    # plt.tick_params(axis='y', which='both',left=False, right=True, labelleft=False, labelright=True)
    plt.xlabel('Robot configuration')
    plt.ylabel('Entropy' )
    
    # move label
    # plt.ylabel('Your label here', labelpad=-725, fontsize=18)
    plt.tight_layout()
    plt.savefig(out_path, dpi=300)


total_entropies = []
tag_all_run = []
tag_single_run = [] 
len_arr = []
full_coverage_index = []
for i in range(0,run):
    tmp = np.loadtxt(open(root + 'entropy_' + str(i) + '/entropy.csv', "rb"), delimiter=",")
    coverage = np.loadtxt(open(root + coverage_log + str(i) + '.csv', "rb"), delimiter=",", skiprows=1)
    full_coverage_index.append(coverage[-1,-3])
    # Remove duplicate lines without sorting
    indexes = np.unique(tmp, return_index=True)[1]
    [tmp[index] for index in sorted(indexes)]
    tmp = np.expand_dims(tmp, axis=1)  # Add one column needed later for stacking
    total_entropies.append(tmp)
    len_arr.append(tmp.shape[0])
    # tag_entropies = []
    for j in range(total_tag):
        tmp = np.loadtxt(root + 'entropy_' + str(i) + '/entropy_' + str(j) + '.csv')
        indexes = np.unique(tmp, return_index=True)[1]
        [tmp[index] for index in sorted(indexes)]
        tmp = np.expand_dims(tmp, axis=1)  # Add one column needed later for stacking
        tag_single_run.append(tmp)
    tag_all_run.append(tag_single_run)
    tag_single_run = []
    #     tag_entropies.append(tmp)
exp_completed = int(np.mean(full_coverage_index))
normaliseDataAndPlot(total_entropies, exp_completed, 
                    out_path=root + 'entropy_' + str(run) + 'runs.png',
                    title='Map total entropy')

# Now I must collect, for every tag, its entropy from all the runs
single_tag = []
for j in range(total_tag):
    for i in range(run):
        single_tag.append(tag_all_run[i][j])
    normaliseDataAndPlot(single_tag, exp_completed,
                        out_path=root + 'entropy_' + str(run) + 'runs_tag' + str(j) + '.png',
                        title='Tag-' + str(j) + ' entropy')
    single_tag = []

