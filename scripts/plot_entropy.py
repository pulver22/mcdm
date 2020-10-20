import numpy as np
import matplotlib.pyplot as plt 

root = '/home/pulver/Desktop/mcdm_ral_experiments/inbeng/'

run = 3
total_tag = 10

def normaliseDataAndPlot(total_entropies, out_path):
    # Normalise lenght
    max_len_index = np.max([x.shape[0] for x in total_entropies])
    final = np.zeros(shape=(max_len_index, run + 2))
    for i in range(run):
        while total_entropies[i].shape[0] < max_len_index:
            total_entropies[i] = np.vstack([total_entropies[i], total_entropies[i][-1,:]])
        final[:,i] = total_entropies[i][:,0]

    # Calculate average and std dev from the trajectorie
    final[:,-2] = np.average(final[:,0:run], axis=1)
    final[:,-1] = np.std(final[:,0:run], axis=1)
    #  Plot the trajectories
    x = np.arange(1, final.shape[0] + 1, 1)
    plt.clf()
    plt.plot(x, final[:, 3],  color='#CC4F1B', label='NBS')
    plt.fill_between(x, final[:, -2]-final[:, -1], final[:, -2]+final[:, -1], alpha=0.5, edgecolor='#CC4F1B', facecolor='#FF9848')
    plt.savefig(out_path)


total_entropies = []
tag_all_run = []
tag_single_run = [] 
len_arr = []
for i in range(0,run):
    tmp = np.loadtxt(root + 'entropy_' + str(i) + '/entropy.csv')
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
normaliseDataAndPlot(total_entropies, out_path=root + 'entropy_' + str(run) + 'runs.png')

# Now I must collect, for every tag, its entropy from all the runs
single_tag = []
for j in range(total_tag):
    for i in range(run):
        single_tag.append(tag_all_run[i][j])
    normaliseDataAndPlot(single_tag, out_path=root + 'entropy_' + str(run) + 'runs_tag' + str(j) + '.png')
    single_tag = []

