import numpy as np
import matplotlib.pyplot as plt 

def normaliseDataAndPlot(ax, total_entropies, color, label):
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
    plt.xlim(xmin=0, xmax=x[-1])
    plt.plot(x, final[:, 3],  color=color, label=label)
    plt.fill_between(x, final[:, -2]-final[:, -1], final[:, -2]+final[:, -1], alpha=0.2, edgecolor=color, facecolor=color)
    # plt.show()
    
path = '/home/pulver/Desktop/mcdm_ral_experiments/static/'
coverage_log = '/coverage_inbeng_mcdm_0_r'

kernel_list = [3, 5, 9]
motion_list = [2, 3, 5]
color_list = ['red', 'blue', 'green']
run = 3
total_tag = 10

for motion_index in range(len(motion_list)):
    motion = motion_list[motion_index]
    print("----")
    print(motion)
    f = plt.figure()
    ax = f.add_subplot(111)
    ax.yaxis.tick_right()
    ax.yaxis.set_label_position("right")
    for kernel_index in range(len(kernel_list)):
        kernel = kernel_list[kernel_index]
        root = ''
        root = path + 'kernel_' + str(kernel) + 'x' + str(kernel) + '/' + str(motion) 
        print(root)
        total_entropies = []
        tag_all_run = []
        tag_single_run = [] 
        len_arr = []
        full_coverage_index = []

        

        color  = color_list[motion_index]
        for i in range(0,run):
            tmp = np.loadtxt(open(root + '/entropy_' + str(i) + '/entropy.csv', "rb"), delimiter=",")
            coverage = np.loadtxt(open(root + coverage_log + str(i) + '.csv', "rb"), delimiter=",", skiprows=1)
            full_coverage_index.append(coverage[-1,-3])
            # Remove duplicate lines without sorting
            indexes = np.unique(tmp, return_index=True)[1]
            [tmp[index] for index in sorted(indexes)]
            tmp = np.expand_dims(tmp, axis=1)  # Add one column needed later for stacking
            total_entropies.append(tmp)
            len_arr.append(tmp.shape[0])
            # tag_entropies = []
            # for j in range(total_tag):
            #     tmp = np.loadtxt(root + 'entropy_' + str(i) + '/entropy_' + str(j) + '.csv')
            #     indexes = np.unique(tmp, return_index=True)[1]
            #     [tmp[index] for index in sorted(indexes)]
            #     tmp = np.expand_dims(tmp, axis=1)  # Add one column needed later for stacking
            #     tag_single_run.append(tmp)
            # tag_all_run.append(tag_single_run)
            # tag_single_run = []
        #     tag_entropies.append(tmp)
        exp_completed = int(np.mean(full_coverage_index))
        normaliseDataAndPlot(ax, total_entropies, color=color_list[kernel_index], label=str(kernel))
    plt.legend(title="Motion Kernel size")
    plt.xlabel('Robot configurations')
    plt.ylabel('Entropy' )
    plt.title('Fixed Motion Gaussian std-dev (' + str(motion) + ') vs variable kernel size')
    # plt.show()
    plt.tight_layout()
    plt.savefig(path + 'stddev' + str(motion) +'_vs_kernel.png', dpi=300)
    
    # Now I must collect, for every tag, its entropy from all the runs
    # single_tag = []
    # for j in range(total_tag):
    #     for i in range(run):
    #         single_tag.append(tag_all_run[i][j])
    #     normaliseDataAndPlot(single_tag, exp_completed,
    #                         out_path=root + 'entropy_' + str(run) + 'runs_tag' + str(j) + '.png',
    #                         title='Tag-' + str(j) + ' entropy')
    #     single_tag = []

