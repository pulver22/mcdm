import numpy as np
import matplotlib.pyplot as plt 

total_entropy = np.loadtxt('/tmp/entropy.csv')
total_tag = 10

tag_entropies = []
for i in range(total_tag):
    tmp = np.loadtxt('/tmp/entropy_' + str(i) + '.csv')
    tag_entropies.append(tmp)



for i in range(int(total_tag)):
    plt.plot(tag_entropies[i], label='tag_'+str(i))

plt.legend(ncol=int(total_tag/3), loc='best')
# plt.show()
plt.savefig('./results/entropy_tags.png')

plt.clf()
plt.plot(total_entropy, label='total')
plt.legend(loc='best')
plt.savefig('./results/entropy_total.png')