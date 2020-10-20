import matplotlib.pyplot as plt
import numpy as np
import math

def prob(x, y):
    return math.exp(-0.5 * x/y)

distances = np.asarray([1.4, 1, 1.4, 1, 0, 1, 1.4, 1, 1.4])
distances = np.expand_dims(distances, axis=1)
list_index = [2, 3, 5, 10, 20, 50, 100]

prob_list = []
for index in list_index:
    prob_list.append(np.asarray([prob(x,index) for x in distances]))
    prob_list[-1] = np.expand_dims(prob_list[-1], axis=1)
# a2 = np.asarray([prob(x,2) for x in distances])
# a3 = np.asarray([prob(x,3) for x in distances])
# a5 = np.asarray([prob(x,5) for x in distances])
# a10 = np.asarray([prob(x,10) for x in distances])
# a20 = np.asarray([prob(x,20) for x in distances])
# a50 = np.asarray([prob(x,50) for x in distances])
# a100 = np.asarray([prob(x,100) for x in distances])

for i in range(len(prob_list)):
    distances = np.concatenate((distances, prob_list[i]), axis=1)

# Remove duplicate items
distances = np.unique(distances, axis=0)

# Plot all the results
for col in range(1, distances.shape[1]):
    plt.plot(distances[:,0], distances[:,col],'-o',  label=list_index[col-1])

plt.legend(title='y value')
plt.title('p(distance) = exp(-0.5 * distance/y)')
plt.xlabel('distance')
plt.ylabel('p(distance)')
# plt.show()
plt.savefig('./results/exponential_prob.png')
