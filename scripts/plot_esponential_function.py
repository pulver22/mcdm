import matplotlib.pyplot as plt
import numpy as np
import math

def prob(x, y):
    return math.exp(-0.5 * x**2/y**2)

def createKernel(size):
    kernel = np.zeros(shape=(size,size))
    centre = (math.ceil((size-1)/2), math.ceil((size-1)/2))
    print(centre)
    for i in range(size):
        for j in range(size):
            kernel[i,j] = math.sqrt(pow(i - centre[0],2) + pow(j - centre[1], 2))
            # print("[{},{}] = {}".format(i,j, kernel[i,j]))
    # Kernel must be normalize with respect to its centre
    kernel -= kernel[int((size-1)/2), int((size-1)/2)]
    kernel = abs(kernel)
    return kernel

kernel_size = 9
distances = createKernel(kernel_size).flatten()
# print(distances)
distances = np.expand_dims(distances, axis=1)
list_index = [2, 3, 5, 10, 20, 50, 100]

prob_list = []
for index in list_index:
    p = np.asarray([prob(x,index) for x in distances])
    prob_list.append(p/sum(p))  # Normalise the probabilities
    prob_list[-1] = np.expand_dims(prob_list[-1], axis=1)


# prob_list /= sum(prob_list)

for i in range(len(prob_list)):
    distances = np.concatenate((distances, prob_list[i]), axis=1)

# Remove duplicate items
distances = np.unique(distances, axis=0)

# Plot all the results
for col in range(1, distances.shape[1]):
    plt.plot(distances[:,0], distances[:,col],'-o',  label=list_index[col-1])
plt.legend(title='y value')
plt.title('[{}x{}] p(distance) = exp(-0.5 * distance^2/y^2)'.format(kernel_size, kernel_size))
plt.xlabel('distance')
plt.ylabel('p(distance)')
# plt.show()
plt.savefig('/home/pulver/Desktop/mcdm_ral_experiments/exponential_prob_{}x{}.png'.format(kernel_size, kernel_size))
