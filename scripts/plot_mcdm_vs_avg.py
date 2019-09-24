import numpy as np
import matplotlib.pyplot as plt

env = "inbatrium"
mcdm_data = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/mcdm/logic_exps/mcdm_vs_avg/" + env + "/mcdm/mcdm_result_" + env + "_1.csv", "rb"), delimiter=",", skiprows=1)
avg_data  = np.loadtxt(open("/home/pulver/Dropbox/University/PostDoc/MCDM/mcdm/logic_exps/mcdm_vs_avg/" + env + "/avg/mcdm_result_" + env + "_1.csv", "rb"), delimiter=",", skiprows=1)


fig, ax = plt.subplots()
bar_width = 0.2
opacity = 0.8

y_pos = np.arange(mcdm_data.shape[0])
configurations = ('[1,0,0]', '[0,1,0]', '[0,0,1]', '[0.33,0.33,0.33]', '[0.6,0.2,0.2]',
                  '[0.42,0.42,0.14]', '[0.2,0.6,0.2]', '[0.14,0.42,0.42]', '[0.2,0.2,0.6]',
                  '[0.42,0.14,0.42]', '[0.5,0.5,0]', '[0,0.5,0.5]', '[0.5,0,0.5]')

# Robot configurations
rects1 = plt.bar(y_pos - bar_width, mcdm_data[:, 5], bar_width, alpha=opacity, color='b', label='config-MCDM')
rects2 = plt.bar(y_pos , avg_data[:, 5], bar_width, alpha=opacity, color='g', label='config-wAVG')
plt.xticks(y_pos + bar_width, configurations)
# plt.ylabel('Robot Configurations')
# plt.title('MCDM vs w-AVG')
# plt.legend()


# plt.tight_layout()
# fig.show()


# Travelled distance
rects1 = plt.bar(y_pos + bar_width, mcdm_data[:, 6], bar_width, alpha=opacity, color='r', label='dist-MCDM')
rects2 = plt.bar(y_pos + 2* bar_width, avg_data[:, 6], bar_width, alpha=opacity, color='y', label='dist-wAVG')
plt.xticks(y_pos + bar_width, configurations)
# plt.ylabel('TravelledDistance')
plt.title('MCDM vs w-AVG')
plt.legend()


plt.tight_layout()
plt.show()