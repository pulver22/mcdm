import numpy as np
import matplotlib.pyplot as plt

# Read the 10 accuracy logs

accuracy_025       = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_025r.csv", "rb"), delimiter=",", skiprows=1)
accuracy_05        = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_05r.csv", "rb"), delimiter=",", skiprows=1)
accuracy_1         = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_r.csv", "rb"), delimiter=",", skiprows=1)
accuracy_15        = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_15r.csv", "rb"), delimiter=",", skiprows=1)
accuracy_2         = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_2r.csv", "rb"), delimiter=",", skiprows=1)
accuracy_25         = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_25r.csv", "rb"), delimiter=",", skiprows=1)
accuracy_3         = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_3r.csv", "rb"), delimiter=",", skiprows=1)
accuracy_4         = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_4r.csv", "rb"), delimiter=",", skiprows=1)
accuracy_5         = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_5r.csv", "rb"), delimiter=",", skiprows=1)
accuracy_6         = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_6r.csv", "rb"), delimiter=",", skiprows=1)
accuracy_11        = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_11r.csv", "rb"), delimiter=",", skiprows=1)


avg = np.average(accuracy_025, axis=0)
std = np.std(accuracy_025, axis=0)
accuracy_025 = np.vstack((accuracy_025, avg))
accuracy_025 = np.vstack((accuracy_025, std))

avg = np.average(accuracy_05, axis=0)
std = np.std(accuracy_05, axis=0)
accuracy_05 = np.vstack((accuracy_05, avg))
accuracy_05 = np.vstack((accuracy_05, std))

avg = np.average(accuracy_1, axis=0)
std = np.std(accuracy_1, axis=0)
accuracy_1 = np.vstack((accuracy_1, avg))
accuracy_1 = np.vstack((accuracy_1, std))

avg = np.average(accuracy_15, axis=0)
std = np.std(accuracy_15, axis=0)
accuracy_15 = np.vstack((accuracy_15, avg))
accuracy_15 = np.vstack((accuracy_15, std))

avg = np.average(accuracy_2, axis=0)
std = np.std(accuracy_2, axis=0)
accuracy_2 = np.vstack((accuracy_2, avg))
accuracy_2 = np.vstack((accuracy_2, std))

avg = np.average(accuracy_25, axis=0)
std = np.std(accuracy_25, axis=0)
accuracy_25 = np.vstack((accuracy_25, avg))
accuracy_25 = np.vstack((accuracy_25, std))

avg = np.average(accuracy_3, axis=0)
std = np.std(accuracy_3, axis=0)
accuracy_3 = np.vstack((accuracy_3, avg))
accuracy_3 = np.vstack((accuracy_3, std))

avg = np.average(accuracy_4, axis=0)
std = np.std(accuracy_4, axis=0)
accuracy_4 = np.vstack((accuracy_4, avg))
accuracy_4 = np.vstack((accuracy_4, std))

avg = np.average(accuracy_5, axis=0)
std = np.std(accuracy_5, axis=0)
accuracy_5 = np.vstack((accuracy_5, avg))
accuracy_5 = np.vstack((accuracy_5, std))

avg = np.average(accuracy_6, axis=0)
std = np.std(accuracy_6, axis=0)
accuracy_6 = np.vstack((accuracy_6, avg))
accuracy_6 = np.vstack((accuracy_6, std))

avg = np.average(accuracy_11, axis=0)
std = np.std(accuracy_11, axis=0)
accuracy_11 = np.vstack((accuracy_11, avg))
accuracy_11 = np.vstack((accuracy_11, std))


# print(accuracy_025[-2])
# print(accuracy_025[-2,1])
# print( accuracy_025[-2,2])
configs    = np.asarray([accuracy_025[-2,1], accuracy_05[-2,1], accuracy_1[-2,1], accuracy_15[-2,1], accuracy_2[-2,1], accuracy_25[-2,1], accuracy_3[-2,1], accuracy_4[-2,1], accuracy_5[-2,1], accuracy_6[-2,1], accuracy_11[-2,1]])
accuracies = np.asarray([accuracy_025[-2,2], accuracy_05[-2,2], accuracy_1[-2,2], accuracy_15[-2,2], accuracy_2[-2,2], accuracy_25[-2,2], accuracy_3[-2,2], accuracy_4[-2,2], accuracy_5[-2,2], accuracy_6[-2,2], accuracy_11[-2,2]])
plt.plot(configs, accuracies, '--', linewidth=1, color="orange")

plt.plot(accuracy_025[-2,1], accuracy_025[-2,2], label="0.25r", marker="o", markersize=10)
plt.plot(accuracy_05[-2,1], accuracy_05[-2,2], label="0.5r", marker="v", markersize=10)
plt.plot(accuracy_1[-2,1], accuracy_1[-2,2], label="r", marker="^", markersize=10)
plt.plot(accuracy_15[-2,1], accuracy_15[-2,2], label="1.5r", marker="<", markersize=10)
plt.plot(accuracy_2[-2,1], accuracy_2[-2,2], label="2r", marker=">", markersize=10)
plt.plot(accuracy_25[-2,1], accuracy_25[-2,2], label="2.5r", marker="D", markersize=10)
plt.plot(accuracy_3[-2,1], accuracy_3[-2,2], label="3r", marker="d", markersize=10)
plt.plot(accuracy_4[-2,1], accuracy_4[-2,2], label="4r", marker="s", markersize=10)
plt.plot(accuracy_5[-2,1], accuracy_5[-2,2], label="5r", marker="p", markersize=10)
plt.plot(accuracy_6[-2,1], accuracy_6[-2,2], label="6r", marker="P", markersize=10)
plt.plot(accuracy_11[-2,1], accuracy_11[-2,2], label="11r", marker="X", markersize=10)







plt.legend(loc="lower right", ncol=3)
plt.ylim((0, 1.1))
plt.xlabel("Robot Configurations")
plt.ylabel("RFID Detection Accuracy")
# x = np.arange(1, final_acc.shape[0] + 1, 1)
# ax = plt.gca()
# ax2 = ax.twinx()

# ax.plot(x, final_acc, color='red', label='RFID detection')
# ax.set_xlabel("Ellipse size [m]")
# ax.set_ylabel("RFID detection accuracy")
# ax.set_ylim((0, 1.1))
# # ax.legend()

# ax2.plot(x, map_coverage, color='blue', label='Robot Configuration')
# ax2.set_ylabel("Robot Configuration")
# ax2.legend()

plt.show()