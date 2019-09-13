import numpy as np
import matplotlib.pyplot as plt

# Read the 10 accuracy logs

accuracy_025       = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_025r.csv", "rb"), delimiter=",", skiprows=1)
accuracy_05        = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_05r.csv", "rb"), delimiter=",", skiprows=1)
accuracy_1         = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_r.csv", "rb"), delimiter=",", skiprows=1)
accuracy_15        = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_15r.csv", "rb"), delimiter=",", skiprows=1)
accuracy_2         = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/accuracy_2r.csv", "rb"), delimiter=",", skiprows=1)
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
# exit(0)


# coverage_1m       = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/coverage_mcdm_inb_3123_1.csv", "rb"), delimiter=",", skiprows=1)
# coverage_2m       = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/coverage_mcdm_inb_3123_2.csv", "rb"), delimiter=",", skiprows=1)
# coverage_3m       = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/coverage_mcdm_inb_3123_3.csv", "rb"), delimiter=",", skiprows=1)
# coverage_4m       = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/coverage_mcdm_inb_3123_4.csv", "rb"), delimiter=",", skiprows=1)
# coverage_5m       = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/coverage_mcdm_inb_3123_5.csv", "rb"), delimiter=",", skiprows=1)
# coverage_6m       = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/coverage_mcdm_inb_3123_6.csv", "rb"), delimiter=",", skiprows=1)
# coverage_7m       = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/coverage_mcdm_inb_3123_7.csv", "rb"), delimiter=",", skiprows=1)
# coverage_8m       = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/coverage_mcdm_inb_3123_8.csv", "rb"), delimiter=",", skiprows=1)
# coverage_9m       = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/coverage_mcdm_inb_3123_9.csv", "rb"), delimiter=",", skiprows=1)
# coverage_10m      = np.loadtxt(open("/home/pulver/Desktop/MCDM/variable_ellipse/coverage_mcdm_inb_3123_10.csv", "rb"), delimiter=",", skiprows=1)

# Calculate the RFID accuracy
# final_acc = accuracies.sum(axis=1) / 10.0
# print(final_acc)

# # Calculate the number of configurations required for full map coverage
# coverage_1m = coverage_1m[-1,4]
# coverage_2m = coverage_2m[-1,4]
# coverage_3m = coverage_3m[-1,4]
# coverage_4m = coverage_4m[-1,4]
# coverage_5m = coverage_5m[-1,4]
# coverage_6m = coverage_6m[-1,4]
# coverage_7m = coverage_7m[-1,4]
# coverage_8m = coverage_8m[-1,4]
# coverage_9m = coverage_9m[-1,4]
# coverage_10m = coverage_10m[-1,4]
# map_coverage = np.asarray([coverage_1m, coverage_2m, coverage_3m, coverage_4m, coverage_5m, coverage_6m, coverage_7m, coverage_8m, coverage_9m, coverage_10m])

print(accuracy_025[-2])
print(accuracy_025[-2,1])
print( accuracy_025[-2,2])
plt.plot(accuracy_025[-2,1], accuracy_025[-2,2], label="0.25r", marker="o", markersize=10)
plt.plot(accuracy_05[-2,1], accuracy_05[-2,2], label="0.5r", marker="o", markersize=10)
plt.plot(accuracy_1[-2,1], accuracy_1[-2,2], label="r", marker="o", markersize=10)
plt.plot(accuracy_15[-2,1], accuracy_15[-2,2], label="1.5r", marker="o", markersize=10)
plt.plot(accuracy_2[-2,1], accuracy_2[-2,2], label="2r", marker="o", markersize=10)
plt.plot(accuracy_4[-2,1], accuracy_4[-2,2], label="4r", marker="o", markersize=10)
plt.plot(accuracy_5[-2,1], accuracy_5[-2,2], label="5r", marker="o", markersize=10)
plt.plot(accuracy_6[-2,1], accuracy_6[-2,2], label="6r", marker="o", markersize=10)
plt.plot(accuracy_11[-2,1], accuracy_11[-2,2], label="11r", marker="o", markersize=10)

plt.legend(loc="lower right")
plt.ylim((0, 1.1))
plt.xlabel("Robot Configuration")
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