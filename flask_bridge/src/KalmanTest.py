from numpy import *
import math
import matplotlib.pyplot as plt
from KalmanFilter import KalmanFilter

t = linspace(0, math.pi/5, 600)

k = KalmanFilter(0.5, 0.5, 0.5)

test_array = []
kalman_array = []
for i in range(0, 600):
    current_t = t[i]
    meas = sin(4*math.pi*current_t)*random.uniform(1, 1.5)
    test_array.append(meas)
    new_est = k.updateEstimate(meas)
    kalman_array.append(new_est)

plt.plot(t, test_array, 'b')
plt.plot(t, kalman_array, 'g')
plt.show()
