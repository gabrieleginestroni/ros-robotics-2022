import pandas as pd
import numpy as np
import math
from progress.bar import Bar

# csv import
df = pd.read_csv("../csv/calibration3.csv")
print(df.head())

# given values for parameters
given_R = 0.067
given_LW = 0.345
given_N = 42
B = 10 ** 9

# set this to control the amplitude of the interval centered around the given value for R
amplitude_R = 0.005
# set this to control the granularity of the interval centered around the given value for R
interval_R = 60

# set this to control the amplitude of the interval centered around the given value for L+W
amplitude_LW = 0.05
# set this to control the granularity of the interval centered around the given value for L+W
interval_LW = 60

# set this to control the amplitude of the interval centered around the given value for N
amplitude_N = 0

total = (interval_R) * (interval_LW) * (2 * amplitude_N + 1)

# define here search order
r_start = given_R
r_end = given_R + amplitude_R
lw_start = given_LW
lw_end = given_LW + amplitude_LW
print("\nTesting " + str(total) + " values...\nR range: [" + str(r_start) + "->" + str(r_end) + "]\nLW range: [" + str(
    lw_start) + "->" + str(lw_end) + "]\n")

# generate search intervals
parameters = [np.linspace(r_start, r_end, num=interval_R).tolist(),
              np.linspace(lw_start, lw_end, num=interval_LW).tolist(),
              np.linspace(given_N - amplitude_N, given_N + amplitude_N, num=(2 * amplitude_N + 1)).tolist()]

# load target position
target_pose_x = np.array(df["x"])
target_pose_y = np.array(df["y"])

# load wheels' motor speeds
fl_vel = np.array(df["fl"])
fr_vel = np.array(df["fr"])
rl_vel = np.array(df["rl"])
rr_vel = np.array(df["rr"])

# computing wheels speeds
fl_vel = np.multiply(fl_vel, 1 / (60 * 5))
fr_vel = np.multiply(fr_vel, 1 / (60 * 5))
rl_vel = np.multiply(rl_vel, 1 / (60 * 5))
rr_vel = np.multiply(rr_vel, 1 / (60 * 5))

# generating time-steps
timestamp_sec = np.array(df["sec"])
timestamp_nsec = np.array(df["nsec"])
time_s = np.zeros(len(df))
for i in range(1, len(df), 1):
    time_s[i] = (timestamp_sec[i] - timestamp_sec[i-1]) + (timestamp_nsec[i] - timestamp_nsec[i-1]) / B

# progress bar
class LoadingBar(Bar):
    suffix = '%(percent)d%% - %(index)d/%(tot)d - %(text)s'
    text = 'Initializing'
    tot = total
    count = 0

# main loop
bestError_rss = 0
bestParameters_rss = []
count = 0
bar = LoadingBar('Processing', max=total, fill='@')
for R in parameters[0]:
    for LW in parameters[1]:
        bar.index = count
        count = count + 1
        # initialization
        x = df.iloc[0]['x']
        y = df.iloc[0]['y']
        theta = -0.0215406
        cumulativeError_rss = 0

        # compute linear and angular velocities
        v_x = np.multiply((R / 4), np.add(np.add(np.add(fl_vel, fr_vel), rl_vel), rr_vel))
        v_y = np.multiply((R / 4), np.add(np.add(np.add(fr_vel, rl_vel), -1 * fl_vel), -1 * rr_vel))
        w = np.multiply((R / (4 * LW)), np.add(np.add(np.add(fr_vel, rr_vel), -1 * fl_vel), -1 * rl_vel))

        # first position has already been set
        for i in range(1, len(df), 1):

            # Runge-Kutta # compute new pose
            x = x + (v_x[i] * math.cos(theta + w[i] * time_s[i] * 0.5) -
                     v_y[i] * math.sin(theta + w[i] * time_s[i] * 0.5)) * time_s[i]
            y = y + (v_x[i] * math.sin(theta + w[i] * time_s[i] * 0.5) +
                     v_y[i] * math.cos(theta + w[i] * time_s[i] * 0.5)) * time_s[i]

            theta = theta + w[i] * time_s[i]

            # compute RSS
            cumulativeError_rss = cumulativeError_rss + ((target_pose_x[i] - x) ** 2 + (target_pose_y[i] - y) ** 2)

            # early pruning
            if bestError_rss != 0 and cumulativeError_rss > bestError_rss:
                break

        if bestError_rss == 0 or cumulativeError_rss < bestError_rss:
            bestError_rss = cumulativeError_rss
            bestParameters_rss = [R, LW]
            bar.text = "Current minimum rss error: " + str(bestError_rss) + " with parameters: " + str(
                bestParameters_rss)

        bar.next()

bar.finish()
print("Best error: " + str(bestError_rss))
print("Best parameters: " + str(bestParameters_rss))
