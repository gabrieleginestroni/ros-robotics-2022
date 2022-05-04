import pandas as pd
import numpy as np
import math
from progress.bar import Bar

# csv import
df = pd.read_csv("../csv/N_calibration.csv")
print(df.head())

# given values for parameters
given_N = 42
R = 0.07
LW = 0.369
B = 10 ** 9
PI = 3.14159265359

# set this to control the amplitude of the interval centered around the given value for N
amplitude_N = 41

total = 2 * amplitude_N + 1

print("\nTesting " + str(total) + " values...\nN range: [" + str(given_N - amplitude_N) + "->" + str(given_N + amplitude_N) + "]")

# generate search intervals
parameters = np.linspace(given_N - amplitude_N, given_N + amplitude_N, num=(2 * amplitude_N + 1)).tolist()

# load target position
target_pose_x = np.array(df["x"])
target_pose_y = np.array(df["y"])

# generating time-steps
timestamp_sec = np.array(df["sec"])
timestamp_nsec = np.array(df["nsec"])
time_s = np.zeros(len(df))
for i in range(1, len(df), 1):
    time_s[i] = (timestamp_sec[i] - timestamp_sec[i-1]) + (timestamp_nsec[i] - timestamp_nsec[i-1]) / B

# load wheels' motor ticks
fl_ticks = np.array(df["fl"])
fr_ticks = np.array(df["fr"])
rl_ticks = np.array(df["rl"])
rr_ticks = np.array(df["rr"])

# computing wheels speeds W/O CONSIDERING N
par_fl_vel = np.zeros(len(df))
par_fr_vel = np.zeros(len(df))
par_rl_vel = np.zeros(len(df))
par_rr_vel = np.zeros(len(df))
for i in range(1, len(df), 1):
    par_fl_vel[i] = ((fl_ticks[i] - fl_ticks[i - 1]) * 2 * 3.14159265359) / (time_s[i] * 5)
    par_fr_vel[i] = ((fr_ticks[i] - fr_ticks[i - 1]) * 2 * 3.14159265359) / (time_s[i] * 5)
    par_rl_vel[i] = ((rl_ticks[i] - rl_ticks[i - 1]) * 2 * 3.14159265359) / (time_s[i] * 5)
    par_rr_vel[i] = ((rr_ticks[i] - rr_ticks[i - 1]) * 2 * 3.14159265359) / (time_s[i] * 5)


# progress bar
class LoadingBar(Bar):
    suffix = '%(percent)d%% - %(index)d/%(tot)d - %(text)s'
    text = 'Initializing'
    tot = total
    count = 0


# main loop
bestError_rss = 0
bestParameter_rss = 0
count = 0
bar = LoadingBar('Processing', max=total, fill='@')
for N in parameters:
    count = count + 1

    # initialization
    x = df.iloc[0]['x']
    y = df.iloc[0]['y']
    theta = 0.0677128
    cumulativeError_rss = 0

    # computing wheels speeds W/O CONSIDERING N
    fl_vel = np.multiply((1 / N), par_fl_vel)
    fr_vel = np.multiply((1 / N), par_fr_vel)
    rl_vel = np.multiply((1 / N), par_rl_vel)
    rr_vel = np.multiply((1 / N), par_rr_vel)

    # compute linear and angular velocities
    v_x = np.multiply((R / 4), np.add(np.add(np.add(fl_vel, fr_vel), rl_vel), rr_vel))
    v_y = np.multiply((R / 4), np.add(np.add(np.add(fr_vel, rl_vel), -1 * fl_vel), -1 * rr_vel))
    w = np.multiply((R / (4 * LW)), np.add(np.add(np.add(fr_vel, rr_vel), -1 * fl_vel), -1 * rl_vel))

    # first position has already been set
    for i in range(1, len(df), 1):

        # compute new pose with Runge-Kutta
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
        bestParameter_rss = N
        bar.text = "Best error: " + str(bestError_rss) + " -> parameter: " + str(bestParameter_rss)

    bar.next()

bar.finish()
print("Best error: " + str(bestError_rss))
print("Best parameters: " + str(bestParameter_rss))
