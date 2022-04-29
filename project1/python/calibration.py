import pandas as pd
import numpy as np
import math
from progress.bar import Bar

# csv import
df = pd.read_csv("../csv/calibration2.csv")
print(df.head())

# given values for parameters
given_R = 0.067
given_LW = 0.345
given_N = 42
B = 10**9

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

total = (interval_R ) * (interval_LW ) * (2 * amplitude_N + 1)

# define here search order
r_start = given_R
r_end = given_R + amplitude_R
lw_start = given_LW
lw_end = given_LW + amplitude_LW
print("\nTesting " + str(total) + " values...\nR range: ["+str(r_start)+"->"+str(r_end)+"]\nLW range: ["+str(lw_start)+"->"+str(lw_end)+"]\n")

parameters = [np.linspace(r_start, r_end, num=interval_R).tolist(),
              np.linspace(lw_start, lw_end, num=interval_LW).tolist(),
              np.linspace(given_N - amplitude_N, given_N + amplitude_N, num=(2 * amplitude_N + 1)).tolist()]

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
        count = count + 1
        bar.index = count

        # initialization
        timestamp_sec = df.iloc[0]['sec']
        timestamp_nsec = df.iloc[0]['nsec']
        x = df.iloc[0]['x']
        y = df.iloc[0]['y']
        theta = 0.0677128
        cumulativeError_rss = 0

        # first position has already been set
        for i in range(1, len(df), 1):
            time_s = (df.iloc[i]['sec'] - timestamp_sec) + (df.iloc[i]['nsec'] - timestamp_nsec) / B
            timestamp_sec = df.iloc[i]['sec']
            timestamp_nsec = df.iloc[i]['nsec']

            # Runge-Kutta
            v_x = (R / 4) * (df.iloc[i]['fl'] + df.iloc[i]['fr'] + df.iloc[i]['rl'] + df.iloc[i]['rr']) / (60 * 5)
            v_y = (R / 4) * (df.iloc[i]['fr'] - df.iloc[i]['fl'] - df.iloc[i]['rr'] + df.iloc[i]['rl']) / (60 * 5)
            w = (R / 4) * (df.iloc[i]['fr'] + df.iloc[i]['rr'] - df.iloc[i]['fl'] - df.iloc[i]['rl']) / (LW * 60 * 5)

            x = x + (v_x * math.cos(theta + w * time_s * 0.5) -
                     v_y * math.sin(theta + w * time_s * 0.5)) * time_s
            y = y + (v_x * math.sin(theta + w * time_s * 0.5) +
                     v_y * math.cos(theta + w * time_s * 0.5)) * time_s

            theta = theta + w * time_s

            # RSS
            cumulativeError_rss = cumulativeError_rss + ((df.iloc[i]['x'] - x)**2 + (df.iloc[i]['y'] - y)**2)

            # early pruning
            if bestError_rss != 0 and cumulativeError_rss > bestError_rss:
                break

        if bestError_rss == 0 or cumulativeError_rss < bestError_rss:
            bestError_rss = cumulativeError_rss
            bestParameters_rss = [R, LW]
            bar.text = "Current minimum rss error: " + str(bestError_rss) + " with parameters: " + str(bestParameters_rss)

        bar.next()

bar.finish()
print("Best error: " + str(bestError_rss))
print("Best parameters: " + str(bestParameters_rss))
