import pandas as pd
import numpy as np
import math
from progress.bar import Bar

# csv import
df = pd.read_csv("../csv/calibration3.csv")
print(df.head())

# given values for parameters
given_R = 0.07
given_LW = 0.369
given_N = 42
B = 10**9

# set this to control the amplitude of the interval centered around the given value for R
amplitude_R = 0.01
# set this to control the granularity of the interval centered around the given value for R
interval_R = 5

# set this to control the amplitude of the interval centered around the given value for L+W
amplitude_LW = 0.02
# set this to control the granularity of the interval centered around the given value for L+W
interval_LW = 5

# set this to control the amplitude of the interval centered around the given value for N
amplitude_N = 0

total = (2 * interval_R + 1) * (2 * interval_LW + 1) * (2 * amplitude_N + 1)
print("\nTesting " + str(total) + " values...\n")

parameters = [np.linspace(given_R - amplitude_R, given_R + amplitude_R, num=(2 * interval_R + 1)).tolist(),
              np.linspace(given_LW - amplitude_LW, given_LW + amplitude_LW, num=(2 * interval_LW + 1)).tolist(),
              np.linspace(given_N - amplitude_N, given_N + amplitude_N, num=(2 * amplitude_N + 1)).tolist()]


# progress bar
class LoadingBar(Bar):
    suffix = '%(percent)d%% - %(text)s'
    text = 'Initializing'


# main loop
bestError = 0
bestParameters = []
count = 0
bar = LoadingBar('Processing', max=total, fill='@')
for R in parameters[0]:
    for LW in parameters[1]:
        count = count + 1

        # initialization
        timestamp_sec = df.iloc[0]['sec']
        timestamp_nsec = df.iloc[0]['nsec']
        x = df.iloc[0]['x']
        y = df.iloc[0]['y']
        theta = -0.0215406
        cumulativeError = 0

        # first position has already been set
        for i in range(1, len(df), 1):
            time_s = (df.iloc[i]['sec'] - timestamp_sec) + (df.iloc[i]['nsec'] - timestamp_nsec) / B
            timestamp_sec = df.iloc[i]['sec']
            timestamp_nsec = df.iloc[i]['nsec']

            # Runge-Kutta
            v_x = (R / 4) * (df.iloc[i]['fl'] + df.iloc[i]['fr'] + df.iloc[i]['rl'] + df.iloc[i]['rr']) / (60 * 5)
            v_y = (R / 4) * (df.iloc[i]['fr'] - df.iloc[i]['fl'] + df.iloc[i]['rr'] - df.iloc[i]['rl']) / (60 * 5)
            w = (R / 4) * (df.iloc[i]['fr'] + df.iloc[i]['rl'] - df.iloc[i]['fl'] - df.iloc[i]['rr']) / (LW * 60 * 5)

            x = x + (v_x * math.cos(theta + w * time_s * 0.5) -
                     v_y * math.sin(theta + w * time_s * 0.5)) * time_s
            y = y + (v_x * math.sin(theta + w * time_s * 0.5) +
                     v_y * math.cos(theta + w * time_s * 0.5)) * time_s

            theta = theta + w * time_s

            # MSE
            cumulativeError = cumulativeError + ((df.iloc[i]['x'] - x)**2 + (df.iloc[i]['y'] - y)**2) / len(df)

            # early pruning
            if bestError != 0 and cumulativeError > bestError:
                break

        if bestError == 0 or cumulativeError < bestError:
            bestError = cumulativeError
            bestParameters = [R, LW]
            bar.text = "Current minimum error: " + str(bestError) + " with parameters: " + str(bestParameters)

        bar.next()

bar.finish()
print("Best error: " + str(bestError))
print("Best parameters: " + str(bestParameters))
