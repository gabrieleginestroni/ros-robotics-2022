import pandas as pd
import numpy as np
import math
from datetime import datetime

# csv import
df = pd.read_csv("../csv/calibration1.csv")
print(df.head())

# initialization
timestamp = int(df.iloc[0]['timestamp'])
x = df.iloc[0]['x']
y = df.iloc[0]['y']
theta = 0

# given values for parameters
given_R = 0.07
given_LW = 0.369
given_N = 42

# set this to control the amplitude of the interval centered around the given value for R
amplitude_R = 0.01
# set this to control the granularity of the interval centered around the given value for R
interval_R = 2

# set this to control the amplitude of the interval centered around the given value for L+W
amplitude_LW = 0.1
# set this to control the granularity of the interval centered around the given value for L+W
interval_LW = 2

# set this to control the amplitude of the interval centered around the given value for N
amplitude_N = 2

print("\nTesting " + str((2 * interval_R + 1) * (2 * interval_LW + 1) * (2 * amplitude_N + 1)) + " values...\n")

parameters = [np.linspace(given_R - amplitude_R, given_R + amplitude_R, num=(2 * interval_R + 1)).tolist(),
              np.linspace(given_LW - amplitude_LW, given_LW + amplitude_LW, num=(2 * interval_LW + 1)).tolist(),
              np.linspace(given_N - amplitude_N, given_N + amplitude_N, num=(2 * amplitude_N + 1)).tolist()]
'''
# main loop
bestError = 0
bestParameters = []
for R in parameters[0]:
    for LW in parameters[1]:
        # TODO: to estimate N we need to compute velocities from ticks and not from rpm
        for N in parameters[2]:
            cumulativeError = 0
            for i in range(len(df)):
                # first position has already been set
                if i != 0:
                    time_s = datetime.timedelta(df.iloc[i]['timestamp'] - timestamp).seconds
                    timestamp = df.iloc[i]['timestamp']
                    # Runge-Kutta
                    # TODO: we need the elapsed time from a measurement and another to compute the integral
                    x = x + ((R / 4) * (df.iloc[i]['fl'] + df.iloc[i]['fr'] + df.iloc[i]['rl'] + df.iloc[i]['rr']) *
                        math.cos(theta + ((R / 4) *
                        (df.iloc[i]['fr'] + df.iloc[i]['rr'] - df.iloc[i]['fl'] - df.iloc[i]['rl']) / LW) *
                        time_s * 0.5) -
                        (R / 4) * (df.iloc[i]['fr'] - df.iloc[i]['fl'] + df.iloc[i]['rl'] - df.iloc[i]['rr']) *
                        math.sin(theta + ((R / 4) *
                        (df.iloc[i]['fr'] + df.iloc[i]['rr'] - df.iloc[i]['fl'] - df.iloc[i]['rl']) / LW) *
                        time_s * 0.5)) * time_s
                    y = y + ((R / 4) * (df.iloc[i]['fl'] + df.iloc[i]['fr'] + df.iloc[i]['rl'] + df.iloc[i]['rr']) *
                        math.sin(theta + ((R / 4) *
                        (df.iloc[i]['fr'] + df.iloc[i]['rr'] - df.iloc[i]['fl'] - df.iloc[i]['rl']) / LW) *
                        time_s * 0.5) +
                        (R / 4) * (df.iloc[i]['fr'] - df.iloc[i]['fl'] + df.iloc[i]['rl'] - df.iloc[i]['rr']) *
                        math.cos(theta + ((R / 4) *
                        (df.iloc[i]['fr'] + df.iloc[i]['rr'] - df.iloc[i]['fl'] - df.iloc[i]['rl']) / LW) *
                        time_s * 0.5)) * time_s

                    cumulativeError = cumulativeError + math.sqrt((math.pow(df.iloc[i]['x'] - x, 2) +
                                                                  (math.pow(df.iloc[i]['y'] - y, 2))))

            if bestError == 0:
                bestError = cumulativeError
                bestParameters.append(R)
                bestParameters.append(LW)
                bestParameters.append(N)
            else:
                if cumulativeError < bestError:
                    bestError = cumulativeError
                    bestParameters.clear()
                    bestParameters.append(R)
                    bestParameters.append(LW)
                    bestParameters.append(N)

print(bestParameters)
'''
timestamp = timestamp // 1000
nsec = timestamp % 1000000
sec = timestamp // 1000000
print(nsec, sec)

for i in range(len(df)):
    timestamp = int(df.iloc[i]['timestamp'])
    timestamp = timestamp // 1000
    nsec = timestamp % 1000000
    sec = timestamp // 1000000
    print(nsec, sec)
