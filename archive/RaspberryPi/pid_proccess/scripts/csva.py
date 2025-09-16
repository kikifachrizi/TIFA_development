from pandas import *

data = read_csv("trajectory_xy_new.csv")

setpoints = []

x = data['x'].tolist()
y = data['y'].tolist()

for idx in range(len(x)):
    cord = {"x": x[idx], "y": y[idx], "th": 0.0}
    setpoints.append(cord)


print(setpoints)
