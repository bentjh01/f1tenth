import rosbag_parser as rp
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

def plot_trajectory(rosbag_path, map_path, odom_topic, title, r):
    odom = rp.read_rosbag_odom(rosbag_path, odom_topic) 
    t = []
    x = []
    y = []
    initial_time = np.nan
    for i, msg in enumerate(odom):
        if np.isnan(msg.pose.pose.position.x ) or np.isnan(msg.pose.pose.position.y):
            continue
        if np.isnan(initial_time):
            initial_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)-initial_time)
        x.append(msg.pose.pose.position.x)
        y.append(msg.pose.pose.position.y)

    im = plt.imread(map_path+".png")
    with open(map_path+".yaml") as file:
        for line in file:
            if "resolution" in line:
                list_line = line.split()
                resolution = float(list_line[1])
                print(resolution)
            if "origin" in line:
                origin = []
                list_line = line.split(":")
                list_line = list_line[-1].split("]")
                list_line = list_line[0].split("[")
                list_line = list_line[-1].split(",")
                for l in list_line:
                    l = l.strip()
                    origin.append(float(l))
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(x, y)
    ax.set_aspect('equal')
    ax.set_title(title)
    ax.imshow(im, cmap='Greys_r', extent=[origin[0], origin[0]+im.shape[0]*resolution, origin[1], origin[1]+im.shape[1]*resolution])
    start_point = [x[0], y[0]]
    ax.annotate("Start", (start_point[0], start_point[1]), textcoords="offset points", xytext=(0,10), ha='center')
    ax.plot(start_point[0], start_point[1], 'ro')
    end_point = [x[-1], y[-1]]
    ax.plot(end_point[0], end_point[1], 'ro')
    ax.annotate("End", (end_point[0], end_point[1]), textcoords="offset points", xytext=(0,10), ha='center')
    cr = mpatches.Circle(start_point, r, fill=False)
    ax.add_patch(cr)
    plt.show()

    near = True
    lap_time = []
    lap_start = np.inf
    for i, t_s in enumerate(t):
        x_s = x[i]
        y_s = y[i]
        distance = np.sqrt((x_s-start_point[0])**2 + (y_s-start_point[1])**2)
        if distance >= r and near:
            if lap_start != np.inf:
                lap_time.append(t_s-lap_start)
            lap_start = t_s
            near = False
        if not near:
            if distance < r:
                near = True
    print(lap_time)