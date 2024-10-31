import rosbag_parser as rp
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

def plot_trajectory(rosbag_path, map_path, odom_topic, title, r):
    # Read the odometry data from the rosbag
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

    # Read the map
    im = plt.imread(map_path+".png")
    # Read the resolution and origin of the map
    with open(map_path+".yaml") as file:
        for line in file:
            if "resolution" in line:
                list_line = line.split()
                resolution = float(list_line[1])
            if "origin" in line:
                origin = []
                list_line = line.split(":")
                list_line = list_line[-1].split("]")
                list_line = list_line[0].split("[")
                list_line = list_line[-1].split(",")
                for l in list_line:
                    l = l.strip()
                    origin.append(float(l))

    # Identify the laps
    start_point = [x[0], y[0]]
    end_point = [x[-1], y[-1]]
    start_lap = False
    laps = []
    lap = {"t":[], "x":[], "y":[]}
    near = True
    lap_times = []
    lap_start_time = 0.0
    for i, t_s in enumerate(t):
        x_s = x[i]
        y_s = y[i]
        distance = np.sqrt((x_s-start_point[0])**2 + (y_s-start_point[1])**2)
        if (not near and distance < r):
            near = True
        if (near and distance >= r):
            if (start_lap):
                laps.append(lap)
                lap_time = t_s - lap_start_time
                lap_times.append(lap_time)
            lap = {"t":[], "x":[], "y":[]}
            near = False
            start_lap = True
            lap_start_time = t_s

        if (start_lap):
            lap["t"].append(t_s)
            lap["x"].append(x_s)
            lap["y"].append(y_s)

    # visualise the trajectory
    fig, ax = plt.subplots(figsize=(16, 16))
    ax.set_title(f"{title}, Best lap time: {min(lap_times):.4f} [s]")
    ax.set_aspect('equal')
    ax.annotate("Start", (start_point[0], start_point[1]), textcoords="offset points", xytext=(0,10), ha='center')
    ax.plot(start_point[0], start_point[1], 'ro')
    ax.annotate("End", (end_point[0], end_point[1]), textcoords="offset points", xytext=(0,10), ha='center')
    ax.plot(end_point[0], end_point[1], 'ro')

    ax.plot(x, y)
    for i, lap in enumerate(laps):
        ax.plot(lap["x"], lap["y"],label=f"Lap {i+1}, time: {lap_times[i]:.4f} s")
    ax.imshow(im, cmap='Greys_r', extent=[origin[0], origin[0]+im.shape[0]*resolution, origin[1], origin[1]+im.shape[1]*resolution])
    cr = mpatches.Circle(start_point, r, fill=False)
    ax.add_patch(cr)
    ax.legend()
    plt.savefig(f"{title}.png")
    plt.show()

def main():
    # Parameters
    rosbag_path = "performance/rosbag2_2024_10_22-13_25_55"
    odom_topic = "/ego_racecar/odom"
    map_path = "f1tenth_ws/src/maps/maps/Spielberg_map"
    title = "Disparity Extender @ Spielberg"
    r = 7
    plot_trajectory(rosbag_path, map_path, odom_topic, title, r)

if (__name__ == "__main__"):
    main()