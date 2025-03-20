import pandas as pd 
import numpy as np

config = {
        "trajectory_file": "./trajectories/traj_race_cl-2025-03-19 00:12:14.569418.csv"
        }

def quaternion_to_euler_angle_vectorized1(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z 

def distance(my_pose, pose_a):
    # my_pose: the current pose of the robot
    # pose_a: an array of poses
    # return: an arraay of distances of pose_a from my_pose
    #np_a = [my_pose["x"], pose_a["y"]]
    np_a = [[0,0],[0,0]]
    np_b = [[1,1],[2,2]]
    return (np.linalg.norm(np.array([np_a, np_b])))

def update_pose():
    # pose is position and orientation
    orientation = (1,1,1,1)
    orientation_euler = quaternion_to_euler_angle_vectorized1(orientation)
    pose_euler = {"x": 0, "y": 0, "z": 0, "thetha":orientation_euler}
    return pose_euler
    
def main (args = None):
    """
    1. read the raceline trajectory
    2. get the current pose of the robot
    3. get the closest point that is in front of the car
        1. get an array of all distances and their orientation with respect to the car
        2. for those points in front of the car, get the vector of the point to the car
        2. get the pose of the point of minimum distance
        3. check orientation of the robot to the point to ensure it is in front
            1. get vector of robot to point
            2. if not in front, check 
    4. 

    """
    # x, y, velocity
    traj_csv = pd.read_csv(config["trajectory_file"], header=None, sep=",")
 #   current_pose = update_pose()
    p_a = {"x": 0, "y": 0}
    p_b = {"x": 1, "y": 1}
    dist = distance(p_a, p_b)
    print(dist)


if (__name__ == "__main__"):
    main()
