# F1TENTH

Hello, this repo functions to hold the f1tenth workspace on ROS2 Foxy as well as any evaluations made on the algorithms
used.

## Timeline/ Plan

1. Code a simple algorithm to complete rounds around a track to obtain starting data

    - Requirements
        - algorithm outputs in Twist
        - node to convert
        - able to make consecutive laps minimum 5

2. Create libreary to analyse alogorithm performance

    - Metrics:
        - fastest lap,
        - consecutive laps.
    - Rules:
        - starts from 5m away from a lap counter line.
        - said line is perpendicular to the wall of the track
        - said line is located at a stright.

3. Code more algorithms with the same requirements as before and collect data on them in a rosbag

    - List of Algorithms
        - Reactive Navigation
            - Minimum Filter
            - Search Algorithm e.g. A\* or Dijkstra
        - Global Planning
            - SLAM
                - gmapping
                - etc
            - Optimisation
                - geometric
                - time
                - etc
            - Controller
                - PID
                - MPC
        - ROS2 Nav

4. use best performing algorithm to collect data set on different maps

5. train neural network on dataset

6. train reinforment learner on environment maybe Isaac Sim

## Setup

### 1. This repo

1. Clone this repo

```
git clone https://github.com/bentjh01/BT_f1tenth.git
```

2. Setup a python virtual environment

```
# Install python venv
pip install virtualenv
# Creates new virtual environment
python -m venv .venv
# Activate virtual environment
source .venv/bin/activate
# Install requirements
pip install -r venv_requirements.txt
```

### 2. Setup F1TENTH Simulator

```
https://github.com/f1tenth/f1tenth_gym_ros.git
```

## Creating a new package

```
ros2 pkg create --build-type ament_cmake <package name>
cd <package name>
mkdir <package name>
touch ./<package name>/__init__.py
mkdir scripts
```

## Resources

-   [PlotJuggler](https://github.com/facontidavide/PlotJuggler)
-   [Python Virtual Environments](https://www.freecodecamp.org/news/how-to-setup-virtual-environments-in-python/)
-   [MarPlotLib Img](https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.imshow.html)
