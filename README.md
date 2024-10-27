# F1TENTH

Hello, this repo functions to hold the f1tenth workspace on ROS2 Foxy as well as any evaluations made on the algorithms
used.

## Setup

### This repo

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
