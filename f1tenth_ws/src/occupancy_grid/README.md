# Occupancy Grid

---

## Objectives/ Requirement

-   return a nav_msgs/Occupancy_Grid
    -   occupancy is represented as a probability from 0 - 100
    -   Row major order and starting with (0,0)
        -   all elements in a row 1 first then row 2 e.g. A11, A12, A13, A21, ...

## Reading List

### To do

-   [Occupancy Grid Mapping: An Empirical Evaluation](https://doi.org/10.1109/MED.2007.4433772)
-   [Learning Occupancy Grid Maps with Forward Sensor Models](https://link.springer.com/article/10.1023/A:1025584807625)
-   [Lab 8: Building Occupancy Grids with TurtleBot](https://pages.github.berkeley.edu/EECS-106/fa21-site/assets/labs/Lab_8__Occupancy_Grids.pdf)
-   [What is an Occupancy Grid Map?](https://automaticaddison.com/what-is-an-occupancy-grid-map/)
-   [RiccardoGiubilato/ros_autonomous_car](https://github.com/RiccardoGiubilato/ros_autonomous_car/blob/master/src/laser_to_occupancy_grid.py)

### In Progress

-   [Occupancy Grid Models for Robot Mapping in Changing Environments](https://ojs.aaai.org/index.php/AAAI/article/view/8377)
    -   Uses Hidden Markov Method to model state changes.
        -   occupancy state
        -   state occupancy prbablities
    -   uses Offline and Online approaches to adapt parameters
        -   Online approach enables dynamically change parameters without storing complete observation sequences
    -   the state of a box, $ z \in \{ hit, miss, no-observation \}$

### Completed

---

## Resources

-   [nav_msgs/OccupancyGrid Message](http://docs.ros.org/en/jade/api/nav_msgs/html/msg/OccupancyGrid.html)
