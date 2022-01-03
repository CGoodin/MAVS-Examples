# SPAV Examples
The examples in this folder are specific to the  *Sensing and Perception for Autonomous Vehicles* (SPAV) course at Mississippi State University.

**IMPORTANT** : To run these examples, you must edit line 18 of 
'mavs_spav_simulation.py' to have the file path to YOUR installation
of MAVS.

## Description of Files
* *mavs_spav_simulation.py* - Loads the vehicle, sensors, and scene used in the sim.
* *autonomy.py* & *astar.py* - Simple autonomy algorithms for demonstrating "in-the-loop" sims.
* *sim_example_keystrokes.py* - Example simulation where labeled data can be generated.
* *sim_example_autonomy.py* - Example "in-the-loop" simulation with MAVS.

## Prequisites

MAVS - https://mavs-documentation.readthedocs.io/en/latest/InstallingMavsBinaries/

Python Image Library - pip install pillow

NumPy - pip install numpy

## Running the Examples
To run the autonomous driving example:
```shell
$python sim_example_autonomy.py
```

The data generation example can be run in waypoint following or tele-op mode. To run in tele-op mode:
```shell
$python sim_example_key.py human
```
To run in waypoint mode:
```shell
$python sim_example_key.py waypoints
```

In both cases, a snapshot of a particular frame can be grabbed by highlighting the camera window and pressing 'c'. This will capture one frame of sematically labeled lidar and camera data and save it to a subfolder called 'output_data'.