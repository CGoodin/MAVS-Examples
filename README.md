# MAVS-Examples
This repo contains example applications using the MSU Autonomous Vehicle Simulator (MAVS).

MAVS is a software library for simulating autonomous ground vehicles in off-road terrain. MAVS simulates the sensors, vehicle, and environment. It uses physics-based models to simulate camera, lidar, and radar interacting with environmental features such as rain, dust, and fog.


These examples use the Python interface to MAVS, [MavsPy](https://pypi.org/project/mavspy/). 

MavsPy can be installed using the Python package installer, PiP, with the following command:
```
$pip install mavspy
```

Once installed, these examples can be run using Python:
```
$cd PythonExamples
$python mavs_driving_example.py
```
***Note*** - *MAVS may create a file called "mavs_config.txt" in your local directory when it runs. It points Python to the local installation of your MAVS data directory.*

## Citing MAVS
If you use MAVS for your research, please cite the following:
* Hudson, C., Goodin, C., Miller, Z., Wheeler, W., & Carruth, D. (2020, August). Mississippi state university autonomous vehicle simulation library. In Proceedings of the Ground Vehicle Systems Engineering and Technology Symposium (pp. 11-13).
* Goodin, C., Carruth, D. W., Dabbiru, L., Hudson, C. H., Cagle, L. D., Scherrer, N., ... & Jayakumar, P. (2022, June). Simulation-based testing of autonomous ground vehicles. In Autonomous Systems: Sensors, Processing and Security for Ground, Air, Sea and Space Vehicles and Infrastructure 2022 (Vol. 12115, pp. 167-174). SPIE.

## Other Documentation

Software documentation for the MAVS library is here:

https://mavs-documentation.readthedocs.io/en/latest/

The MAVS-API is documented here:

https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/
