"""Run a MAVS simulation with autonomy 'in-the-loop'

Run this example:
python sim_example_autonomy.py
"""
# import the MAVS simulation loader
from mavs_spav_simulation import MavsSpavSimulation
# Import the additional autonomy modules
import autonomy
import astar
# Import some python functions we'll need
from math import sqrt
import time

# Create and occupancy grid and resize it
grid = autonomy.OccupancyGrid()
grid.resize(400,400)
# Set the resolution (in meters) of the grid
grid.info.resolution = 1.0
# Set the origin of the grid (lower left corner)
grid.set_origin(-200.0,-200.0)
# Define the goal point on our map and convert it to a grid index 
goal_point = [62.75,7.5]
goal_index = grid.coordinate_to_index(goal_point[0],goal_point[1])

# Create the simulation
sim = MavsSpavSimulation()

# The dist_to_goal value will be used to test completion
# For now, just set it to a high value to start the simulation
dist_to_goal = 1000.0

# Start the main simulation loop
dt = 1.0/30.0 # time step, seconds
n = 0 # loop counter
while dist_to_goal > 4.0:
    # a timing value to be used later
    tw0 = time.time()

    # Update the driving command using the controller
    sim.controller.SetCurrentState(sim.veh.GetPosition()[0],sim.veh.GetPosition()[1],
                                   sim.veh.GetSpeed(),sim.veh.GetHeading())
    dc = sim.controller.GetDrivingCommand(dt)

    # Update the vehicle
    sim.veh.Update(sim.env, dc.throttle, dc.steering, dc.braking, dt)

    # Get the current vehicle position
    position = sim.veh.GetPosition()
    orientation = sim.veh.GetOrientation()

    # The AdvanceTime method updates the other actors
    sim.env.AdvanceTime(dt)

    # Check to see if we've advanced towards our goal
    dist_to_goal = sqrt(pow(position[0]-goal_point[0],2)+pow(position[1]-goal_point[1],2))

    # Update the sensors and recalculate the path at 10 Hz
    if n%3==0 and n>0:
        # Update and display the drive camera
        # which is for visualization purposes only.
        sim.drive_cam.SetPose(position,orientation)
        sim.drive_cam.Update(sim.env,dt)
        sim.drive_cam.Display()  
        
        # Update and display the lidar, which will be used
        # by the A* algorithm
        sim.lidar.SetPose(position,orientation)
        sim.lidar.Update(sim.env,dt)
        sim.lidar.Display()    

        # Get lidar point cloud registered to world coordinates
        registered_points = sim.lidar.GetPoints()
        # add the points to the grid
        grid.add_registered_points(registered_points)
        # determine the grid index of the current vehicle location
        current_grid_index = grid.coordinate_to_index(position[0],position[1])
        # calculate a path through the occupancy grid using A*
        path = astar.astar(grid.data,(current_grid_index[0],current_grid_index[1]),(goal_index[0],goal_index[1]))
        # if the path is valid, set it as the new path for the controller
        if path:
            # first convert it back to ENU coordinates
            path_enu = grid.index_path_to_coordinates(path)
            # Update the controller path
            sim.controller.SetDesiredPath(path_enu)

    # Update the loop counter
    n = n + 1

    # Check the timing to make sure we don't exceed real-time
    tw1 = time.time()
    wall_dt = tw1-tw0
    if (wall_dt<dt):
        time.sleep(dt-wall_dt)
    



