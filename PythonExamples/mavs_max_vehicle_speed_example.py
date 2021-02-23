'''
Example that simulates a maximum speed on grade test for a user defined surface.
You can set the surface type, soil strength, and slope and get the max speed.
'''
import sys
# Set the path to the mavs python api, mavs_interface.py
sys.path.append(r'C:\your\path\to\mavs\src\mavs_python')
# Load the mavs python modules
import mavs_interface
import mavs_python_paths
# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path

# Set the soil and terrain properties
# soil type can be 'dry', 'wet', 'snow', 'clay', or 'sand'.
soil_type = 'dry'
# soil strenght in PSI - only used when soil_type is 'sand' or 'clay'
soil_strength = 100.0
# fractional surface slope, 0.5 = 26.5 degrees, 1 = 45 degrees
surface_slope = 0.0

# Load a MAVS scene, the exact scene doesn't matter for this test
scene = mavs_interface.MavsEmbreeScene()
scene.Load(mavs_data_path + "/scenes/cube_scene.json")

# Create a MAVS environment and add the scene to it
env = mavs_interface.MavsEnvironment()
env.SetScene(scene.scene)

#Create and load a MAVS vehicle
veh = mavs_interface.MavsRp3d()
# vehicle files can be found in mavs/data/vehicles/rp3d_vehicles
veh_file = 'forester_2017_rp3d.json'
veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
veh.SetInitialPosition(0.0, 0.0, 0.0) # in global ENU
veh.SetInitialHeading(0.0) # in radians

# Set the terrain properties. This will overide the scene geometry already loaded
# check out the API documentation for the parameter definitions
# https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/classmavs__python_1_1mavs__interface_1_1_mavs_rp3d.html#a521cacdc4ed5a84bdbed81a2b0b5b034
veh.SetTerrainProperties(terrain_type='sloped',terrain_param1=surface_slope, terrain_param2=0.0, soil_type=soil_type, soil_strength=soil_strength)

# Load waypoints and create controller to follow them
# Waypoint files are in mavs/data/waypoints
# These waypoints are just a straight line
waypoints = mavs_interface.MavsWaypoints()
waypoints_file = 'x_axis_points.vprp'
waypoints.Load(mavs_data_path+'/waypoints/'+waypoints_file)
waypoints.FillIn(0.5)
# Now create a vehicle controller to adjust the throttle and steering
# Check the API for more info on the params
# https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/classmavs__python_1_1mavs__interface_1_1_mavs_vehicle_controller.html
controller = mavs_interface.MavsVehicleController()
controller.SetDesiredPath(waypoints.GetWaypoints2D())
controller.SetDesiredSpeed(200.0) # m/s, set to an absurdly high number 
controller.SetSteeringScale(2.35)
controller.SetWheelbase(3.8) # meters
controller.SetMaxSteerAngle(0.855) # radians
controller.TurnOnLooping()

# Set up some simulation parameters
dt = 1.0/100.0 # time step, seconds
time_elapsed = 0.0 # total elapsed time, in seconds
velocity = 0.0 # track the velocity of the vehicle to determine when max is reached
n = 0 # loop counter

# start the simulation 
while (time_elapsed<100000.0):

    # Get the driving command from the controller
    controller.SetCurrentState(veh.GetPosition()[0],veh.GetPosition()[1], veh.GetSpeed(),veh.GetHeading())
    dc = controller.GetDrivingCommand(dt)

    # Update the vehicle
    veh.Update(env, dc.throttle, dc.steering, dc.braking, dt)

    # check if max speed is reached
    new_velocity = veh.GetSpeed()
    if (new_velocity<=velocity and time_elapsed>5.0):
        print('Max Speed = '+str(velocity)+' m/s')
        sys.exit()
    velocity = new_velocity

    ## Print the state every half second
    if n%50==0:
        print(n*dt,new_velocity)
        sys.stdout.flush()

    n = n+1
    time_elapsed = time_elapsed + dt
