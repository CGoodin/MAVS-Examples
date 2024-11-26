''' 
Us MAVS to create a skid steered vehicle,
in this case the Clearpath Warthog.
Drive it with the W-A-S-D keys.
'''
import math
import mavspy.mavs as mavs

# create a path for the vehicle to follow
path_amplitude = 5.0
path_length = 100.0
desired_path = []
x = 0.0
while x<path_length:
    y = path_amplitude*math.sin(0.1*x)
    desired_path.append([x,y])
    x = x + 1.0

# Load a MAVS scene and add it to the environment
mavs_scenefile = "/scenes/surface_only.json"
scene = mavs.MavsEmbreeScene()
scene.Load(mavs.mavs_data_path+mavs_scenefile)
env = mavs.MavsEnvironment()
env.SetScene(scene)

# Create a MAVS vehicle
veh = mavs.MavsRp3d()
veh_file = 'clearpath_warthog_cpu_tires.json'
veh.Load(mavs.mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
veh.SetInitialPosition(0.0, 0.0, 0.0) # in global ENU
veh.SetInitialHeading(0.0) # in radians

# Add a camera for visualizing the results
front_cam = mavs.MavsCamera()
front_cam.Initialize(256,256, 0.0035,0.0035,0.0035)
front_cam.SetGammaAndGain(0.5,2.0)
front_cam.RenderShadows(True)
front_cam.SetOffset([-15.0, 0.0, 2.0],[1.0, 0.0, 0.0, 0.0])

# Start the simulatio nmain loop
dt = 1.0/120.0 # time step, seconds
n = 0 # loop counter
while veh.GetPosition()[0]<0.95*path_length:

    # Get the driving command
    dc = front_cam.GetDrivingCommand()
    
    # Update the vehicle with the driving command
    veh.Update(env, dc.throttle, dc.steering, dc.braking, dt)

    # Update the visualization at 30 Hz
    if n%4==0:
        front_cam.SetPose(veh.GetPosition(),veh.GetOrientation())
        front_cam.Update(env,0.05)
        front_cam.Display()
    n = n+1
