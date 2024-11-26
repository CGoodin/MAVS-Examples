''' 
Driving example with different weather effects.
Create a MAVS vehicle and driving it
with the W-A-S-D keys.
'''
import time
import mavspy.mavs as mavs

# Select a scene and load it
mavs_scenefile = "/scenes/cube_scene.json"
scene = mavs.MavsEmbreeScene()
scene.Load(mavs.mavs_data_path+mavs_scenefile)

# Create a MAVS environment and add the scene to it
env = mavs.MavsEnvironment()
env.SetScene(scene)

# Set environment properties
env.SetTime(13) # 0-23
env.SetRainRate(5.0) # 0-25
env.SetTurbidity(7.0) # 2-10
env.SetAlbedo(0.2) # 0-1
env.SetFog(50) # 0-100
env.SetCloudCover(0.4) # 0-1
env.SetSnow(0.0) # 0-25
env.SetWind([0.0,2.0]) # wind lateral velocity (m/s)

#Create and load a MAVS vehicle
veh = mavs.MavsRp3d()
# vehicle files are in the mavs "data/vehicles/rp3d_vehicles" folder
veh_file = 'forester_2017_rp3d_tires.json'
veh.Load(mavs.mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
# Starting point for the vehicle
veh.SetInitialPosition(100.0, 0.0, 0.0) # in global ENU
# Initial Heading for the vehicle, 0=X, pi/2=Y, pi=-X
veh.SetInitialHeading(0.0) # in radians
veh.Update(env, 0.0, 0.0, 1.0, 0.000001)

# The vehicle we just loaded is actor 0
# The following command adds dust to the vehicle animation
# uncomment to add dust to the vehicle
#env.AddDustToActor(0)

# Create a window for driving the vehicle with the W-A-S-D keys
# window must be highlighted to input driving commands
drive_cam = mavs.MavsCamera()
# nx,ny,dx,dy,focal_len
drive_cam.Initialize(256,256,0.0035,0.0035,0.0035)
# offset of camera from vehicle CG
drive_cam.SetOffset([-12.5,0.0,2.0],[1.0,0.0,0.0,0.0])
# Set camera compression and gain
drive_cam.SetGammaAndGain(0.5,2.0)
# Turn off shadows for this camera if it is slow on your system
drive_cam.RenderShadows(True)

# Create the lidar and set the offset relative to the vehicle CG
lidar = mavs.MavsLidar('VLP16')
lidar.SetOffset([0.0, 0.0, 1.830],[1.0,0.0,0.0,0.0])

# Now start the simulation main loop
dt = 1.0/30.0 # time step, seconds
n = 0 # loop counter
while (True):
    # tw0 is for timing purposes used later
    tw0 = time.time()

    # Get the driving command
    dc = drive_cam.GetDrivingCommand()

    # Update the vehicle with the driving command
    veh.Update(env, dc.throttle, dc.steering, dc.braking, dt)

    # Get the current vehicle position
    position = veh.GetPosition()
    orientation = veh.GetOrientation()

    # This line updates the dust, rain, and snow particle systems
    env.AdvanceTime(dt)

    # Update the animated vehicle position
    # The Ego-Vehicle is always actor 0
    #env.SetActorPosition(0,position,orientation)
    #env.AddDustToLocation(position,veh.GetVelocity(),1.0, 100.0, 0.1)

    # Update the camera sensors at 10 Hz
    # Each sensor calls three functions
    # "SetPose" aligns the sensor with the current vehicle position,
    # the offset is automatically included.
    # "Update" creates new sensor data, point cloud or image
    # "Display" is optional and opens a real-time display window
    if n%3==0:
        # Update the drive camera at 10 Hz
        drive_cam.SetPose(position,orientation)
        drive_cam.Update(env,dt)
        drive_cam.Display()
        lidar.SetPose(position,orientation)
        lidar.Update(env,0.1)
        lidar.Display()
    # uncomment the following lines to get some state
    # variables for the vehicle 
    #long_acc = veh.GetLongitudinalAcceleration()
    #lat_acc = veh.GetLateralAcceleration()
    #front_left_normal_force = veh.GetTireNormalForce(0)

    # Update the loop counter
    n = n+1

    # The following lines ensure that the sim
    # doesn't run faster than real time, which 
    # makes it hard to drive
    tw1 = time.time()
    wall_dt = tw1-tw0
    if (wall_dt<dt):
        time.sleep(dt-wall_dt)