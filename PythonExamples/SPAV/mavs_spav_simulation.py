"""Create a MAVS simulation environment.

This script creates a MAVS environment, loads a scene,
loads a vehicle, creates sensors, and initialized a 
waypoint following module.

This is the initialization step for
sim_example_keystrokes.py
and 
autonomy_example.py

Note that it may take 1-2 minutes for everything to load.
"""
# The first step is to make a system call to load the python interface modules
import sys
# Set the path to the mavs python api, mavs.py
# You will need to edit the following path to match the location on your computer
sys.path.append(r'C:/mavs-binaries/mavs_python')
# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths

class MavsSpavSimulation(object):
    def __init__(self):

        # Set the path to the mavs data folder
        mavs_data_path = mavs_python_paths.mavs_data_path

        # https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/classmavs__python_1_1mavs__interface_1_1_mavs_scene.html
        # Specify a scene and load it
        mavs_scenefile = "/scenes/spa_city.json"
        self.scene = mavs.MavsEmbreeScene()
        self.scene.Load(mavs_data_path+mavs_scenefile)

        # Turn on automatic labeling of the scene
        self.scene.TurnOnLabeling()

        # https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/classmavs__python_1_1mavs__interface_1_1_mavs_environment.html
        # Create a MAVS environment and add the scene to it
        self.env = mavs.MavsEnvironment()
        self.env.SetScene(self.scene)

        # Set environment properties
        self.env.SetTime(13) # 0-23
        self.env.SetFog(50.0) # 0.0-100.0
        self.env.SetRainRate(0.0) # 0-25
        self.env.SetWind( [2.5, 1.0] ) # Horizontal windspeed in m/s

        # https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/classmavs__python_1_1mavs__interface_1_1_mavs_camera.html
        # Create a MAVS camera and set its properties
        self.cam = mavs.MavsCamera()
        # num_horizontal_pix, num_vert_pix, horiz_image_plane_size_meters, vert_image_plane_size_meters, focal_length_meters
        self.cam.Initialize(640,360,0.006222,0.0035,0.0035)
        self.cam.RenderShadows(True)
        # Increasing anti-aliasing will slow down simulation but gives nicer images
        self.cam.SetAntiAliasingFactor(4)
        # If raining, render drops splattered on the lens
        self.cam.SetDropsOnLens(True)
        # Set the gamma (0-1.0) and gain (0-2.0) of the camera
        self.cam.SetGammaAndGain(0.5,2.0)
        # Set the camera offsets. 
        # This is the offset of the sensor from vehicle from the CG. 
        self.cam.SetOffset([1.0, 0.0, 2.0],[1.0,0.0,0.0,0.0])

        # Create a camera window for driving the vehicle with the W-A-S-D keys
        # window must be highlighted to input driving commands
        self.drive_cam = mavs.MavsCamera()
        # nx,ny,dx,dy,focal_len
        self.drive_cam.Initialize(512,512,0.0035,0.0035,0.0035)
        # offset of camera from vehicle CG
        self.drive_cam.SetOffset([-10.0,0.0,3.0],[1.0,0.0,0.0,0.0])
        # Set camera compression and gain
        self.drive_cam.SetGammaAndGain(0.75,2.0)
        # Turn off shadows for this camera for efficiency purposes
        self.drive_cam.RenderShadows(True)
        # Turn on anti-aliasing
        self.drive_cam.SetAntiAliasingFactor(3)

        # https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/classmavs__python_1_1mavs__interface_1_1_mavs_lidar.html
        # Create a MAVS lidar and set its properties
        self.lidar = mavs.MavsLidar('VLP-16')
        # Set the same offset as the camera
        self.lidar.SetOffset([1.0, 0.0, 2.0],[1.0,0.0,0.0,0.0])

        # https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/classmavs__python_1_1mavs__interface_1_1_mavs_rp3d.html
        #Create and load a MAVS vehicle
        self.veh = mavs.MavsRp3d()
        # vehicle files are in the mavs "data/vehicles/rp3d_vehicles" folder
        veh_file = 'forester_2017_rp3d_tires.json'
        self.veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
        # Starting point for the vehicle
        self.veh.SetInitialPosition(10.0, 7.5, 0.0) # in global ENU
        # Initial Heading for the vehicle, 0=X, pi/2=Y, pi=-X
        self.veh.SetInitialHeading(0.0) # in radians
        # Do a short update step for MAVS to finish loading the vehicle
        self.veh.Update(self.env, 0.0, 0.0, 1.0, 0.000001)

        # https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/classmavs__python_1_1mavs__interface_1_1_mavs_environment.html#ab8e46c79f0817e4056a64561c4e5cbfb
        # Add actors to the scene, in this case two vehicles which will drive around
        # These have to be added after the main vehicle is loaded
        # Actors in MAVS are any dynamic object in the scene other than the ego-vehicle
        self.env.AddActor(mavs_data_path+'/actors/actors/l200_actor_top_loop.json')
        self.env.AddActor(mavs_data_path+'/actors/actors/lambo_actor_outer_loop.json')

        # https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/classmavs__python_1_1mavs__interface_1_1_mavs_waypoints.html
        # Load waypoints for the vehicle to follow 
        waypoints = mavs.MavsWaypoints()
        # waypoints files are in the data/waypoints folder
        #waypoints_file = 'spa_city_outer_loop.vprp'
        waypoints_file = 'spa_city_top_block.vprp'
        waypoints.Load(mavs_data_path+'/waypoints/'+waypoints_file)

        # https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/classmavs__python_1_1mavs__interface_1_1_mavs_vehicle_controller.html
        # Create a vehicle controller to follow the waypoints
        self.controller = mavs.MavsVehicleController()
        # Tell the controller to follow the waypoints loaded above
        self.controller.SetDesiredPath(waypoints.GetWaypoints2D())
        # Tell the controller how fast to drive the vehicle
        self.controller.SetDesiredSpeed(5.0) # m/s 
        # The next three lines set controller parameters that
        # steer the vehicle. The vehicle steering may be sensitive
        # to these, so don't tinker with them too much
        self.controller.SetSteeringScale(6.0)
        self.controller.SetWheelbase(3.3) # meters
        self.controller.SetMaxSteerAngle(0.855) # radians
        # this makes the vehicle drive the loop repeatedly
        self.controller.TurnOnLooping()

if __name__ == "__main__":
    # test the loading of the simulation class
    sim = MavsSpavSimulation()