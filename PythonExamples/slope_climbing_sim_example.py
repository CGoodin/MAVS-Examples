import math
import sys
# Set the path to wheverver you cloned and built the MAVS software
# see: https://mississippi-state-university-otm.github.io/MAVS/docs/MavsBuildInstructions.html
sys.path.append(r'C:/Users/YourUserName/PathToMavs/mavs/src/mavs_python')

# import MAVS and set the default data path
import mavs_interface as mavs
import mavs_python_paths
mavs_data_path = mavs_python_paths.mavs_data_path

def CreateTerrains(slope_angle_min_deg, slope_angle_max_deg, slope_angle_step_deg):
    # create the different sloped terrains  
    # using the "AddTrapezoidalFeature" of MAVS
    sloped_terrains = []
    terrain_length = 1000.0 # meters
    slope_degrees = slope_angle_min_deg
    while slope_degrees<=slope_angle_max_deg:
        sloped_terrain = mavs.MavsTerrainCreator()
        height = (0.5*terrain_length)*math.tan(math.radians(slope_degrees))
        offset = 15.0 + 0.5*terrain_length
        # Inputs are width of the top/bottom of the feature, width of the base of the feature, height/depth of the feature (positive=depth), and x-offset from center of the terrain
        sloped_terrain.AddTrapezoidalFeature(0.0, terrain_length, -height, offset)
        sloped_terrains.append(sloped_terrain)
        slope_degrees += slope_angle_step_deg
    return sloped_terrains

def RunSimulations(terrains_to_sim):
    # create a camera for rendering the simulations
    cam = mavs.MavsCamera()
    # Hor. Pixnum, Vert. Pixnum, Hor FP width, Vert FP height, focal_length
    cam.Initialize(960,540,0.00525,0.0035,0.0035)
    # position and orientation relative to vehicle CG
    cam.SetOffset([0.0,7.0,1.0],[0.7071,0.0,0.0,-0.7071])
    # Camera compression and gain
    cam.SetGammaAndGain(0.85,1.0)
    cam.RenderShadows(True)

    frame_num = 0
    for i in range(len(terrains_to_sim)):
        # create the MAVS scene from the terrain definition
        # inputs are xmin, ymin, xmax, ymax, resolution
        scene_ptr = terrains_to_sim[i].CreateMavsScenePointer(-50.0, -25.0, 200.0, 25.0, 0.25 )

        # Create a MAVS environment and add the scene to it
        env = mavs.MavsEnvironment()
        env.SetScene(scene_ptr)
        env.SetTime(19) # 0-23

        # Load the vehicle to test
        veh = mavs.MavsRp3d()
        veh_file = 'mrzr4_tires_low_gear.json'
        veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
        veh.SetInitialPosition(0.0, 0.0, 0.0) # in global ENU
        veh.SetInitialHeading(0.0) # in radians

        # create a throttle controller for the vehicle and set the desired speed to 5 m/s
        pid = mavs.PidController(kp=0.5, ki=0.1, kd=0.05, setpoint=5.0)

        # Run the simulation at 100 Hz
        dt = 1.0/100.0 
        elapsed_time = 0.0
        n = 0     
        while (elapsed_time<10.0): # let each sim run for 10 seconds
            # Update the controller setting with the vehicles current state
            throttle = pid.Update(veh.GetSpeed())
            # Update the vehicle using the desired setting
            veh.Update(env, throttle, 0.0, 0.0, dt)

            # Render at ~33 FPS
            if n%3==0 and elapsed_time>1.0: 
                # get the vehicle position and orientation for the camera
                pos = veh.GetPosition()
                yaw = veh.GetHeading()
                ori = [math.cos(0.5*yaw), 0.0, 0.0, math.sin(0.5*yaw)]
                cam.SetPose(pos,ori)
                cam.Update(env,dt)
                cam.Display()
                # Uncomment the line below to save images to file
                #cam.SaveCameraImage(str(frame_num).zfill(5)+"_image.bmp")
                frame_num += 1
        
            n = n+1
            elapsed_time = elapsed_time + dt
        
if __name__=="__main__":
    # create the slopes for testing, defining the slope min, max, and step in degrees
    sloped_terrains = CreateTerrains(5.0, 45.0, 5.0)
    
    # run the simulations
    RunSimulations(sloped_terrains)