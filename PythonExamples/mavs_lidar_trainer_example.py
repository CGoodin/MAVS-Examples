''' 
Script for generating labeled lidar and camera data.
Shows how to create a MAVS scene, add it to an environment,
and generate labeled sensor data.
'''
import random
import mavspy.mavs as mavs

# Create the lidar and set the offset
lidar = mavs.MavsLidar('OS1')
lidar.SetOffset([0.0, 0.0, 1.830],[1.0,0.0,0.0,0.0])

# Select a scene and load it
mavs_scenefile = "/scenes/cube_scene.json"
scene = mavs.MavsEmbreeScene()
scene.Load(mavs.mavs_data_path+mavs_scenefile)
scene.TurnOnLabeling()

#--- Load the waypoints that go with this scene ----#
waypoints = mavs.MavsWaypoints()
waypoints.Load(mavs.mavs_data_path+'/waypoints/vehicle2_pos_1.vprp')

#----- Environment Creation ---------------------------#
# Create a MAVS environment and add the scene to it
env = mavs.MavsEnvironment()
env.SetScene(scene)
env.SetTurbidity(float(random.randrange(2,7)))

# loop over all the poses in the waypoints list
for i in range(waypoints.num_waypoints):
    current_position = waypoints.GetWaypoint(i)
    current_orient = waypoints.GetOrientation(i)
    #Set pose of the lidar and scan a frame, saving labeledpoints
    lidar.SetPose(current_position,current_orient)
    lidar.Update(env,0.1)
    lidar.AnnotateFrame(env)
    lidar.AnalyzeCloud('labeled_lidar',i,False)
    lidar.SetDisplayColorType('label')
    lidar.SaveLabeledPcd('labeled_lidar'+str(i).zfill(4)+'.pcd')
    lidar.DisplayPerspective()
    lidar.SaveProjectedLidarImage('labeled_lidar'+str(i).zfill(4)+'.bmp')
