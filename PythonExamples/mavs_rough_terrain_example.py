'''
Create a rough terrain from an input .bmp heightmap
'''
import mavspy.mavs as mavs

# Select a scene and load it
mavs_scenefile = "/scenes/heightmap_example_scene.json"
scene = mavs.MavsEmbreeScene()
scene.Load(mavs.mavs_data_path+mavs_scenefile)

# Create a MAVS environment
env = mavs.MavsEnvironment()
# Set environment properties
env.SetTime(19) # 0-23
env.SetTurbidity(10.0) # 2-10
# Add the scene to the environment
env.SetScene(scene)

# Create a camera for rendering the environment
# The MavsPathTraceCamera is slow but makes nice images 
# arguments are 'camera_type', number of rays/pixel, number of reflections per pixel, intensity cutoff threshold.
cam = mavs.MavsPathTraceCamera('custom',400,10,0.55,nx=768,ny=432,h_s=0.00622222,
                                     v_s=0.0035,flen=0.0035,gamma=0.75)

# Set the position and orientation (as a quaternion) of the camera in the scene
cam.SetPose([0.0, 0.0, 1.75],[0.7071, 0.0, 0.0, -0.7071])
# Update the camera (generate an image)
cam.Update(env,0.05)
# Display the image
cam.Display()
# Save the image
cam.SaveCameraImage('rough_surface_image.bmp')
