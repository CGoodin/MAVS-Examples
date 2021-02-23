'''
Example of using the MavsRandomScene to generate a rough surface.
Explains parameters for creating a rough surface and makes a rendering.
'''
import sys
# Set the path to the mavs python api, mavs.py
sys.path.append(r'C:\your\path\to\mavs\src\mavs_python')
# Load the mavs python modules
import mavs_interface as mavs

# Create a MAVS Random Scene
random_scene = mavs.MavsRandomScene()
random_scene.terrain_width = 100.0 # meters
random_scene.terrain_length = 100.0 # meters
# The lowest frequency of noise will be 2/min(terrain_width,terrain_length)
# The highest frequency of noise will be 1/(2*mesh_resolution)
random_scene.mesh_resolution=0.15
# roughness type can be 'variable', 'gaussian', or 'perlin'
random_scene.surface_roughness_type = "variable"
# When surface_roughness_type='variable', this will also generate a file called
# 'rms_truth.txt' that gives the RMS roughness (in meters) on an ENU grid.
random_scene.lo_mag = 0.0 # Always set this to 0 when using 'variable' roughness
random_scene.hi_mag = 0.1 # The magnitude of the highest frequency noise, should be <0.15 in meters
random_scene.plant_density = 0.05 # 0-1
# Set the trail parameters to zero if you don't want a trail
random_scene.trail_width = 0.0 
random_scene.track_width = 0.0
random_scene.wheelbase = 0.0
# The scene name can be whatever you choos
random_scene.basename = 'bumpy_surface'
# Choose from the ecosystem files in mavs/data/ecosystems
random_scene.eco_file = 'american_southwest_desert.json'
# Have the trail follow 'Ridges', 'Valleys', or 'Loop'
random_scene.path_type = 'Ridges'
# Create the scene with the properties defined above
random_scene.CreateScene()

# Create a MAVS environment
env = mavs.MavsEnvironment()
# Set environment properties
env.SetTime(19) # 0-23
env.SetTurbidity(10.0) # 2-10
# Add the scene to the environment
env.SetScene(random_scene.scene)

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
