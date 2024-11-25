'''
Example of using the MavsRandomScene to generate a scene
and then measure the vegetation density of the scene with the
"GetVegDensityOnAGrid" method. Shows how to plot the veg density
in elevation slices
'''
import matplotlib.pyplot as plt
import numpy as np
import mavspy.mavs as mavs

terrain_width = 50.0
terrain_length = 50.0

# Select a scene and load it
mavs_scenefile = "/scenes/grass_dense.json"
scene = mavs.MavsEmbreeScene()
scene.Load(mavs.mavs_data_path+mavs_scenefile)

# Create a MAVS environment
env = mavs.MavsEnvironment()
# Add the scene to the environment
env.SetScene(scene)

# Render a top-down view of the scene for reference later
cam = mavs.MavsCamera()
cam.Initialize(512,512,0.0035,0.0035,0.0035)
cam.SetOffset([0.0,0.0,0.0],[1.0,0.0,0.0,0.0])
cam.SetGammaAndGain(0.6,1.0)
cam.SetPose([0.0, 0.0, max(10,1.25*terrain_width)],[0.7071, 0.0, 0.7071, 0.0])
cam.Update(env,0.03)
cam.Display()
cam.SaveCameraImage("top_down.bmp")

# Get the density grid
# Arguments are : 
# Lower left corner in ENU
# Upper right corner in ENU
# grid resolution in meters
density_grid = env.GetVegDensityOnAGrid([-0.5*terrain_width,-0.5*terrain_length,0.0],
                                        [0.5*terrain_width,0.5*terrain_length,20],1.0)

# convert the python list to a numpy array
veg_dens = np.array(density_grid)

# get the shape of the numpy array
nx,ny,nz = veg_dens.shape

# plot the vegetation density slices, one by one
for k in range(nz):
    slice = veg_dens[:,:,k]
    snx = len(slice)
    sny = len(slice[0])
    plt.xlim((nx,0))
    plt.margins(0,0)
    plt.axis('off')
    plt.imshow(slice,interpolation='none')
    plt.savefig('slice_'+str(k)+'.png',bbox_inches='tight')