"""Run a MAVS simulation to generate labeled sensor data

Run this example with the control mode as a command line argument.
Either
python spa_sim_example.py human
or
python spa_sim_example.py waypoints

Pressing the 'c' key during the simulation will capture
the current frame and save a labeled and raw version.
"""
# import the MAVS simulation loader
from mavs_spav_simulation import MavsSpavSimulation
# import other python functions
import time
import keyboard
import sys
from PIL import Image
import os

# get the control mode (human or waypoints)
# from the command line
control_mode = 'human'
if (len(sys.argv)>1):
    control_mode = sys.argv[1]

# create a folder for the saved data
output_folder = 'output_data'
if not os.path.isdir(output_folder):
    os.makedirs(output_folder)

# Create the simulation
sim = MavsSpavSimulation()

# Set display options for the lidar
# can be 'height', 'color', 'range', 'intensity', 'label', or 'white'
sim.lidar.SetDisplayColorType("color")

# Start the simulation main loop
dt = 1.0/30.0 # time step, seconds
n = 0 # loop counter
while (True):
    # tw0 is for timing purposes used later
    tw0 = time.time()

    # Get the driving command, either from the WASD keys or the controller
    if (control_mode=='human'):
        dc = sim.drive_cam.GetDrivingCommand()
    else:
        sim.controller.SetCurrentState(sim.veh.GetPosition()[0],sim.veh.GetPosition()[1],
                                   sim.veh.GetSpeed(),sim.veh.GetHeading())
        dc = sim.controller.GetDrivingCommand(dt)

    # Update the vehicle with the driving command
    sim.veh.Update(sim.env, dc.throttle, dc.steering, dc.braking, dt)

    # Get the current vehicle position
    position = sim.veh.GetPosition()
    orientation = sim.veh.GetOrientation()

    # The AdvanceTime method updates the other actors
    sim.env.AdvanceTime(dt)

    # Update the sensors at 10 Hz
    # Each sensor calls three functions
    # "SetPose" aligns the sensor with the current vehicle position,
    # the offset is automatically included.
    # "Update" creates new sensor data, point cloud or image
    # "Display" is optional and opens a real-time display window
    if n%3==0:
        # Update the camera
        sim.cam.SetPose(position,orientation)
        # Update the drive camera
        sim.drive_cam.SetPose(position,orientation)
        sim.drive_cam.Update(sim.env,dt)
        sim.drive_cam.Display()
        # Update the lidar 
        sim.lidar.SetPose(position,orientation)
        sim.lidar.Update(sim.env,dt)
        sim.lidar.DisplayPerspective()

        # Save labeled data fram when 'c' key is pressed
        if keyboard.is_pressed('c'):
            # print feedback that the frame is being saved
            print('Saving frame ' + str(n))
            sys.stdout.flush()

            # update the camera sensor
            sim.cam.Update(sim.env,dt)

            # save the camera data
            im_name = (output_folder+'/'+str(n)+'_image')
            sim.cam.SaveCameraImage(im_name+'.bmp')

            # convert it to jpg for later
            img = Image.open(im_name+'.bmp')
            img.save(im_name+'.jpg' , 'JPEG', quality=100)

            # save the annotated camera frame
            # don't include file extension, it's automatically .bmp
            sim.cam.SaveAnnotation(sim.env,im_name+'_annotated')

            # and convert the annotated from to jpg
            label_img = Image.open(im_name+'_annotated.bmp')
            label_img.save(im_name+'_annotated.jpg' , 'JPEG', quality=100)

            # Save annotated lidar point cloud
            sim.lidar.AnnotateFrame(sim.env)
            sim.lidar.SaveLabeledPcd(output_folder+'/'+str(n)+'_labeled.pcd')    

    # Update the loop counter
    n = n+1

    # The following lines ensure that the sim
    # doesn't run faster than real time, which 
    # makes it hard to drive
    tw1 = time.time()
    wall_dt = tw1-tw0
    if (wall_dt<dt):
        time.sleep(dt-wall_dt)