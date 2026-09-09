import sys
import os
import math
import matplotlib.pyplot as plt
# packages for making the final movie
import moviepy.video.io.ImageSequenceClip
from PIL import Image

# Import MAVS
import mavspy.mavs as mavs
mavs_data_path = mavs.mavs_data_path

def SimulateImu():
    #------------------ scene -------------------------------------------------------------------#
    scene = mavs.MavsEmbreeScene()
    mavs_scenefile = "/scenes/valley_big.json"
    scene.Load(mavs_data_path+mavs_scenefile)

    #------------------ environment -------------------------------------------------------------#
    env = mavs.MavsEnvironment()
    env.SetScene(scene)
    env.SetFog(10.0) # 0-100
    env.SetRainRate(0)
    env.SetTime(17)

    #------------------ vehicle -----------------------------------------------------------------#
    veh = mavs.MavsRp3d()
    veh_file = 'mrzr4_tires_low_gear.json'
    veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
    veh.SetInitialPosition(-75.0, 0.0, 2.0) # in global ENU
    veh.SetInitialHeading(0.0) # in radians

    #------------------ camera --------------------------------------------------------------------#
    drive_cam = mavs.MavsCamera() 
    # nx,ny,dx,dy,focal_len
    drive_cam.Initialize(1024,576,(1920.0/1080.0)*0.0035,0.0035,0.0035) 
    drive_cam.RenderShadows(True) # Turn on/off shadows for this camera 
    drive_cam.SetAntiAliasingFactor(3)
    drive_cam.SetOffset([0.0,7.5,1.0],[math.cos(-0.25*math.pi),0.0,0.0,math.sin(-0.25*math.pi)])
    drive_cam.SetSaturationAndTemp(1.05, 7500.0)
    drive_cam.Update(env, 0.1)

    #------------------ mavs imu ------------------------------------------------------------------#
    imu = mavs.MavsMems('accelerometer')
    imu.SetMeasurementRange(55.0)
    imu.SetAccelerationBias([0.0,0.0,-9.806])
    imu.SetRandomWalk(0.0001)
    ambient_temp = 20.0

    #------------------ main loop ---------------------------------------------------------#
    dt = 1.0/100.0 # time step, seconds
    n = 0 # loop counter
    desired_speed = 5.0
    throttle = 0.0
    px = veh.GetPosition()[0]
    twait = 2.0
    accels = [] 
    true_a = [] 
    times = [] 
    elapsed_time = 0.0

    goal = [200.0, 0.0]
    while px<200.0 and elapsed_time<180.0:
        # simple steering control
        pos = veh.GetPosition()
        px = pos[0]
        dx = goal[0]-pos[0]
        dy = goal[1]-pos[1]
        heading = math.atan2(dy, dx)
        steering = 0.125*heading;
    
        # simple speed control
        speed = veh.GetSpeed();
        if (speed<desired_speed):
            throttle=throttle+0.01
        elif (speed>desired_speed):
            throttle = throttle-0.025
        throttle = max(0.0, min(1.0, throttle))

        # Update the vehicle with the driving command
        veh.Update(env, throttle, steering, 0.0, dt) 
    
        # get the true vehicle state
        _,_,_,angular_velocity,linear_acceleration,_ = veh.GetFullState()
    
        # simulate the IMU sensor
        acc = imu.Update(linear_acceleration, ambient_temp, 100.0)

        # populate the arrays for the plots
        if (elapsed_time>=twait):
            if (n<501):
                accels.append(acc[2])
                true_a.append(linear_acceleration[2])
                times.append(elapsed_time)
            else:
                accels.append(acc[2])
                accels.pop(0)
                true_a.append(linear_acceleration[2])
                true_a.pop(0)
                times.append(elapsed_time)
                times.pop(0)
    
        # update the camera and z-accel plat at 50 Hz
        if n%2==0 and elapsed_time>=twait:
            # update the camera and save image
            cam_position = veh.GetPosition()
            yaw = veh.GetHeading()
            cam_orientation = [math.cos(0.5*yaw), 0.0, 0.0, math.sin(0.5*yaw)]
            #cam_position[2]=1.25
            drive_cam.SetPose(cam_position,cam_orientation)
            drive_cam.Update(env,0.05)
            drive_cam.Display()
            drive_cam.SaveCameraImage('image_'+str(n).zfill(4)+'.bmp')
            # Plot the acceleration and save image
            plt.figure(figsize=(10.24, 5.12))
            plt.plot(times, accels, label='Measured Accel')
            plt.plot(times, true_a, label='True Accel', color='red', linestyle='--')
            plt.xlabel("Time (s)")
            plt.ylabel("Accel (g)")
            plt.savefig('timeseries_plot'+str(n).zfill(4)+'.png', format='png')
            plt.close() 
        # update loop variables    
        n = n+1
        elapsed_time = elapsed_time + dt
        
def MakeVideo():
    image_folder='./'
    fps=50

    image_files = [image_folder+'/'+img for img in sorted(os.listdir(image_folder)) if img.endswith(".bmp")]
    acc_files = [image_folder+'/'+img for img in sorted(os.listdir(image_folder)) if img.endswith(".png")]

    for i in range(len(image_files)):
        image1 = Image.open(image_files[i])
        image2 = Image.open(acc_files[i])

        # Get the dimensions of the images
        width1, height1 = image1.size
        width2, height2 = image2.size

        # Create a new image with the combined width and the height of the tallest image
        new_height = height1 + height2
        new_width = max(width1, width2)
        new_image = Image.new("RGB", (new_width, new_height))

        # Paste the two images onto the new image
        new_image.paste(image1, (0, 0))
        #new_image.paste(image2, (width1, 0))
        new_image.paste(image2, (0, height1))

        # Save the new image
        newname = str(i).zfill(5)+"_merged.bmp"
        new_image.save(newname)
    merged_files = [image_folder+'/'+img for img in sorted(os.listdir(image_folder)) if img.endswith("_merged.bmp")]
    clip = moviepy.video.io.ImageSequenceClip.ImageSequenceClip(merged_files, fps=fps)
    clip.write_videofile('mavs_imu_demo_video.mp4')
    os.system("rm image_*.bmp *_merged.bmp timeseries_*.png")
    
if __name__ == "__main__":
    SimulateImu()
    MakeVideo()