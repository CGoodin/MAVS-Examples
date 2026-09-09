# This demo shows off some of the environmental features on MAVS
# You'll need to have tkinter installed to run it
# And MAVS built from source.
# See: https://mississippi-state-university-otm.github.io/MAVS/docs/MavsBuildInstructions.html
import math
from tkinter import *
# Import MAVS
import mavspy.mavs as mavs
mavs_data_path = mavs.mavs_data_path

# Tkinter sliderbar widget for changing environmental parameters
class EnvironmentWindow():
    def __init__(self):
        self.window = None
        self.darkgrey = '#%02x%02x%02x'%(117, 120, 123) 
    def EditProperties(self,master,env):
        master.configure(background=self.darkgrey)
        if self.window is None:
            self.window = master 
            self.window.title("Environmental Properties")

            self.turbid_slider = Scale(self.window, from_=10.0, to=2.0,background=self.darkgrey,foreground='white')
            self.turbid_slider.set(env.turbidity)
            self.turbid_label = Label(self.window,text="Turbidity",background=self.darkgrey,foreground='white')
            self.turbid_label.grid(column=0,row=0)
            self.turbid_slider.grid(column=0,row=1)

            self.cloud_slider = Scale(self.window, from_=100, to=0,background=self.darkgrey,foreground='white')
            self.cloud_slider.set(100*env.cloud_cover)
            self.cloud_label = Label(self.window,text="Cloud Cover",background=self.darkgrey,foreground='white')
            self.cloud_label.grid(column=1,row=0)
            self.cloud_slider.grid(column=1,row=1)

            self.rain_slider = Scale(self.window, from_=25, to=0,background=self.darkgrey,foreground='white')
            self.rain_label = Label(self.window,text="Rain Rate",background=self.darkgrey,foreground='white')
            self.rain_label.grid(column=2,row=0)
            self.rain_slider.set(env.rain_rate)
            self.rain_slider.grid(column=2,row=1)

            self.fog_slider = Scale(self.window, from_=100, to=0,background=self.darkgrey,foreground='white')
            self.fog_slider.set(env.fog)
            self.fog_label = Label(self.window,text="Fog",background=self.darkgrey,foreground='white')
            self.fog_label.grid(column=3,row=0)
            self.fog_slider.grid(column=3,row=1)

            self.time_slider = Scale(self.window, from_=19, to=6,background=self.darkgrey,foreground='white')
            self.time_slider.set(env.hour)
            self.time_label = Label(self.window,text="Time",background=self.darkgrey,foreground='white')
            self.time_label.grid(column=4,row=0)
            self.time_slider.grid(column=4,row=1)

            self.albedo_label = Label(self.window,text="Albedo",background=self.darkgrey,foreground='white')
            self.albedo_label.grid(column=5,row=0)
            self.albedo_slider = Scale(self.window,from_=100,to=0,background=self.darkgrey,foreground='white')
            self.albedo_slider.set(100*env.albedo)
            self.albedo_slider.grid(column=5,row=1)

            self.snow_slider = Scale(self.window, from_=25, to=0,background=self.darkgrey,foreground='white')
            self.snow_label = Label(self.window,text="Snow Rate",background=self.darkgrey,foreground='white')
            self.snow_label.grid(column=6,row=0)
            self.snow_slider.set(env.snow_rate)
            self.snow_slider.grid(column=6,row=1)

            self.window.protocol('WM_DELETE_WINDOW', self.removewindow)
    def removewindow(self):
        self.window.destroy()
        self.window = None
    def GetFog(self):
        return self.fog_slider.get()
    def GetRain(self):
        return self.rain_slider.get()
    def GetTurbidity(self):
        return self.turbid_slider.get()
    def GetCloudCover(self):
        return 0.01*self.cloud_slider.get()
    def GetTime(self):
        return self.time_slider.get()
    def GetAlbedo(self):
        return (0.01*self.albedo_slider.get())
    def GetSnow(self):
        return (self.snow_slider.get())
    def UpdateEnv(self, env):
        env.SetTime(self.GetTime()) # 0-23
        env.SetFog(self.GetFog()) # 0.0-100.0
        env.SetSnow(self.GetSnow()) # 0-25
        env.SetTurbidity(self.GetTurbidity()) # 2-10
        env.SetAlbedo(self.GetAlbedo()) # 0-1
        env.SetCloudCover(self.GetCloudCover()) # 0-1
        env.SetRainRate(self.GetRain()) # 0-25

if __name__ == "__main__":
    # create the mavs scene and environment
    scene = mavs.MavsEmbreeScene()
    env = mavs.MavsEnvironment()

    # load a mavs scene and add it to the environment
    mavs_scenefile = "/scenes/cube_scene.json"
    scene.Load(mavs_data_path+mavs_scenefile)
    env.SetScene(scene)

    # create the camera sensor
    drive_cam = mavs.MavsCamera()
    # num_hor_pixels, num_vert_pixels, pixplane_hor_dim, pixplane_vert_dim, focal_len
    drive_cam.Initialize(768,512,(768.0/512.0)*0.0035,0.0035,0.0035)
    # offset and orientation relative to the vehicle CG
    drive_cam.SetOffset([-10.0,0.0,3.0],[1.0,0.0,0.0,0.0])
    # camera processing electronics
    drive_cam.SetGammaAndGain(0.9,2.0)
    drive_cam.RenderShadows(True)

    # create the lidar sensor
    lidar = mavs.MavsLidar('VLP-16')
    # offset and orientation relative to the vehicle CG
    lidar.SetOffset([0.0, 0.0, 1.5],[1.0, 0.0, 0.0, 0.0])

    # create a load the MAVS vehicle
    veh = mavs.MavsRp3d()
    veh_file = 'mrzr4_tires_low_gear.json'
    veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
    veh.SetInitialPosition(0.0, 0.0, 0.0) # in global ENU
    veh.SetInitialHeading(math.pi*75.0/180.0) # in radians

    # create the Tkinter master and an instance of the sliderbar editor class
    master = Tk()
    env_window = EnvironmentWindow();
    env_window.EditProperties(master, env);
    
    # set up the simulation variables
    dt = 0.05
    fnum = 0;
    # start the simulation loop
    while env_window.window:
        # update the GUI loop
        master.update_idletasks()
        master.update()

        # get the driving command from the camera window
        # use WASD to drive the vehicle
        dc = drive_cam.GetDrivingCommand()
        # update the vehicle state with the driving command
        veh.Update(env,dc.throttle, dc.steering, dc.braking, dt)

        # update the sensors at 10 Hz
        if (fnum%2==0):
            env_window.UpdateEnv(env)
            drive_cam.SetPose(veh.GetPosition(),veh.GetOrientation())
            drive_cam.Update(env,0.1)
            drive_cam.Display()
            lidar.SetPose(veh.GetPosition(),veh.GetOrientation())
            lidar.Update(env,0.1)
            lidar.Display()
        fnum = fnum+1