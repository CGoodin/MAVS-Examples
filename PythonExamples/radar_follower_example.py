import os
import sys
import moviepy.video.io.ImageSequenceClip
from PIL import Image
import sys
import math

#--- Import MAVS --------------------------------------------------------------------- #
# If you built MAVS from source, modify the following lines to match your install location
sys.path.append(r'C:/Users/cgoodin/Desktop/goodin_docs/repos/mavs/src/mavs_python')
import mavs_interface as mavs
import mavs_python_paths
mavs_data_path = mavs_python_paths.mavs_data_path
# if you installed MAVS using pip install mavspy, comment out the foure lines above 
# and uncomment the two lines below 
#import mavspy.mavs as mavs
#mavs_data_path = mavs.mavs_data_path

# define the waypoints that the lead vehicle will follow
waypoints = [[-700,-700],[-697.9810181,-698.0328369],[-696.3403931,-696.4155884],[-694.1400146,-694.6057739],[-691.4595337,-692.5979004],[-688.4504395,-690.3410645],[-685.1350098,-687.8634033],[-681.5493774,-685.1914673],[-677.7485352,-682.355835],[-673.7685547,-679.3921509],[-669.6293945,-676.2756958],[-665.3262939,-673.0349121],[-660.8955078,-669.7011719],[-656.355835,-666.2884521],[-651.7030029,-662.8150635],[-646.9696655,-659.3110962],[-642.1929321,-655.7533569],[-637.3556519,-652.1308594],[-632.4771729,-648.4360962],[-627.565918,-644.7202759],[-622.6609497,-641.0125122],[-617.7242432,-637.3432617],[-612.7587891,-633.6854248],[-607.743103,-630.0298462],[-602.6652832,-626.392395],[-597.5606079,-622.7818604],[-592.49646,-619.21698],[-587.5675049,-615.6813965],[-582.6640015,-612.222168],[-577.6991577,-608.7651367],[-572.7014771,-605.3081055],[-567.7324829,-601.8168335],[-562.8287354,-598.2584229],[-557.9828491,-594.6158447],[-553.1331177,-590.9761353],[-548.265686,-587.3521118],[-543.3731079,-583.7475586],[-538.4689941,-580.1394653],[-533.5507813,-576.5100098],[-528.647522,-572.8756714],[-524.1732178,-568.7816772],[-520.1834106,-564.1682739],[-516.2529297,-559.4494629],[-512.2799072,-554.708313],[-508.2506714,-549.9766235],[-504.1989746,-545.2329102],[-500.1366272,-540.4956665],[-496.1052856,-535.7372437],[-492.1047058,-530.9979858],[-488.1100159,-526.2850342],[-484.1018372,-521.6056519],[-480.0823364,-516.9351807],[-476.047821,-512.2946167],[-472.0344543,-507.6858215],[-468.0311584,-502.9996643],[-464.0082397,-498.3044434],[-459.9856262,-493.7143555],[-456.01474,-489.1499939],[-452.099884,-484.523407],[-448.1878357,-479.8421021],[-444.3386536,-475.1629028],[-440.5659485,-470.4889526],[-436.9159851,-465.7235718],[-433.7401428,-460.5629272],[-430.6690063,-455.3210754],[-427.6309204,-450.0535583],[-424.589447,-444.7748718],[-421.3375854,-439.6222534],[-417.7667236,-434.6697083],[-413.8339233,-429.9934082],[-409.3060303,-425.8248291],[-404.6708679,-421.7746582],[-400.0101929,-417.7602234],[-395.3699951,-413.7276001],[-390.7825317,-409.644165],[-386.1704102,-405.5357971],[-381.5057373,-401.3662109],[-376.4746399,-397.6381531],[-371.2097778,-394.2460938],[-365.5758667,-391.9024048],[-359.6035767,-390.7617493],[-353.5162354,-389.8424072],[-347.3303528,-388.9115295],[-341.1789856,-387.5769043],[-335.4047546,-385.1672974],[-329.784668,-382.4879761],[-324.4514771,-379.3753357],[-319.6295471,-375.4710999],[-314.907196,-371.3595276],[-310.1354065,-367.2648621],[-305.3464355,-363.2053833],[-300.6164246,-359.087677],[-296.8608704,-354.1845093],[-294.1160889,-348.6741943],[-291.6741943,-343.078949],[-290.0350342,-337.3964233],[-289.7753906,-331.5732422],[-289.8833923,-325.7409668],[-289.9547424,-319.8775024],[-289.5084839,-313.9822083],[-289.002594,-308.0947876],[-288.6004028,-302.3304138],[-288.1834717,-296.6315308],[-287.8095093,-290.9002686],[-287.4620667,-285.0812683],[-287.1192932,-279.1941223],[-286.8337097,-273.2767029],[-286.5765076,-267.4082642],[-286.3591003,-261.6002808],[-286.1474915,-255.7660522],[-285.9361572,-249.943924],[-285.7252502,-244.1395721],[-285.5291443,-238.2949524],[-285.34552,-232.400116],[-285.17453,-226.5194244],[-285.0402832,-220.6429291],[-284.8977966,-214.6672974],[-284.7509155,-208.5839233],[-284.5866394,-202.435318],[-283.8985291,-196.3622284],[-281.5793762,-190.845871],[-278.3256531,-185.7763214],[-275.1269531,-180.620697],[-273.5114746,-174.9389343],[-273.7423401,-169.2351837],[-275.4432983,-163.8251801],[-278.8675842,-159.3964233],[-283.1472473,-155.728775],[-287.2302551,-151.9904633],[-290.8905029,-147.9438324],[-294.5893555,-143.8240967],[-298.3533936,-139.6955566],[-302.1897278,-135.5240936],[-306.0935364,-131.2927246],[-310.0344543,-127.0213852],[-313.9064026,-122.7806168],[-317.7565918,-118.6320953],[-321.6736145,-114.4661789],[-325.6395569,-110.1683578],[-329.6867676,-105.8672638],[-333.6999512,-101.8499527],[-337.5960693,-98.02114868],[-341.5604858,-94.10185242],[-345.6763611,-90.06950378],[-349.8268738,-86.02682495],[-354.0252075,-81.97942352],[-358.2955322,-77.90447998],[-362.4222107,-73.65510559],[-366.1386414,-68.98423767],[-369.8393555,-64.20284271],[-373.6083069,-59.40552139],[-377.3739319,-54.6268959],[-380.4080811,-49.48183823],[-382.3417969,-43.87374115],[-384.9761353,-38.57613373],[-388.891571,-34.33098221],[-392.8530884,-30.25416183],[-396.9363098,-26.30491447],[-400.9112854,-22.06775284],[-405.0630188,-17.97126198],[-409.301239,-13.99144077],[-413.4355774,-9.975626945],[-418.0134583,-6.763978004],[-423.1260986,-4.674379349],[-428.4088135,-2.991976976],[-433.8240662,-1.791483879],[-438.7404785,-0.882062674],[-443.5590515,-0.113351896],[-447.9017639,1.114452958],[-451.7036438,3.335098267],[-454.0889893,6.814501762],[-454.2311707,10.81219101],[-453.3166504,14.74549294],[-453.2759094,18.44381714],[-453.646759,22.13319206],[-454.120697,26.03481865],[-454.3747253,30.16038322],[-454.3587341,34.56079483]]

# create a PID controller that will control the speed of the following vehicle based on the deviation from the desired follow distance
class PidController:
    def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(0.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral = 0.0
        self.last_error = 0.0
        #self.last_time = time.time()
        self.output_min, self.output_max = output_limits

    def Update(self, measured_value, dt):
        #current_time = time.time()
        #dt = current_time - self.last_time
        if dt <= 0.0:
            dt = 1e-16

        error = self.setpoint - measured_value
        derivative = (error - self.last_error) / dt
        new_integral = self.integral + error * dt
        output = (self.kp * error + self.ki * new_integral + self.kd * derivative)
        clamped_output = max(self.output_min, min(self.output_max, output))

        if clamped_output == output:
            self.integral = new_integral

        self.last_error = error
        #self.last_time = current_time

        return clamped_output

def RunSimulation():
    # Create a MAVS waypoint follower for the lead vehicle
    leader_waypoint_follower = mavs.MavsVehicleController()
    leader_waypoint_follower.SetDesiredPath(waypoints)
    leader_waypoint_follower.SetDesiredSpeed(9.0) # m/s 
    leader_waypoint_follower.SetSteeringScale(2.35)
    leader_waypoint_follower.SetWheelbase(3.8) # meters
    leader_waypoint_follower.SetMaxSteerAngle(0.855) # radians

    # The follow vehicle will just be tracking the lead vehicle based on radar returns
    follower_speed_controller = PidController(0.6, 0.05, 0.05, setpoint=0.0, output_limits=(0.0, 1.0))

    # create a MAVS scene and load it
    scene = mavs.MavsEmbreeScene()
    mavs_scenefile = "/scenes/valley_big.json"
    scene.Load(mavs_data_path+mavs_scenefile)

    # create a MAVS environment and add the scene to it
    env = mavs.MavsEnvironment()
    env.SetScene(scene)

    # set some scene properties for a snowy terrain
    env.SetTime(19) # 0-23
    env.SetFog(35) # 0.0-100.0
    env.SetSnow(10.0) # 0-25
    env.SetTurbidity(2.0) # 2-10
    env.SetAlbedo(0.8) # 0-1
    env.SetWind( [2.5, 1.0] ) # Horizontal windspeed in m/s

    #Create and load the leader and follower vehicles MAVS vehicle
    lead_veh = mavs.MavsRp3d()
    follow_veh = mavs.MavsRp3d()
    veh_file = 'mrzr4_tires_low_gear.json'
    px_init = -700.0
    py_init = -700.0
    heading_init = math.radians(45.0)
    lead_veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
    lead_veh.SetInitialPosition(px_init, py_init, 0.0) # in global ENU
    lead_veh.SetInitialHeading(heading_init) # in radians
    follow_veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
    follow_veh.SetInitialPosition(px_init-15.0*math.cos(heading_init), py_init-15.0*math.sin(heading_init), 0.0) # in global ENU
    follow_veh.SetInitialHeading(heading_init) # in radians
    
    # Create the camera sensor that will chase the follow vehicle
    follow_cam = mavs.MavsCamera()
    follow_cam.Initialize(910,512,0.0062207,0.0035,0.0035)
    follow_cam.SetOffset([-7.0,0.0,2.0],[1.0,0.0,0.0,0.0])
    follow_cam.SetGammaAndGain(0.85,2.0)
    follow_cam.RenderShadows(True)
    follow_cam.SetAntiAliasingFactor(3)

    # create the radar sensor that the follow vehicle will use to chase the lead vehicle
    radar = mavs.MavsRadar()
    radar.SetOffset([2.0,0.0,0.5],[1.0,0.0,0.0,0.0])
    radar.SetFieldOfView(60.0, 0.025)
    radar.SetMaxRange(35.0)

    # set the initial throttle and steering values for the follower
    # and set the desired distance between the vehicles
    follow_throttle = 0.0
    follow_steering = 0.0
    follow_dist = 6.0    

    # set some simulation loop variables
    dt = 1.0/120.0 # time step, seconds
    n = 0 # loop counter
    elapsed_time = 0.0
    
    # Now start the simulation main loop
    while (elapsed_time<95.0):
        # update the waypoint follower for the lead vehicle
        leader_waypoint_follower.SetCurrentState(lead_veh.GetPosition()[0],lead_veh.GetPosition()[1],lead_veh.GetSpeed(),lead_veh.GetHeading())
        dc = leader_waypoint_follower.GetDrivingCommand(dt)

        # Update the lead vehicle with the driving commands from the waypoint follower
        lead_veh.Update(env, dc.throttle, dc.steering, dc.braking, dt)
    
        # update the follow vehicle commands based on the PID controller
        follow_veh.Update(env, follow_throttle, follow_steering, 0.0, dt)

        # update the sensors at 30 Hz
        if n%4==0:
            # update the camera
            follow_cam.SetPose(follow_veh.GetPosition(),follow_veh.GetOrientation())
            follow_cam.Update(env,0.05)
            follow_cam.SaveCameraImage(str(n).zfill(5)+'_camera.bmp')
            follow_cam.Display()

            # update the radar
            radar.SetPose(follow_veh.GetPosition(),follow_veh.GetOrientation())
            radar.Update(env,0.05)
            radar.Display()
            radar.SaveImage(str(n).zfill(5)+"_radar.bmp")
            
            # get the radar targets and find the closest one
            # assume closest target is the lead vehicle
            targets = radar.GetTargets()
            closest = 10000.0
            tg_closest = -1
            for tg in range(len(targets)):
                if targets[tg].range<closest:
                    closest = targets[tg].range
                    tg_closest = tg
            # calculate throttle and steering for the follow vehicle based on the angle and distance of the lead vehicle
            if tg_closest >= 0:
                # PID speed controller tries to maintain the desired following distance
                follow_throttle = follower_speed_controller.Update(-(targets[tg_closest].range-follow_dist), dt) 
                # steering controller is a simple proportional value
                follow_steering = min(max(-1.0, targets[tg_closest].angle/0.35), 1.0)    

        # Update the loop counters
        env.AdvanceTime(dt)
        n = n+1
        elapsed_time = elapsed_time + dt

def MakeVideo():
    # create the output video using the images saved in the simulation loop
    image_folder='./'
    fps=30
    image_files = [image_folder+'/'+img for img in sorted(os.listdir(image_folder)) if img.endswith("_camera.bmp")]
    map_files = [image_folder+'/'+img for img in sorted(os.listdir(image_folder)) if img.endswith("_radar.bmp")]
    for i in range(len(image_files)):
        image1 = Image.open(image_files[i])
        image2 = Image.open(map_files[i])

        # Get the dimensions of the images
        width1, height1 = image1.size
        width2, height2 = image2.size

        # Create a new image with the combined width and the height of the tallest image
        new_height = max(height1, height2)
        new_width = width1 + width2
        new_image = Image.new("RGB", (new_width, new_height))

        # Paste the two images onto the new image
        new_image.paste(image1, (0, 0))
        new_image.paste(image2, (width1, 0))

        # Save the new image
        newname = str(i).zfill(5)+"_merged.bmp"
        new_image.save(newname)
    merged_files = [image_folder+'/'+img for img in sorted(os.listdir(image_folder)) if img.endswith("_merged.bmp")]
    clip = moviepy.video.io.ImageSequenceClip.ImageSequenceClip(merged_files, fps=fps)
    clip.write_videofile('radar_following_video.mp4')
    for filename in os.listdir("./"):
        if filename.lower().endswith("_merged.bmp"):
            os.remove(os.path.join("./", filename))
        if filename.lower().endswith("_radar.bmp"):
            os.remove(os.path.join("./", filename))
        if filename.lower().endswith("_camera.bmp"):
            os.remove(os.path.join("./", filename))
    
if __name__=="__main__":
    RunSimulation()
    MakeVideo()