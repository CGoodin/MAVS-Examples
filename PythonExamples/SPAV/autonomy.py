"""Classes and functions for a simple autonomous driver

Algorithms for processing lidar data into an occupancy grid.
For demonstration purposes with MAVS - not intended to be a 
fully featured autonomy solution.
"""
import numpy as np
import math

class Vector3(object):
    def __init__(self,vals=[0.0, 0.0, 0.0]):
        self.x = vals[0]
        self.y = vals[1]
        self.z = vals[2]
    def __str__(self):
        return '['+str(self.x)+', '+str(self.y)+', '+str(self.z)+']'
    def __add__(self,u):
        v = Vector3()
        v.x = self.x + u.x
        v.y = self.y + u.y
        v.z = self.z + u.z
        return v
    def __sub__(self,u):
        v = Vector3()
        v.x = self.x - u.x
        v.y = self.y - u.y
        v.z = self.z - u.z
        return v
    def scale(self,s):
        v = Vector3()
        v.x = s*self.x
        v.y = s*self.y
        v.z = s*self.z
        return v
    def __radd__(self,u):
       return self.__add__(u)
    def __rsub__(self,u):
        v = Vector3()
        v.x = u.x - self.x
        v.y = u.y - self.y
        v.z = u.z - self.z
        return v
    def __truediv__(self,s):
        v = Vector3()
        v.x = self.x/s
        v.y = self.y/s
        v.z = self.z/s
        return v
    def normalize(self):
        m = math.sqrt(self.x*self.x + self.y*self.y + self.z*self.z)
        self.x = self.x/m
        self.y = self.y/m
        self.z = self.z/m
    def magnitude(self):
        m = math.sqrt(self.x*self.x + self.y*self.y + self.z*self.z)
        return m
    def dot(self,b):
        return self.x*b.x + self.y*b.y + self.z*b.z
    def cross(self,b):
        v = Vector3()
        v.x = self.y*b.z - self.z*b.y
        v.y = self.z*b.x - self.x*b.z
        v.z = self.x*b.y - self.y*b.x
        return v

class Quaternion(object):
    def __init__(self,vals=[1.0, 0.0, 0.0, 0.0]):
        self.w = vals[0]
        self.x = vals[1]
        self.y = vals[2]
        self.z = vals[3]
    def __str__(self):
        return '['+str(self.w)+', '+str(self.x)+', '+str(self.y)+', '+str(self.z)+']'
    def normalize(self):
        m = math.sqrt(self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z)
        self.w = self.w/m
        self.x = self.x/m
        self.y = self.y/m
        self.z = self.z/m
    def conjugate(self):
        q = Quaternion()
        q.w = self.w
        q.x = -self.x
        q.y = -self.y
        q.z = -self.z
        return q
    def __mul__(self,q):
        w1 = self.w
        x1 = self.x
        y1 = self.y
        z1 = self.z
        w2 = q.w
        x2 = q.x
        y2 = q.y
        z2 = q.z
        qout = Quaternion()
        qout.w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        qout.x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        qout.y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        qout.z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return qout
    def __rmul__(self,q):
        w2 = self.w
        x2 = self.x
        y2 = self.y
        z2 = self.z
        w1 = q.w
        x1 = q.x
        y1 = q.y
        z1 = q.z
        qout = Quaternion()
        qout.w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        qout.x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        qout.y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        qout.z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return qout
    def rotate(self,u):
        uq = Quaternion()
        uq.w = 0.0
        uq.x = u.x
        uq.y = u.y
        uq.z = u.z
        q1 = Quaternion
        q1 = self*uq*self.conjugate()
        v = Vector3()
        v.x = q1.x
        v.y = q1.y
        v.z = q1.z
        return v

class Header(object):
    def __init__(self):
        self.seq = 0
        self.stamp = 0
        self.frame_id = 'default'

class Pose(object):
    def __init__(self):
        self.position = Vector3()
        self.orientation = Quaternion()

class Twist(object):
    def __init(self):
        self.linear = Vector3()
        self.angular = Vector3()

class MapMetaData(object):
    def __init__(self):
        self.map_load_time = 0
        self.resolution = 1.0
        self.width = 100
        self.height = 100
        self.origin = Vector3()

class OccupancyGrid(object):
    def __init__(self,dimensions=None):
        self.header = Header()
        self.info = MapMetaData()
        self.data = np.zeros(shape=(1,1))
        self.height_thresh = 1.0
        if dimensions:
            if len(dimensions)==2:
                self.resize(dimensions[0],dimensions[1])
    def resize(self,width,height):
        self.data = np.zeros(shape=(width,height))
        self.info.width = width
        self.info.height = height
    def coordinate_to_index(self,x,y):
        i = math.floor((x-self.info.origin.x)/self.info.resolution)
        j = math.floor((y-self.info.origin.y)/self.info.resolution)
        return [i,j]
    def index_to_coordinate(self,ix,iy):
        x = ix*self.info.resolution + self.info.origin.x
        y = iy*self.info.resolution + self.info.origin.y
        return [x,y]
    def index_path_to_coordinates(self,path):
        path_enu = []
        for p in path:
            point_enu = self.index_to_coordinate(p[0],p[1])
            path_enu.append(point_enu)
        return path_enu
    def set_origin(self,x,y):
        self.info.origin.x = x
        self.info.origin.y = y
    def add_registered_points(self,points):
        for p in points:
            if p[2]>self.height_thresh:
                idx = self.coordinate_to_index(p[0],p[1])
                if idx[0]>=0 and idx[0]<self.info.width and idx[1]>=0 and idx[1]<self.info.height:
                    self.data[idx[0],idx[1]] = 1.0
    def add_points(self,pos,quat,points):
        q = Quaternion([quat[0],quat[1],quat[2],quat[3]])
        position = Vector3([pos[0],pos[1],pos[2]])
        for p in points:
            v = Vector3([p[0],p[1],p[2]])
            vprime = q.rotate(v)
            vprime = vprime + position
            if vprime.z>self.height_thresh:
                idx = self.coordinate_to_index(vprime.x,vprime.y)
                if idx[0]>=0 and idx[0]<self.info.width and idx[1]>=0 and idx[1]<self.info.height:
                    self.data[idx[0],idx[1]] = 1.0

def Odometry(object):
    def __init__(self):
        self.header = Header()
        self.child_frame_id = 'default'
        self.pose = Pose()
        self.twist = Twist()

def DrivingCommand(object):
    def __init__(self):
        self.throttle = 0.0
        self.steering = 0.0
        self.braking = 0.0
