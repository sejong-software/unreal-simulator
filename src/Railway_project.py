##### Notice #####
# IMU parameter can be set in file named 'ImuSimpleParams.hpp' directlry #
# GPS parameter also set in file named 'GpsSimpleParams.hpp' directly #
##################

import setup_path 
import airsim
import numpy as np
import os
import tempfile
import pprint
import cv2
import time
import math
import sys

class DataAcqusite:
    def __init__(self):
        # Connect
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        
        #print('Drone flying...')
        #self.client.takeoffAsync().join()

        data_init = self.client.getMultirotorState()

        self.pos_init_x = data_init.kinematics_estimated.position.x_val
        self.pos_init_y = data_init.kinematics_estimated.position.y_val
        self.pos_init_z = data_init.kinematics_estimated.position.z_val

    def shape_8(self,pos,t_tot,r,k):

        pos_x = pos[0]
        pos_y = pos[1]
        pos_z = pos[2]

        # pos should be 3x1 in array type
        
        pre_del_x_pos = 0
        pre_del_y_pos = 0
        
        pi = math.pi
        d_theta = 2*pi/t_tot
        dis = 0
        offset = r

        path = []
        path_x_list = []
        path_y_list = []
        t = 0
        
        while t <= t_tot:
            
            theta = d_theta*t
            del_x_pos = r*math.cos(theta)
            del_y_pos = r*math.sin(theta)
            del_z_pos = pos_z
            
            if t != 0:
                del_x = del_x_pos - pre_del_x_pos
                del_y = del_y_pos - pre_del_y_pos
                rel_dis = math.sqrt(del_x**2 + del_y**2)
                dis += rel_dis

            pre_del_x_pos = del_x_pos
            pre_del_y_pos = del_y_pos

            del_x_pos = -del_x_pos+pos_x*(-1)**k
            del_y_pos = -del_y_pos*(-1)**k
            del_x_pos = float(del_x_pos)
            #del_y_pos = del_y_pos.tolist()
            del_z_pos = float(del_z_pos)
            
            path.append(airsim.Vector3r(del_x_pos, del_y_pos, del_z_pos))
            
            t += 1

        print("distance : " + str(dis))
        return path, dis
        
    def tra_zigzag(self, pos, vel, r, t_tot):
        
        for k in range(0,12):
            print("Free flight for " + str(t_tot*12) + "s and " + str(k+1) + " loop")
            path1, dis = self.shape_8(pos, t_tot, r, k)
            vel_circle = dis/(t_tot/2) # Why flying time should be divied to half???

            try:
                result = self.client.moveOnPathAsync(path1,vel_circle,t_tot,airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0), vel+vel/2, 1).join()
            except:
                errorType, value, traceback = sys.exc_info()
                print("moveOnPath threw exception : " + str(value))
                pass

        hei = 20
        time.sleep(50*8)    # Simulation Speed : 0.02
        #time.sleep(8)
        self.client.moveToPositionAsync(-30,0,-hei,3).join()   # Move to start location

        # Generate zigzag path
        path = []
        path.append(airsim.Vector3r(-30,0,-hei))
        path.append(airsim.Vector3r(30,0,-hei))
        path.append(airsim.Vector3r(30,3,-hei))
        path.append(airsim.Vector3r(-30,3,-hei))
        path.append(airsim.Vector3r(-30,6,-hei))
        path.append(airsim.Vector3r(30,6,-hei))
        path.append(airsim.Vector3r(30,9,-hei))
        path.append(airsim.Vector3r(-30,9,-hei))

        # Path parameter
        dis = 249*1.2
        t_tot = dis/vel
        time.sleep(50*8)    # Simulation Speed : 0.02
        # 2 types of Yaw modes : ForwardOnly, MaxDegreeOfFreedom
        
        try:
            result = self.client.moveOnPathAsync(path,vel,t_tot,airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), vel+vel/2, 1).join()
        except:
            errorType, value, traceback = sys.exc_info()
            print("moveOnPath threw exception : " + str(value))
            pass
        
if __name__ == "__main__":  # Discriminate current module or not
    
# Data Recording

    k = 0
    isLanded = 1

    Req_data = DataAcqusite()
    
    time_s = 0

    T_state = Req_data.client.getMultirotorState()
    time_init = T_state.timestamp

    en = 0      # check flying or not
    en1 = 0     # Check implemented or not

    vel = 9
    radius = 20
    mission_time = 15

    #Req_data.client.takeoffAsync().join()
    #Req_data.client.moveToPositionAsync(0,0,-3,3).join()
    
    while en1 == 0:
    
        T_state = Req_data.client.getMultirotorState()
        
        time_s = (T_state.timestamp - time_init)/1000000000

        if time_s > 1:
                Req_data.client.takeoffAsync().join()
                Req_data.client.moveToPositionAsync(0,0,-20,3).join()
                pos = Req_data.client.getMultirotorState().kinematics_estimated.position
                pos_x = pos.x_val
                pos_y = pos.y_val
                pos_z = pos.z_val
                pos = np.array([[pos_x],[pos_y],[pos_z]], dtype=np.float64)
                Req_data.tra_zigzag(pos, vel, radius, mission_time)
                en1 = 1
    
# Stop Recording
    time.sleep(50*10)
    #time.sleep(10)
    print('Drone landing...')
    Req_data.client.landAsync().join()

