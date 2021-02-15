# This code should be implemented after start to command
import setup_path
import airsim
import numpy as np
import os
import pprint
import cv2
import time
import math
import sys

# I_hz : IMU update rate
# G_hz : GPS update rate
# C_hz : Camera image update rate
# if문 대신 다른 방법으로 데이터를 추출하는 방법 탐색 필요

class DataRequest:
    def __init__(self, I_hz, G_hz, C_hz):
        self.client = airsim.MultirotorClient()
        self.True_hz = I_hz
        self.IMU_hz = I_hz
        self.GPS_hz = G_hz
        self.IMG_hz = C_hz

    def TrueRequest(self,t,hz):

        True_pos = self.client.getMultirotorState()

        True_x = True_pos.kinematics_estimated.position.x_val
        True_y = True_pos.kinematics_estimated.position.y_val
        True_z = True_pos.kinematics_estimated.position.z_val

        True_x_acc = True_pos.kinematics_estimated.linear_acceleration.x_val
        True_y_acc = True_pos.kinematics_estimated.linear_acceleration.y_val
        True_z_acc = True_pos.kinematics_estimated.linear_acceleration.z_val

        True_x_gyr = True_pos.kinematics_estimated.angular_velocity.x_val
        True_y_gyr = True_pos.kinematics_estimated.angular_velocity.y_val
        True_z_gyr = True_pos.kinematics_estimated.angular_velocity.z_val

        True_data = open('True.txt','a')
        True_message = format(t,".2f") + "\t" + str(True_x) + "\t" + str(True_y) + "\t" + str(True_z) + "\t" + str(t) + "\t" + str(True_x_gyr) + "\t" + str(True_y_gyr) + "\t" + str(True_z_gyr) + "\t" + str(True_x_acc) + "\t" + str(True_y_acc) + "\t" + str(True_z_acc) + "\n"
        print(True_message, file=True_data)

    def IMURequest(self,t,hz):
        IMU_state = self.client.getImuData()

        IMU_acc = IMU_state.linear_acceleration
        IMU_gyr = IMU_state.angular_velocity

        IMU_acc_x = IMU_acc.x_val
        IMU_acc_y = IMU_acc.y_val
        IMU_acc_z = IMU_acc.z_val

        IMU_gyr_x = IMU_gyr.x_val
        IMU_gyr_y = IMU_gyr.y_val
        IMU_gyr_z = IMU_gyr.z_val

        IMU_data = open('IMU.txt', 'a')
        IMU_message = format(t,".2f") + "\t" + str(IMU_gyr_x) + "\t" + str(IMU_gyr_y) + "\t" + str(IMU_gyr_z) + "\t" + str(IMU_acc_x) + "\t" + str(IMU_acc_y) + "\t" + str(IMU_acc_z) + "\n"
        print(IMU_message, file = IMU_data)

    def GPSRequest(self,t,hz):
        GPS_state = self.client.getGpsData()

        GPS_pos = GPS_state.gnss.geo_point

        GPS_Lon = GPS_state.gnss.geo_point.longitude
        GPS_Lat = GPS_state.gnss.geo_point.latitude
        GPS_Alt = GPS_state.gnss.geo_point.altitude

        GPS_data = open('GPS.txt', 'a')
        GPS_message = format(t,".2f") + "\t" + str(GPS_Lat) + "\t" + str(GPS_Lon) + "\t" + str(GPS_Alt) + "\n"
        print(GPS_message, file=GPS_data)

    def IMGRequest(self,t,hz):
        responses = self.client.simGetImages([airsim.ImageRequest("3", airsim.ImageType.Scene)])
        for idx, response in enumerate(responses):
            airsim.write_file(os.path.normpath("C:/Users/YeoungMin Kim/Desktop/2021전반기/Simulation_data/20210130_20m/" + format(t,".2f") + '.png'), response.image_data_uint8)

if __name__ == "__main__":
    
    Sen_dat = DataRequest(100, 5, 10)
    IMU_hz = Sen_dat.IMU_hz
    GPS_hz = Sen_dat.GPS_hz
    IMG_hz = Sen_dat.IMG_hz

    t_max = 350
    
    t_init = Sen_dat.client.getMultirotorState().timestamp

    t_s = (Sen_dat.client.getMultirotorState().timestamp - t_init)/1000000000
    pre_t_s = -1
    pre_t_s_gps = -1
    pre_t_s_img = -1
    
    print("Data recording...")
    while t_s <= t_max:

        if int(t_s/(1/IMU_hz)) != pre_t_s:
            Sen_dat.TrueRequest(t_s, IMU_hz)
            Sen_dat.IMURequest(t_s, IMU_hz)
            if int(t_s/(1/GPS_hz)) != pre_t_s_gps:
                Sen_dat.GPSRequest(t_s, GPS_hz)
            if int(t_s/(1/IMG_hz)) != pre_t_s_img:
                Sen_dat.IMGRequest(t_s, IMG_hz)

        pre_t_s = int(t_s/(1/IMU_hz))
        pre_t_s_gps = int(t_s/(1/GPS_hz))
        pre_t_s_img = int(t_s/(1/IMG_hz))
        
        t_s = (Sen_dat.client.getMultirotorState().timestamp - t_init)/1000000000

    print("Record end")    
