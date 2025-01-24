# MASLAB 2025
# Object Detection
# 
# Developed by Joseph Hobbs
# This code is open-source
#   under the MIT License

import cv2
import numpy as np
from math import pi
from math import tan

from os import system
import multiprocessing
import time
from icm42688 import ICM42688
import board, busio
import board
import busio
import adafruit_vl53l0x

from raven import Raven

i2c = busio.I2C(board.SCL, board.SDA)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
#timeofflight = adafruit_vl53l0x.VL53L0X(i2c)
timeofflight = None


while not spi.try_lock():
    pass

spi.configure(baudrate=5000000)

imu = ICM42688(spi)
imu.begin()

vdev = 0

# Capture video from webcam
capture = cv2.VideoCapture("/dev/video" + str(vdev))

kp_history = [[]] * 8

lowpass_pos = False

fov = 55 #field of view, degrees

arm_rpc = 1/260 #rotations per click of arm encoder
arm_ratio = 1/16.25
clicksToDegrees = arm_rpc * arm_ratio * (1/360)

arm_motor = Raven.MotorChannel.CH3

ravenbrd = Raven()
ravenbrd.set_motor_encoder(arm_motor, 0) # Reset encoder
ravenbrd.set_motor_mode(arm_motor, Raven.MotorMode.POSITION) # Set motor mode to POSITION
ravenbrd.set_motor_pid(arm_motor, p_gain = 100, i_gain = 0, d_gain = 0) # Set PID values

angleP = 1.3
angleI = 6
angleD = -0.004
angleFeedforward = 12

def getArmAngle():
    armClicks = ravenbrd.get_motor_encoder(arm_motor)
    armAngle = armClicks # * clicksToDegrees
    print(armAngle)
    return armAngle

def setArmAngle(angle_setpoint):
    clicks_setpoint = angle_setpoint * (1/clicksToDegrees)
    ravenbrd.set_motor_target(Raven.MotorChannel.CH1, clicks_setpoint)
    return





if __name__ == "__main__":
    time.sleep(1)


    while True:
        getArmAngle()
       
        """
        good_keypoints = sorted(good_keypoints, key=lambda k: k.size)
        if good_keypoints:
            redcube = good_keypoints[-1]
            angle = (redcube.pt[0] - 320) * (55 / 640)
            print("Red cube angle:", angle)
        """

        # Wait for the user to press Q
        #k = cv2.waitKey(1) & 0xFF
        #if k == ord('q'):
        #    # Quit the program
        #    break
        #if k == ord('l'):
        #    lowpass_pos = not lowpass_pos
