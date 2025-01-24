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
dryrun = False


# Capture video from webcam
capture = cv2.VideoCapture("/dev/video" + str(vdev))

kp_history = [[]] * 8

lowpass_pos = False

fov = 55 #field of view, degrees

arm_rpc = 1/(64*270) #rotations per click of arm encoder
arm_ratio = 1
clicksToDegrees = arm_rpc * arm_ratio * 360 
degreesToClicks = 1 / clicksToDegrees


left_drive = Raven.MotorChannel.CH1
right_drive = Raven.MotorChannel.CH2
arm_motor = Raven.MotorChannel.CH3
servo_bottom = Raven.ServoChannel.CH1
servo_top = Raven.ServoChannel.CH2

ravenbrd = Raven()
ravenbrd.set_motor_encoder(arm_motor, 0) # Reset encoder
ravenbrd.set_motor_mode(arm_motor, Raven.MotorMode.POSITION) # Set motor mode to POSITION
ravenbrd.set_motor_pid(arm_motor, p_gain = 100, i_gain = 0, d_gain = 0) # Set PID values


if not dryrun:
    print(ravenbrd.set_motor_mode(left_drive, Raven.MotorMode.DIRECT))
    print(ravenbrd.set_motor_mode(right_drive, Raven.MotorMode.DIRECT))

redangle_mp = multiprocessing.Value('d', 0)
redangle_new = multiprocessing.Value('i', 0)
ty = multiprocessing.Value('d', 0)
stack_type = multiprocessing.Value('i', 1)
# 0: whatever
# 1: red cube alone
# 2: red on top
# 3: green on top
redangle = 0
redint = 0
rederiv = 0
lastred = 0


angleP = 1.3
angleI = 6
angleD = -0.004
angleFeedforward = 12
intmax = 3.5
killtimer = 0

factor = 12

t1 = 0

h1 = .186 #height of camera
h2 = .0254 #height of block center
a1 = -15.0 #angle of camera


angleMode = True
amtimer = 0

donehere = 0

switchcl = True

def grab_cube(closeit=True):
    if switchcl: closeit = not closeit
    ravenbrd.set_servo_position(servo_bottom, -90 if closeit else 90)
    time.sleep(0.8)
    ravenbrd.set_servo_position(servo_bottom, 0)


def ipthread(redangle_mp):
    while True:
        imageproc(redangle_mp)

def getArmAngle():
    armClicks = ravenbrd.get_motor_encoder(arm_motor)
    armAngle = armClicks * clicksToDegrees
    return armAngle

def setArmAngle(angle_setpoint):
    clicks_setpoint = angle_setpoint * (1/clicksToDegrees)
    ravenbrd.set_motor_target(Raven.MotorChannel.CH1, clicks_setpoint)
    return


def imageproc(redangle_mp):
    _, bgr_image = capture.read()
    #t1 = time.time()
    #bgr_image = cv2.copyMakeBorder(bgr_image, 16, 16, 16, 16, cv2.BORDER_CONSTANT, (0, 0, 0))
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)


    #blurpreview = cv2.GaussianBlur(bgr_image, (11,11), 0)

    goodMask = cv2.inRange(
        hsv_image,
        np.array([35, 10, 0]),
        np.array([95, 255, 240])
    )


    #badMask = cv2.inRange(
    #    hsv_image,
    #    np.array([30, 0, 0]),
    #    np.array([100, 255, 255])
    #)
    badMask = cv2.inRange(
        hsv_image,
        np.array([11, 0, 0]),
        np.array([160, 255, 255])
    )

    badMask = cv2.bitwise_not(badMask)

    badMask2 = cv2.inRange(hsv_image,
                            np.array([0, 100, 80]), np.array([179, 255, 255]))

    badMask = cv2.bitwise_and(badMask, badMask2)

    #bozoMask = cv2.inRange(hsv_image,
    #                       np.array([0, 20, 150]),np.array([40, 120, 255]))



    #badMask = cv2.GaussianBlur(badMask, (5,5), 0)

    #whiteMask = cv2.inRange(
    #    hsv_image,
    #    np.array([0, 0, 235]),
    #    np.array([180, 40, 255])
    #)

    #itsFuckingGreenNow(goodMask, goodMask)

    #mask = cv2.bitwise_or(goodMask, whiteMask)

    greenMask = goodMask
    redMask = badMask

    #goodPixels = []
    #for index, value in np.ndenumerate(goodMask):
    #    if value != 0: goodPixels.append(index)
    #print(len(goodPixels))
    """
    params = cv2.SimpleBlobDetector_Params()

    params.minThreshold = 70
    params.maxThreshold = 255

    params.filterByArea = True
    params.minArea = 40
    params.maxArea = 99999
    params.filterByCircularity = False
    params.filterByConvexity = True
    params.minConvexity = 0.2
    params.filterByInertia = False
    """

    #detector = cv2.SimpleBlobDetector_create(params)

    #masked = cv2.bitwise_and(bgr_image, bgr_image, mask=cv2.bitwise_or(redMask, greenMask))
    #masked = cv2.bitwise_and(bgr_image, bgr_image, mask=bozoMask)
    #scaled = cv2.multiply(bgr_image, 1/4) + cv2.multiply(masked, 3/4)

    """ keypoints = detector.detect(cv2.bitwise_not(mask))

    kp_history = kp_history[1:]
    kp_history.append(keypoints)



    good_keypoints = []
    for kp in keypoints:
        allgood = True
        pos_history = []
        for hist in kp_history:
            good = False
            for oldkp in hist:
                if (kp.pt[0] - oldkp.pt[0])**2 + (kp.pt[1] - oldkp.pt[1])**2 <= kp.size **2:
                    good = True
                    pos_history.append(oldkp.pt)
                    break
            if not good:
                allgood = False
                break
        if allgood:
            if lowpass_pos:
                newpos = [0, 0]
                for p in pos_history:
                    newpos[0] += p[0] / len(pos_history)
                    newpos[1] += p[1] / len(pos_history)
                kp.pt = newpos
            good_keypoints.append(kp)

    with_keypoints = scaled
    """
    green_contours, _ = cv2.findContours(greenMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_contours, _ = cv2.findContours(redMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = green_contours + red_contours
    c_colors = ["g"] * len(green_contours) + ["r"] * len(red_contours)
    good_red = []
    good_green = []
    for i in range(len(contours)):
        c = contours[i]
        col = c_colors[i]
        if cv2.contourArea(c) < 200: continue
        # 4 pi^2 r^2 / pi r ^2 -> 4 pi
        inverse_circularity = (cv2.arcLength(c,True)**2) / (4 * cv2.contourArea(c) * pi)
        if inverse_circularity > 4: continue
        M = cv2.moments(c)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        #scaled = cv2.circle(scaled, (cx,cy), 5, (255, 0, 255), 2)
        if col == "r":
            good_red.append([cv2.convexHull(c),cx,cy])
        else:
            good_green.append([cv2.convexHull(c),cx,cy])
    if good_red and good_green:
        ty.value = max(good_red[0][2], good_green[0][2])
    elif good_green:
        ty.value = good_green[0][2]
    elif good_red:
        ty.value = good_red[0][2]



    #circ_contours, _ = cv2.findContours(bozoMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #good_circs = []
    #for c in circ_contours:
    #    if cv2.contourArea(c) < 150: continue
    #    inverse_circularity = (cv2.arcLength(c,True)**2) / (4 * cv2.contourArea(c) * pi)
    #    if inverse_circularity > 2.2: continue
    #    good_circs.append(c)
    #    M = cv2.moments(c)
    #    cx = int(M['m10']/M['m00'])
    #    cy = int(M['m01']/M['m00'])
    #    scaled = cv2.circle(scaled, (cx,cy), 5, (255, 0, 255), 2)


    good_red = sorted(good_red, key = lambda a: -cv2.contourArea(a[0]))
    good_green = sorted(good_green, key = lambda a: -cv2.contourArea(a[0]))
    if good_red:
        p = good_red[0][1:]
        angle = (p[0] - 320) * (55 / 640)
        redangle = angle
        redangle_mp.value = redangle + 2
        redangle_new.value = 1
        for g in good_green:
            if abs(g[1] - p[0]) < 50:
                if g[2] < p[1]:
                    stack_type.value = 3
                else:
                    stack_type.value = 2
        #killtimer = 0
    else:
        redangle_new.value = 2
        pass#killtimer += 1
    #stack_type.value = 1


def bt(x):
    if x < -100: return -100
    if x > 100: return 100
    return x


def get_distance():
    m_ty = ty.value
    angleRad = pi * (m_ty + a1) / 180.0
    distanceFromHub = (h2 - h1) / tan(angleRad)
    #print(distanceFromHub)
    return distanceFromHub

tofcycle = 0

cut1 = 5
cut2 = 10

didgrab = False

seek_cube = True
seek_target = 0
seekP = 2
seekI = 0#6
seekD = 0#-0.005
seekFeedforward = 24
seekint = 0
lastseek = 0

seek_current = 0
seek_cycle = 0



if __name__ == "__main__":
    system("v4l2-ctl --device /dev/video" + str(vdev) + " -c auto_exposure=1")
    system("v4l2-ctl --device /dev/video" + str(vdev) + " -c exposure_time_absolute=500") # 500
    time.sleep(1)
    p1 = multiprocessing.Process(target=ipthread, args=(redangle_mp,))
    p1.start()
    grab_cube(False)
    while True:
        # Read a frame
        dist = get_distance()
        if redangle_new.value:
            if redangle_new.value == 2 or stack_type.value == 1:
                seek_cube = True
            else:
                seek_cube = False
            redangle_new.value = 0
            redangle = redangle_mp.value

        #if good_green: scaled = cv2.drawContours(scaled, [g[0] for g in good_green], -1, (0, 255, 0), 3)
        #if good_red: scaled = cv2.drawContours(scaled, [g[0] for g in good_red], -1, (0, 0, 255), 3)
        #scaled = cv2.drawContours(scaled, good_circs, -1, (0, 255, 255), 3)
        #with_keypoints = cv2.drawKeypoints(scaled, good_keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # Display the frame in the video feed
        # NOTE: `cv2.imshow` takes images in BGR
        
        #cv2.putText(scaled, str(len(good_red) + len(good_green)) + " contour(s)", (30, 30), cv2.FONT_HERSHEY_SIMPLEX,
        #        1,
        #        (0, 0, 255),
        #        2)




        #cv2.imshow("Video Feed", blurpreview)

        t1 = time.time()
        if donehere >= 6:
            ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH1, 50)
            ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH2, 50)
            ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH1, 0) #not sure why need rev
            ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH2, 0)
            if not didgrab:
                grab_cube(True)
                time.sleep(0.12)
                grab_cube(False)
                ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH1, 50)
                ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH2, 50)
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH1,50,reverse=True) #not sure why need rev
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH2,50)
                time.sleep(0.12)
                ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH1, 50)
                ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH2, 50)
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH1, 0) #not sure why need rev
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH2, 0)
                grab_cube(True)
                ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH1, 50)
                ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH2, 50)
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH1,100) #not sure why need rev
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH2,100)
                time.sleep(0.25)
                ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH1, 50)
                ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH2, 50)
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH1,100,reverse=True) #not sure why need rev
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH2,100,reverse=True)
                time.sleep(0.25)
                ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH1, 50)
                ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH2, 50)
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH1, 0) #not sure why need rev
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH2, 0)
                grab_cube(False)
                didgrab = True
                print("i grabbed: ", ["????", "RED", "REDONTOP", "GREENONTOP"][stack_type.value])
                stack_type.value = 1
                time.sleep(0.5)
                ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH1, 50)
                ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH2, 50)
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH2,75,reverse=True) #not sure why need rev
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH1,75)
                time.sleep(0.2)
                ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH1, 50)
                ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH2, 50)
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH2,0,reverse=True) #not sure why need rev
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH1,0)
                time.sleep(3)
                didgrab = False
                angleMode = True
                donehere = 0
        elif angleMode or seek_cube:
            redint += redangle*0.001
            if redint > intmax: redint = intmax
            if redint < -intmax: redint = -intmax
            rederiv = (lastred - redangle)/0.001 
            lastred = redangle

            
            turn = redangle * angleP + redint * angleI + rederiv * angleD
            turn = -turn
            if abs(turn) < 0.5: pass
            elif turn > 0: turn += angleFeedforward
            elif turn < 0: turn -= angleFeedforward
            
            if abs(redangle) < cut1:
                amtimer += 1
            else:
                amtimer = 0
            if amtimer > 12:
                angleMode = False
                turn = 0
                amtimer = 50
            if seek_cube:
                #turn = angleFeedforward*2.5
                serr = seek_target - seek_current
                seekint += serr*0.001
                if seekint > intmax: seekint = intmax
                if seekint < -intmax: seekint = -intmax
                turn = seekP * serr + seekD * ((lastseek - serr)/0.001) + seekI * seekint
                if abs(turn) < 0.5: pass
                elif turn > 0: turn += seekFeedforward
                elif turn < 0: turn -= seekFeedforward
                lastseek = serr
                print("seek_cube", seek_current, seek_target, turn)
            else:
                print(redangle, -turn, ["????", "RED", "REDONTOP", "GREENONTOP"][stack_type.value])
            if turn > 100: turn = 100
            if turn < -100: turn = -100
            #if redangle < -4: turn = 25
            #elif redangle > 4: turn = -25
            #else: turn = 0
            if killtimer < 5:
                if not dryrun:
                    ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH1, 50)
                    ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH2, 50)
                    lturn = -turn# - 15
                    rturn = turn# - 15
                    ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH1, abs(bt(lturn)), reverse=lturn>0) #not sure why need rev
                    ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH2, abs(bt(rturn)), reverse=rturn<0)
                #else:
                #    print(turn)
        else:
            ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH1, 50)
            ravenbrd.set_motor_torque_factor(Raven.MotorChannel.CH2, 50)
            lturn = 26
            rturn = 26
            if not seek_cube:
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH1, abs(bt(lturn)), reverse=lturn>0) #not sure why need rev
                ravenbrd.set_motor_speed_factor(Raven.MotorChannel.CH2, abs(bt(rturn)), reverse=rturn<0)
            amtimer -=1
            #if amtimer == 0:
            print(redangle)
            if abs(redangle) > cut2:
                angleMode = True
                redint = 0
        tofcycle += 1
        if donehere < 6 and tofcycle % 10 == 0:
            if timeofflight is not None:
                t = timeofflight.range
                if t < 80:
                    donehere += 1
                else:
                    donehere = 0
        #print((time.time() - t1)*1000)
        zvel = imu.get_data()[1][2]
        redangle += factor * zvel * (180 / pi) * 0.001
        seek_current += zvel * (180 / pi) * 0.001
        seek_cycle += 1
        if seek_cycle % 600 == 0:
            seek_target = seek_current + 18
        time.sleep(0.001)
        if dryrun:
            pass#cv2.imshow("ITS FUCKKING RED NOW", scaled)
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
