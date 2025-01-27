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
timeofflight = adafruit_vl53l0x.VL53L0X(i2c)
#timeofflight = None
timeofflight.measurement_timing_budget = 20000


while not spi.try_lock():
    pass

spi.configure(baudrate=5000000)

imu = ICM42688(spi)
imu.begin()


vdev = 0


# Capture video from webcam
capture = cv2.VideoCapture("/dev/video" + str(vdev))

kp_history = [[]] * 8


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
wrist_servo = Raven.ServoChannel.CH3

ravenbrd = Raven()
ravenbrd.set_motor_encoder(arm_motor, 0) # Reset encoder
ravenbrd.set_motor_mode(arm_motor, Raven.MotorMode.POSITION) # Set motor mode to POSITION
ravenbrd.set_motor_pid(arm_motor, p_gain = 70, i_gain = 1e-4, d_gain = 6) # Set PID values


print(ravenbrd.set_motor_mode(left_drive, Raven.MotorMode.DIRECT))
print(ravenbrd.set_motor_mode(right_drive, Raven.MotorMode.DIRECT))

redangle_mp = multiprocessing.Value('d', 0)
redangle_new = multiprocessing.Value('i', 0)
ty = multiprocessing.Value('d', 0)
stack_type = multiprocessing.Value('i', 1)
REDONLY = 1
REDONTOP = 2
GREENONTOP = 3
cube_vfrac = multiprocessing.Value('d', 0)
# 0: whatever
# 1: red cube alone
# 2: red on top
# 3: green on top
redangle = 0
redint = 0
rederiv = 0
lastred = 0


angleP = 1.6
angleI = 9
angleD = -0.004
angleFeedforward = 38
intmax = 4.5
killtimer = 0

factor = 12

t1 = 0

h1 = .186 #height of camera
h2 = .0254 #height of block center
a1 = -15.0 #angle of camera


switchcl = True


armlo = 5
armmid = -40
armhi = -195

tofcycle = 0

angleToMove = 5
moveToAngle = 8


seek_target = 0
seekP = .2
seekI = 0#6
seekD = -.003
seekFeedforward = 30
seekint = 0
lastseek = 0

seek_current = 0


def gimmegimmegimme(closeit=True, t=0.8):
    #if switchcl: closeit = not closeit
    #ravenbrd.set_servo_position(servo_bottom, -90 if closeit else 90)
    #time.sleep(0.8)
    #ravenbrd.set_servo_position(servo_bottom, 0)
    ravenbrd.set_servo_position(servo_bottom, -90 if closeit else 90)
    ravenbrd.set_servo_position(servo_top, -90 if closeit else 90)
    time.sleep(t)

def release_cube(top=True):
    if top:
        ravenbrd.set_servo_position(servo_top, 90)
    else:
        ravenbrd.set_servo_position(servo_bottom, 90)

def wrist(angle):
    ravenbrd.set_servo_position(wrist_servo, angle, min_us=500, max_us=2500)

def relieve():
    ravenbrd.set_motor_mode(arm_motor, Raven.MotorMode.DIRECT)
    ravenbrd.set_motor_speed_factor(arm_motor, 100,reverse=True)
    ravenbrd.set_motor_torque_factor(arm_motor, 100)
    time.sleep(0.1)
    ravenbrd.set_motor_speed_factor(arm_motor, 0,reverse=True)
    ravenbrd.set_motor_torque_factor(arm_motor, 0)
    time.sleep(0.5)

def ipthread(redangle_mp):
    while True:
        imageproc(redangle_mp)

def getArmAngle():
    armClicks = ravenbrd.get_motor_encoder(arm_motor)
    armAngle = armClicks * clicksToDegrees
    return armAngle

def setArmAngle(angle_setpoint):
    clicks_setpoint = angle_setpoint * (1/clicksToDegrees)
    ravenbrd.set_motor_target(arm_motor, clicks_setpoint)
    return

def arm_goto(tgt):
    ravenbrd.set_motor_mode(arm_motor, Raven.MotorMode.POSITION) # Set motor mode to POSITION
    while abs(getArmAngle()-tgt) > 0.5:
        setArmAngle(tgt)
        ravenbrd.set_motor_torque_factor(arm_motor, 100)


def imageproc(redangle_mp):
    _, bgr_image = capture.read()
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

    goodMask = cv2.inRange(
        hsv_image,
        np.array([35, 10, 0]),
        np.array([95, 255, 240])
    )

    badMask = cv2.inRange(
        hsv_image,
        np.array([11, 0, 0]),
        np.array([160, 255, 255])
    )

    badMask = cv2.bitwise_not(badMask)

    badMask2 = cv2.inRange(hsv_image,
                            np.array([0, 100, 80]), np.array([179, 255, 255]))

    badMask = cv2.bitwise_and(badMask, badMask2)
    greenMask = goodMask
    redMask = badMask
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

    good_red = sorted(good_red, key = lambda a: -cv2.contourArea(a[0]))
    good_green = sorted(good_green, key = lambda a: -cv2.contourArea(a[0]))
    if good_red:
        p = good_red[0][1:]
        angle = (p[0] - 320) * (55 / 640)
        cube_vfrac.value = p[1] / 480
        redangle = angle
        redangle_mp.value = redangle + 2
        redangle_new.value = 1
        stack_type.value = REDONLY
        for g in good_green:
            if abs(g[1] - p[0]) < 50:
                if g[2] < p[1]:
                    stack_type.value = GREENONTOP
                else:
                    stack_type.value = REDONTOP
    else:
        redangle_new.value = 2


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


SEEK_CUBE = 0
ANGLE_FOLLOW = 1
MOVE_FOLLOW = 2
FINAL_APPROACH = 3
GRAB_STACK = 4
WALL_LOCATE = 5
WALL_APPROACH = 6
WALL_BACKUP = 7
DUMP_CUBES = 8
RELEASE_GREENS = 9
VICTORY = 10

state_names = ["SEEK_CUBE", "ANGLE_FOLLOW", "MOVE_FOLLOW", "FINAL_APPROACH", "GRAB_STACK", "WALL_LOCATE", "WALL_APPROACH", "WALL_BACKUP", "DUMP_CUBES", "RELEASE_GREENS", "VICTORY"]


cur_state = SEEK_CUBE
state_timer = -400
trans_timer = 0

seen_for = 0
notseen_for = 0

report = ""
dorept = False
rept_cycle = 0

disable_wheels = False
actual_grab = True

cumulative_stack = REDONLY

greenchute = 90
redchute = -90

def transition(tgt):
    global report, cur_state, state_timer, trans_timer, dorept
    report += "\t==> "+state_names[tgt]
    cur_state = tgt
    state_timer = 0
    trans_timer = 0
    dorept = True

def wheelturn(turn):
    if disable_wheels: turn = 0
    ravenbrd.set_motor_torque_factor(left_drive, 85)
    ravenbrd.set_motor_torque_factor(right_drive, 85)
    lturn = -turn# - 15
    rturn = turn# - 15
    ravenbrd.set_motor_speed_factor(left_drive, abs(bt(lturn)), reverse=lturn>0)
    ravenbrd.set_motor_speed_factor(right_drive, abs(bt(rturn)), reverse=rturn<0)

def wheelfwd(fwd):
    if disable_wheels: fwd = 0
    ravenbrd.set_motor_torque_factor(left_drive, 50)
    ravenbrd.set_motor_torque_factor(right_drive, 50)
    lturn = fwd
    rturn = fwd
    ravenbrd.set_motor_speed_factor(left_drive, abs(bt(lturn)), reverse=lturn>0)
    ravenbrd.set_motor_speed_factor(right_drive, abs(bt(rturn)), reverse=rturn<0)

def f3d(x):
    return '%.3f' % x

if __name__ == "__main__":
    system("v4l2-ctl --device /dev/video" + str(vdev) + " -c auto_exposure=1")
    system("v4l2-ctl --device /dev/video" + str(vdev) + " -c exposure_time_absolute=800") # 500
    time.sleep(0.5)
    n = 0
    p1 = multiprocessing.Process(target=ipthread, args=(redangle_mp,))
    p1.start()
    gimmegimmegimme(False)
    arm_goto(armlo)
    print("All systems nominal™")
    print("Press Enter to get cooking: ")
    input()
    while True:
        report = "Cur:" + state_names[cur_state]
        # fetch data from imageproc thread

        if cur_state == SEEK_CUBE:
            if state_timer == 0:
                seek_target = seek_current
                cumulative_stack = 0
            serr = seek_target - seek_current
            seekint += serr*0.001
            if seekint > intmax: seekint = intmax
            if seekint < -intmax: seekint = -intmax
            turn = seekP * serr + seekD * ((lastseek - serr)/0.001) + seekI * seekint
            if abs(turn) < 0.5: pass
            elif turn > 0: turn += seekFeedforward
            elif turn < 0: turn -= seekFeedforward
            lastseek = serr
            report += "\tSeekErr=" + f3d(serr) + "\tTurn=" + f3d(turn)
            wheelturn(turn)
            if state_timer % 350 == 0:
                dorept = True
                report += "\tTarget=" + f3d(seek_current+18)
                seek_target = seek_current + 10

            if seen_for >= 4:
                transition(ANGLE_FOLLOW)

        elif cur_state == ANGLE_FOLLOW:
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
            report += "\tAngleErr=" + f3d(redangle) + "\tTurn=" + f3d(turn)
            wheelturn(turn)

            if abs(redangle) < angleToMove:
                trans_timer += 1
            else:
                trans_timer = 0
            if trans_timer > 6:
                transition(MOVE_FOLLOW)
            elif notseen_for >= 4:
                transition(SEEK_CUBE)
        elif cur_state == MOVE_FOLLOW:
            wheelfwd(24)
            report += "\tAngleErr= " + f3d(redangle) + "\tCubeHeight=" + f3d(cube_vfrac.value)
            if abs(redangle) > moveToAngle:
                trans_timer += 1
            else:
                trans_timer = 0
            if trans_timer > 2:##and cube_vfrac.value < 0.75:
                transition(ANGLE_FOLLOW)
            elif timeofflight.range < 140:
                transition(FINAL_APPROACH)
            elif notseen_for >= 4:
                if cube_vfrac.value > 0.75:
                    transition(FINAL_APPROACH)
                else:
                    transition(SEEK_CUBE)
        elif cur_state == FINAL_APPROACH:
            wheelfwd(20)
            tof_range = timeofflight.range
            maxtime = 42
            report += "\tTimeout " + str(state_timer)+"/"+str(maxtime) + "\tTOFRange=" + f3d(tof_range)
            if state_timer > maxtime:
                transition(SEEK_CUBE)
            elif tof_range < 90:
                transition(GRAB_STACK)
        elif cur_state == GRAB_STACK:
            wheelfwd(0)
            if not actual_grab:
                time.sleep(0.5)
                transition(VICTORY)
            elif state_timer > 20:
                wrist(0)
                gimmegimmegimme(True)
                time.sleep(0.12)
                gimmegimmegimme(False)
                ravenbrd.set_motor_torque_factor(left_drive, 85)
                ravenbrd.set_motor_torque_factor(right_drive, 85)
                ravenbrd.set_motor_speed_factor(left_drive,35,reverse=True) #not sure why need rev
                ravenbrd.set_motor_speed_factor(right_drive,35)
                gimmegimmegimme(True, t=0.4)
                ravenbrd.set_motor_torque_factor(left_drive, 85)
                ravenbrd.set_motor_torque_factor(right_drive, 85)
                ravenbrd.set_motor_speed_factor(left_drive, 0) #not sure why need rev
                ravenbrd.set_motor_speed_factor(right_drive, 0)
                time.sleep(0.5)
                arm_goto(armhi)
                
                if cumulative_stack == REDONLY:
                    wrist(redchute)
                    time.sleep(0.7)
                    release_cube(True)
                    time.sleep(0.5)
                    release_cube(False)
                else:
                    wrist(redchute if cumulative_stack == REDONTOP else greenchute)
                    time.sleep(0.7)
                    release_cube(True)
                    time.sleep(0.7)
                    wrist(greenchute if cumulative_stack == REDONTOP else redchute)
                    time.sleep(1.3)
                    release_cube(False)
                    time.sleep(0.4)

                time.sleep(0.7)
                wrist(0)
                time.sleep(0.7)
                arm_goto(armlo)
                time.sleep(0.5)
                transition(VICTORY)
            report += "\tType: " + ["", "RED", "REDONTOP", "GREENONTOP"][cumulative_stack]
        elif cur_state == RELEASE_GREENS:
            pass
        elif cur_state == VICTORY:
            if state_timer == 1000:
                pass
        else:
            wheelfwd(0)
        
        if redangle_new.value:
            dorept = True
            if redangle_new.value == 2:# or stack_type.value == 1:
                seen_for = 0
                notseen_for += 1
                report += "\t(No Stack)"
            else:
                notseen_for = 0
                seen_for += 1
                report += "\t(" + ["", "RED", "REDONTOP", "GREENONTOP"][stack_type.value] + " Seen)"
                if cur_state == ANGLE_FOLLOW or cur_state == MOVE_FOLLOW:
                    if cumulative_stack <= REDONLY:
                        if stack_type.value > REDONLY:
                            cumulative_stack = stack_type.value
                        elif stack_type == REDONLY:
                            cumulative_stack = REDONLY
                    else:
                        if stack_type.value > REDONLY and stack_type.value != cumulative_stack:
                            cumulative_stack = stack_type.value
                redangle = redangle_mp.value
            redangle_new.value = 0
        zvel = imu.get_data()[1][2]
        redangle += factor * zvel * (180 / pi) * 0.001
        seek_current += zvel * (180 / pi) * 0.001
        state_timer += 1
        if dorept or rept_cycle % 20 == 0:
            print(report)
        dorept = False
        rept_cycle += 1
        time.sleep(0.001)
