#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Libraries
import time    #https://docs.python.org/fr/3/library/time.html
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en
import sys
import readchar.readchar
import json
from pprint import pprint as pp
from typing import List
from actuator import Actuator



CALIB_FILE = "calibration.json"
R_INDEX = 0
U_INDEX = 1
L_INDEX = 2
D_INDEX = 3
X_AXIS = 0
Y_AXIS = 1
CW = 1
CCW = -1

calibration = {}
with open(CALIB_FILE) as infile:
    calibration = json.load(infile)

print("Calibration is:")
pp(calibration)

servos:List[Actuator] = []
servos.append(Actuator(calibration["R"]))
servos.append(Actuator(calibration["U"]))
servos.append(Actuator(calibration["L"]))
servos.append(Actuator(calibration["D"]))

#Constants
nbPCAServo = 8
#Parameters
MIN_IMP  =[500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500]
MAX_IMP  =[2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]
MIN_ANG  =[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MAX_ANG  =[180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180]

#Objects
pca = ServoKit(channels=16)
channel_letter = {
    "d": 0,
    "w": 1,
    "a": 2,
    "z": 3,
    "l": 4,
    "i": 5,
    "j": 6,
    "m": 7
    }
current_angle = []

step_size = 10
init_min = 0
init_max = 180
init_rotate = [100, 100, 85, 97]
init_extension = [56, 100, 67, 86]

u0 = [56, 100, 67, 86, 100, 100, 85, 97]
u1 = [56, 100, 67, 86, 100, 160, 85, 97]
u2 = [56, 180, 67, 86, 100, 160, 85, 97]
u3 = [56, 180, 67, 86, 100, 100, 85, 97]
u4 = [56, 100, 67, 86, 100, 100, 85, 97]


def init():
    for i in range(nbPCAServo):
        pca.servo[i].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])

#    for i in range(0,4):
#        print("setting {}".format(i))
#        pca.servo[i].angle = init_extension[i]
#        current_angle.append(init_extension[i])

#    print("retracting")
#    time.sleep(0.5)
#    for i in range(4, nbPCAServo):
#        print("setting {}".format(i))
#        pca.servo[i].angle  = init_rotate[i - 4]
#        current_angle.append(init_rotate[i - 4])


def pause():
    c = readchar.readchar()
        
        
def update_angles():
    for i in range(nbPCAServo):
        newang  = current_angle[i]
        print("Setting {} to {}".format(i, newang))
        pca.servo[i].angle = newang
    time.sleep(0.1)
    

def to_angles():
    newangles = [0] * 8
    for i in range(int(nbPCAServo/2)):
        newangles[i] = servos[i].pos
        newangles[i + int(nbPCAServo/2)] = servos[i].rot
    handle_angle(newangles)
    
    return newangles
    
def handle_angle(newangles):
    diff = [0, 0, 0, 0, 0, 0, 0, 0]
    for i in range(nbPCAServo):
        diff[i] = current_angle[i] - newangles[i]

    steps = 10
    channel = -1
    for i in range(nbPCAServo):
        if diff[i] != 0:
            stepsize = int(diff[i]/steps)
            print("Channel {} step is {}".format(i, stepsize))
            #for k in range(steps):
            #      current_angle[i] = current_angle[i] + stepsize
            #      update_angles()
            channel = i
        current_angle[i] = newangles[i]
            
    outheader = ""
    outstring = ""
    outsignal = ""
    for i in range(nbPCAServo):
        outheader = outheader + "{:5d}".format(i)
        if i == channel:
            outsignal = outsignal + "    *"
        else:
            outsignal = outsignal + "    _"
        outstring = outstring + "{:5d}".format(current_angle[i])

    print(outsignal)
    print(outheader)
    print(outstring)


def rotate_side(si:int, direction:int):
    if (direction < 0):
        servos[si].rccw()
    else:
        servos[si].rcw()
    to_angles()
    time.sleep(1.0)
    update_angles()

    time.sleep(1.0)
    servos[si].ret()
    to_angles()
    update_angles()

    time.sleep(1.0)
    servos[si].rcent()
    to_angles()
    update_angles()

    time.sleep(1.0)
    servos[si].ext()
    to_angles()
    update_angles()

    time.sleep(1.0)
    servos[si].home()
    to_angles()
    update_angles()
    
def flip(si: int, direction:int):
    offset = 1
    if si == Y_AXIS:
        offset = -1
    
    time.sleep(1.0)
    servos[si + offset].ret()
    servos[si + 2 + offset].ret()
    to_angles()
    update_angles()
    
    if (direction < 0):
        servos[si].rccw()
        servos[si + 2].rcw()
    else:
        servos[si].rccw()
        servos[si + 2].rcw()
    to_angles()
    time.sleep(1.0)
    update_angles()

    time.sleep(1.0)
    servos[si + offset].home()
    servos[si + 2 + offset].home()
    to_angles()
    update_angles()

    time.sleep(1.0)
    servos[si].ret()
    servos[si + 2].ret()
    to_angles()
    update_angles()

    time.sleep(1.0)
    servos[si].rcent()
    servos[si + 2].rcent()
    to_angles()
    update_angles()
    
    
    time.sleep(1.0)
    servos[si + offset].ext()
    servos[si + 2 + offset].ext()
    to_angles()
    update_angles()

    time.sleep(1.0)
    servos[si].home()
    servos[si + 2].home()
    to_angles()
    update_angles()

    
    
def main():
    for i in range(4):
        servos[i].home()
    to_angles()
    update_angles()
    str = ""
    
#    while True:
#        c = readchar.readchar() # reads one byte at a time, similar to getchar()
#        handle_angle(c)
#        update_angles()
#        if c == ' ':
#            break





if __name__=='__main__':
    init()
    main()
