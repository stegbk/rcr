#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Libraries
import time    #https://docs.python.org/fr/3/library/time.html
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en
import sys
import readchar.readchar

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


def init():
    for i in range(nbPCAServo):
        pca.servo[i].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])

    for i in range(0,4):
        print("setting {}".format(i))
        pca.servo[i].angle = init_extension[i]
        current_angle.append(init_extension[i])

    print("retracting")
    time.sleep(0.5)
    for i in range(4, nbPCAServo):
        print("setting {}".format(i))
        pca.servo[i].angle  = init_rotate[i - 4]
        current_angle.append(init_rotate[i - 4])

        
def update_angles():
    for i in range(nbPCAServo):
        newang  = current_angle[i]
        pca.servo[i].angle = current_angle[i]
    time.sleep(0.01)
    
        
def handle_angle(letter):
    if letter.lower() in channel_letter.keys():
        print("letter is {}".format(letter))
        channel = channel_letter[letter.lower()]
        direction = 1
        if letter.lower() != letter:
            print('That was uppercase')
            direction = -1
        if current_angle[channel] + (direction * step_size) < MIN_ANG[channel]:
            current_angle[channel] = MIN_ANG[channel]
            print("MIN of {} hit at channel {}".format(MIN_ANG[channel], channel))
        elif current_angle[channel] + (direction * step_size) > MAX_ANG[channel]:
            current_angle[channel] = MAX_ANG[channel]
            print("MAX of {} hit at channel {}".format(MAX_ANG[channel], channel))

        else:
            current_angle[channel] = current_angle[channel] + (direction * step_size)
            print("new angle of {} hit at channel {}".format(current_angle[channel], channel))
    else:
        print("The letter {} is not a command".format(letter))
        channel = -1
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
        

def main():
    str = ""

    while True:
        c = readchar.readchar() # reads one byte at a time, similar to getchar()
        handle_angle(c)
        update_angles()
        if c == ' ':
            break

        



if __name__=='__main__':
    init()
    main()
