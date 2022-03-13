# Libraries
import time  # https://docs.python.org/fr/3/library/time.html
# https://circuitpython.readthedocs.io/projects/servokit/en
from adafruit_servokit import ServoKit
import sys
from matplotlib.pyplot import step
import readchar.readchar
import json
from pprint import pprint as pp
from pprint import pformat as pf
from typing import List
from actuator import Actuator
from logging import Logger

CALIB_FILE = "calibration.json"
R_INDEX = 0
U_INDEX = 1
L_INDEX = 2
D_INDEX = 3
X_AXIS = 0
Y_AXIS = 1
CW = 1
CCW = -1
DEFAULT_SLEEP_TIME = 1.0
DEFAULT_STEPS_ON_FLIP = 10
DEFAULT_FLIP_PAUSE_TIME = 0.1
DEFAULT_PAUSE_FOR_KEY_PRESS = False

# Constants
nbPCAServo = 8
# Parameters
MIN_IMP = [500, 500, 500, 500, 500, 500, 500,
           500, 500, 500, 500, 500, 500, 500, 500, 500]
MAX_IMP = [2500, 2500, 2500, 2500, 2500, 2500, 2500,
           2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]
MIN_ANG = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MAX_ANG = [180, 180, 180, 180, 180, 180, 180,
           180, 180, 180, 180, 180, 180, 180, 180, 180]

logger = Logger(name=__file__)

step_size = 10
init_min = 0
init_max = 180
init_rotate = [100, 100, 85, 97]
init_extension = [56, 100, 67, 86]

class CubeBot():
    calibration = {}
    servos: List[Actuator] = []
    pca = ServoKit(channels=16)
    current_angle = []
    move_sleep_time = DEFAULT_SLEEP_TIME
    steps_per_flip = DEFAULT_STEPS_ON_FLIP
    flip_pause_time = DEFAULT_FLIP_PAUSE_TIME
    wait_for_keypress = DEFAULT_PAUSE_FOR_KEY_PRESS


    def __init__(self):

        with open(CALIB_FILE) as infile:
            self.calibration = json.load(infile)
        logger.debug("Calibration is:")
        logger.debug(pf(self.calibration))
        self.servos.append(Actuator(self.calibration["R"]))
        self.servos.append(Actuator(self.calibration["U"]))
        self.servos.append(Actuator(self.calibration["L"]))
        self.servos.append(Actuator(self.calibration["D"]))
        for i in range(nbPCAServo):
            self.pca.servo[i].set_pulse_width_range(MIN_IMP[i], MAX_IMP[i])

        for i in range(0, 4):
            logger.info("setting {}".format(i))
            self.pca.servo[i].angle = init_extension[i]
            self.current_angle.append(init_extension[i])

        self.pause()
        for i in range(4, nbPCAServo):
            logger.info("setting {}".format(i))
            self.pca.servo[i].angle = init_rotate[i - 4]
            self.current_angle.append(init_rotate[i - 4])

    def update_angles(self):
        for i in range(nbPCAServo):
            newang = self.current_angle[i]
            logger.info("Setting {} to {}".format(i, newang))
            self.pca.servo[i].angle = newang
            time.sleep(self.flip_pause_time)

    def handle_angle(self, newangles):
        diff = [0, 0, 0, 0, 0, 0, 0, 0]
        for i in range(nbPCAServo):
            diff[i] = self.current_angle[i] - newangles[i]

        steps = 1
        channel = -1
        for i in range(nbPCAServo):
            if diff[i] != 0:
                channel = i
            self.current_angle[i] = newangles[i]

        outheader = ""
        outstring = ""
        outsignal = ""
        for i in range(nbPCAServo):
            outheader = outheader + "{:5d}".format(i)
            if i == channel:
                outsignal = outsignal + "    *"
            else:
                outsignal = outsignal + "    _"
            outstring = outstring + "{:5d}".format(self.current_angle[i])

        logger.debug(outsignal)
        logger.debug(outheader)
        logger.debug(outstring)
    
    def to_angles(self):
        newangles = [0] * 8
        for i in range(int(nbPCAServo/2)):
            newangles[i] = self.servos[i].pos
            newangles[i + int(nbPCAServo/2)] = self.servos[i].rot
        self.handle_angle(newangles)
        return newangles


    def rotate_side(self, si:int, direction:int):
        if (direction < 0):
            self.servos[si].rccw()
        else:
            self.servos[si].rcw()
        self.to_angles()
        self.update_angles()
        
        self.servos[si].ret()
        self.to_angles()
        self.update_angles()
        
        self.servos[si].rcent()
        self.to_angles()
        self.update_angles()
        
        self.servos[si].ext()
        self.to_angles()
        self.update_angles()
        
        self.servos[si].home()
        self.to_angles()
        self.update_angles()
        
    def flip(self, si: int, direction:int):
        offset = 1
        if si == Y_AXIS:
            offset = -1

        self.pause()
        # Retract the sides
        
        self.servos[si + offset].ret()
        self.servos[si + 2 + offset].ret()
        self.to_angles()
        self.update_angles()

            
        # Rotate slowly
        # Rotate
        if (direction < 0):
            first = self.servos[si].ccw
            second = self.servos[si + 2].cw
        else:
            first = self.servos[si].cw
            second = self.servos[si + 2].ccw

        steps = 10
        first_step_size = (first - self.servos[si].r_home)/steps
        second_step_size = (second - self.servos[si + 2].r_home)/steps
        logger.info("First step size is {} second is {}".format(first_step_size, second_step_size))
        for i in range(steps):
            self.servos[si].rot = self.servos[si].r_home + round(i * first_step_size)
            self.servos[si + 2].rot = self.servos[si + 2].r_home + round(i * second_step_size)
            self.to_angles()
            time.sleep(self.flip_pause_time)
            self.update_angles()    
            logger.info("Rotated to {} {}".format(self.servos[si].rot, self.servos[si + 2].rot))

        self.servos[si].rot = first
        self.servos[si + 2].rot = second
        self.to_angles    ()
        self.update_angles()    

        self.pause()
 
        # Return the sides
        
        self.servos[si + offset].ext()
        self.servos[si + 2 + offset].ext()
        self.to_angles    ()
        self.update_angles()    
        self.pause()

        # Retract the active clamps
        
        self.servos[si].ret()
        self.servos[si + 2].ret()
        self.to_angles    ()
        self.update_angles()    
        self.pause()

        # Rotate the active clamps back home
        
        self.servos[si].rcent()
        self.servos[si + 2].rcent()
        self.to_angles    ()
        self.update_angles()    
        self.pause()

        # Return the active clamps
        
        self.servos[si + offset].ext()
        self.servos[si + 2 + offset].ext()
        self.to_angles    ()
        self.update_angles()    
        self.pause()

        
        self.servos[si].home()
        self.servos[si + 2].home()
        self.to_angles    ()
        self.update_angles()    
        print("Done")
        self.pause()
    
    def pause(self):
        if self.wait_for_keypress:
            print("Press any key to continue")
            c = readchar.readchar()
        else:
            time.sleep(self.move_sleep_time)
    

    def retract_arms(self, axis:int):
        si = axis
        self.servos[si].ret()
        self.servos[si + 2].ret()
        self.to_angles    ()
        self.update_angles()    
        self.pause()

    def extend_arms(self, axis:int):
        si = axis
        self.servos[si].ext()
        self.servos[si + 2].ext()
        self.to_angles    ()
        self.update_angles()    
        self.pause()

    def home(self):
        for i in range(len(self.servos)):
            self.servos[i].home()

        self.to_angles()
        self.update_angles()
        self.pause()
 
 
def main():
    cb = CubeBot() 
    for i in range(4):
        cb.servos[i].home()
    cb.to_angles()
    cb.update_angles()
    cb.pause()
    str = ""


        
    for k in [1, -1]:
        for i in range(4):
            cb.rotate_side(i, k)

    for i in [Y_AXIS, X_AXIS]:
        for j in [CW, CCW]:
            cb.flip(i, j)

if __name__=='__main__':
    main()