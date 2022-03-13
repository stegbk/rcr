# import the necessary packages
from distutils.log import INFO
from cv2 import sqrt
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import time    #https://docs.python.org/fr/3/library/time.html
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en
from matplotlib.pyplot import step
from pprint import pprint as pp
from typing import List
from actuator import Actuator
from load_calib_test import *
import numpy as np
from logging import Logger
from robot import CubeBot
from math import sqrt


SQUARE_MIN_PIXELS = 10000
SQUARE_MAX_PIXELS = 60000
logger = Logger(__file__)
TEST_IMAGE_NAME = 'test'
TEST_IMAGE_EXT = '.png'
DEFAULT_SIDE_TO_SQRT_AREA_LIMIT = 0.30

class Eyes():
    camera:PiCamera = None
    rawCapture:PiRGBArray = None
    all_contours = []
    found_contours = []
    square_centers = []
    bot:CubeBot = None
    save_images = False
    image_index = 0
    min_area = SQUARE_MIN_PIXELS
    max_area = SQUARE_MAX_PIXELS
    show_wait_time = 0
    squareness_limit = DEFAULT_SIDE_TO_SQRT_AREA_LIMIT

    def __init__(self, cb:CubeBot):
        self.bot = cb
        self.camera = PiCamera()
        self.rawCapture = PiRGBArray(self.camera)

    def display_and_pause(self, image, name='default name'):
        cv2.imshow(name, image)
        cv2.waitKey(self.show_wait_time)

    def obtain_image(self):
        self.camera.capture(self.rawCapture, format="bgr")
        image = self.rawCapture.array
        self.rawCapture.truncate(0)
        return image

    def set_cube_bot(self, cb:CubeBot):
        self.bot = cb

    def save_image(self, image, imageName=None, imageExtension=None):
        if imageName is None:
            imageName = TEST_IMAGE_NAME
        if imageExtension is None:
            imageExtension = TEST_IMAGE_EXT
        cv2.imwrite('{}_{}{}'.format(imageName, self.image_index, imageExtension), image)
        self.image_index += 1

    def image_to_channel_image(self, image, channel_index):
        channel = np.uint8(image[:,:,channel_index])
        logger.info(np.shape(image))
        channel_image = np.uint8(np.zeros(np.shape(image)))
        channel_image[:,:,channel_index] = channel
        return channel_image

    def find_squares_in_image_channel(self, channel_image):
        gray = cv2.cvtColor(channel_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, 5)
        thresh = cv2.threshold(blur, 10, 255, cv2.THRESH_BINARY_INV)[1]
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
        sharpen = cv2.Canny(close, 10, 255)
        cnts = cv2.findContours(sharpen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        return cnts

    def filter_squares_by_size(self, all_cnts):
        filtered_cnts = []
        for c in all_cnts:
            area = cv2.contourArea(c)
            x,y,w,h = cv2.boundingRect(c)
            if area > self.min_area and area < self.max_area:
                filtered_cnts.append(c)
        return filtered_cnts

    def decide_square(self, c):
        area = cv2.contourArea(c)
        x, y, w, h = cv2.boundingRect(c)
        area_side = sqrt(area)
        width_error = abs(area_side - w)
        height_error = abs(area_side - h)
        wef = width_error/area_side
        hef = height_error/area_side
        if wef > self.squareness_limit or hef > self.squareness_limit:
            return False
        return True

    def filter_squares_by_shape(self, all_cnts=None):
        if all_cnts is None:
            all_cnts = self.all_contours

        square_contours = []
        for c in all_cnts:
            if self.decide_square(c):
                square_contours.append(c)
        return square_contours

    def find_squares_in_image(self, image):
        cnts = []
        for i in range(3):
            channel_image = self.image_to_channel_image(image, i)
            tmp_cnts = self.find_squares_in_image_channel(channel_image)
            logger.warning("for channel {} found {} contours total".format(i, len(tmp_cnts)))
            tmp_cnts = self.filter_squares_by_size(tmp_cnts)
            logger.warning("for channel {} found {} contours that were big enough".format(i, len(tmp_cnts)))
            tmp_cnts = self.filter_squares_by_shape(tmp_cnts)
            logger.warning("for channel {} found {} contours that were square enough".format(i, len(tmp_cnts)))
            cnts.extend(tmp_cnts)
        return cnts

    def apply_squares_to_image(self, image, cnts=None):
        if cnts is None:
            cnts = self.all_contours
        for c in cnts:            
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(image, (x, y), (x + w, y + h), (36,255,12), 2)
        return image
    
    def retract_arms_update_squares(self, axis:int):
        if self.bot is None:
            raise Exception('The bot must be initialized to obtain face images')

        self.bot.retract_arms(axis)
        image = self.obtain_image()
        self.bot.pause()
        self.bot.extend_arms(axis)
        cnts = self.find_squares_in_image(image)
        self.all_contours.extend(cnts)

    def obtain_face_image_squares(self):
        if self.bot is None:
            raise Exception('The bot must be initialized to obtain face images')
        self.retract_arms_update_squares(X_AXIS)
        self.retract_arms_update_squares(Y_AXIS)

    def save_image_with_squares(self):
        image = self.obtain_image()
        square_image = self.apply_squares_to_image(image, self.all_contours)
        self.save_image(square_image)

    def display_image_with_squares(self):
        image = self.obtain_image()
        image = np.uint8(image)
        square_image = self.apply_squares_to_image(image, self.all_contours)
        self.display_and_pause(square_image)

def main():
    logger.setLevel(INFO)
    logger.info('Starting acquisition')
    bot = CubeBot()
    bot.home()
    brn = Eyes(bot)
    bot.pause()
    brn.obtain_face_image_squares()
    for i in range(3):
        bot.flip(X_AXIS, 1)
        brn.obtain_face_image_squares()
    brn.display_image_with_squares()
    brn.save_image_with_squares()


if __name__ == '__main__':
    main()