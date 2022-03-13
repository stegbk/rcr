# import the necessary packages
from distutils.log import INFO
from cv2 import CAP_PROP_XI_AUTO_BANDWIDTH_CALCULATION, sqrt
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
from robot import *
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
DEFAULT_OVERLAP_THRESHOLD = 0.20

class Eyes():
    camera:PiCamera = None
    rawCapture:PiRGBArray = None
    all_contours = []
    found_contours = []
    square_centers = []
    bot:CubeBot = None
    save_images = False
    use_test_images = True
    image_index = 0
    read_image_index = 0
    min_area = SQUARE_MIN_PIXELS
    max_area = SQUARE_MAX_PIXELS
    show_wait_time = 0
    squareness_limit = DEFAULT_SIDE_TO_SQRT_AREA_LIMIT
    no_bot_test = True
    no_show_image_test = False


    def __init__(self, cb:CubeBot):
        self.bot = cb
        self.camera = PiCamera()
        self.rawCapture = PiRGBArray(self.camera)

    def display_and_pause(self, image, name='default name'):
        if self.no_show_image_test is False:
            cv2.imshow(name, image)
            cv2.waitKey(self.show_wait_time)

    def obtain_image(self):
        if self.use_test_images:
            image = self.load_image()
        else:
            self.camera.capture(self.rawCapture, format="bgr")
            image = self.rawCapture.array
            self.rawCapture.truncate(0)
            if self.save_images:
                self.save_image(image)
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

    def load_image(self, imageName=None, imageExtension=None):
        if imageName is None:
            imageName = TEST_IMAGE_NAME
        if imageExtension is None:
            imageExtension = TEST_IMAGE_EXT
        imagename = '{}_{}{}'.format(imageName, self.read_image_index, imageExtension)
        print('Reading {}'.format(imagename))
        image = cv2.imread(imagename)
        self.read_image_index += 1
        return image

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

    def filter_squares_by_location(self, all_cnts=None):
        if all_cnts is None:
            all_cnts = self.all_contours

        center_list = []
        contour_list = []
        for c in all_cnts:
            if len(center_list) == 0:
                center_list.append(self.contour_to_center(c))
            else:
                for c_compare in contour_list:
                    if not self.is_point_in_contour_box(c_compare, c):
                        center_list.append(c_compare)
        return all_cnts

    def is_point_in_contour_box(self, c_compare, c):
        x, y, w, h = cv2.boundingRect(c_compare)        
        x_cent, y_cent = self.contour_to_center(c)
        if x_cent <= x or y_cent <= y:
            return False
        if x_cent >= (x + w) or y_cent >= (y + h):
            return False
        return True

    def filter_boxes_by_location(self):
        tmp_cnt = self.filter_squares_by_location()
        logger.warning("Found {} boxes by location vs {} before".format(len(tmp_cnt), len(self.all_contours)))
        self.all_contours = tmp_cnt

    def get_rectangle_from_contour(self, c):
        x, y, w, h = cv2.boundingRect(c)
        return x, y, x+w, y+h

    def compute_overlap(self, c1, c2):
        x1_0, y1_0, x1_1, y1_1 = self.get_rectangle_from_contour(c1)
        x2_0, y2_0, x2_1, y2_1 = self.get_rectangle_from_contour(c2)
        width = self.calculateIntersection(x1_0, x1_1, x2_0, x2_1)
        height = self.calculateIntersection(y1_0, y1_1, y2_0, y2_1)
        overlap = width * height
        return overlap

    def get_box_area(self, c):
        x, y, w, h = cv2.boundingRect(c)
        a = w * h 
        return a


    def filter_squares_by_overlap(self, cnts=None):
        if cnts is None:
            cnts = self.all_contours

        all_contours_list = cnts.copy()
        duplicated_contours_list = []
        best_overlaps_list = []
        found_duplicate = True
        total_checks = 0
        while found_duplicate is True:
            found_duplicate = False
            index = 0
            for c1 in all_contours_list:
                best = c1
                if len(all_contours_list) > index:
                    i2 = 0
                    for c2 in all_contours_list[index + 1:]:
                        if self.arrayisin(c2, duplicated_contours_list) == False:
                            t_best = self.threshold_overlap(best, c2, DEFAULT_OVERLAP_THRESHOLD)
                            total_checks += 1
                            if t_best is not 0:
                                #found_duplicate = True
                                logger.warning("Found duplicate at index c1 = {} c2 = {} duplicate_list_len is {} total checks {}".format(index, i2, len(duplicated_contours_list), total_checks))
                                if t_best > 0:
                                    best = c1
                                    worst = c2
                                else:
                                    best = c2
                                    worst = c1
                                if self.arrayisin(worst, duplicated_contours_list) == False: 
                                    duplicated_contours_list.append(worst)
                                #found_duplicate = True
                        i2 += 1
                index += 1


        not_in_duplicates = []
        for c in all_contours_list:
            if self.arrayisin(c, duplicated_contours_list) == False:
                not_in_duplicates.append(c)
        return not_in_duplicates
                            
    def arrayisin(self, array, list_of_arrays):
        for a in list_of_arrays:
            if np.array_equal(array, a):
                return True
        return False
    def threshold_overlap(self, c1, c2, threshold):
        ol = self.compute_overlap(c1, c2)
        if ol == 0:
            return 0
        a1 = self.get_box_area(c1)
        a2 = self.get_box_area(c2)
        big_area = a1 if a1 > a2 else a2
        percent_area = ol / big_area
        if percent_area > threshold:
            if a1 >= a2:
                return 1
            else:
                return -1
        return 0

    def calculateIntersection(self, a0, a1, b0, b1):
        if a0 >= b0 and a1 <= b1: # Contained
            intersection = a1 - a0
        elif a0 < b0 and a1 > b1: # Contains
            intersection = b1 - b0
        elif a0 < b0 and a1 > b0: # Intersects right
            intersection = a1 - b0
        elif a1 > b1 and a0 < b1: # Intersects left
            intersection = b1 - a0
        else: # No intersection (either side)
            intersection = 0

        return intersection

    def contour_to_center(self, c):
        x, y, w, h = cv2.boundingRect(c)
        return (x + w/2), (y + w/2)

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
            if self.no_bot_test == False:
                raise Exception('The bot must be initialized to obtain face images')
        if not self.use_test_images:
            self.bot.retract_arms(axis)
        image = self.obtain_image()
        if not self.use_test_images:
            self.bot.pause()
            self.bot.extend_arms(axis)
        cnts = self.find_squares_in_image(image)
        self.all_contours.extend(cnts)

    def obtain_face_image_squares(self):
        if self.bot is None and self.no_bot_test is False:
            raise Exception('The bot must be initialized to obtain face images')
        self.retract_arms_update_squares(X_AXIS)
        self.retract_arms_update_squares(Y_AXIS)

    def save_image_with_squares(self):
        image = self.obtain_image()
        square_image = self.apply_squares_to_image(image, self.all_contours)
        self.save_image(square_image)

    def display_image_with_squares(self, title=None):
        image = self.obtain_image()
        image = np.uint8(image)
        square_image = self.apply_squares_to_image(image, self.all_contours)
        self.display_and_pause(square_image, title)

def main():
    logger.setLevel(INFO)
    logger.info('Starting acquisition')
    #bot = CubeBot()
    bot = None
    #bot.home()
    brn = Eyes(bot)
    #bot.pause()
    brn.obtain_face_image_squares()
    #for i in range(3):
    #    if not brn.use_test_images:
    #        bot.flip(X_AXIS, 1)
    #   brn.obtain_face_image_squares()
    brn.display_image_with_squares()
    #brn.filter_boxes_by_location()
    brn.all_contours = brn.filter_squares_by_overlap()
    brn.display_image_with_squares("Non-duplicated squares")
    brn.save_image_with_squares()


if __name__ == '__main__':
    main()