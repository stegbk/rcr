from square import Square
from typing import List
import cv2
import numpy

# The square positions
UL = 0
UM = 1
UR = 2
ML = 3
MM = 4
MR = 5
DL = 6
DM = 7
DR = 8

COLOR_SAMPLE_SQUARE_SIZE = 5

LOCATION_ARRAY = [UL, UM, UR, ML, MM, MR, DL, DM, DR]

class Face():
    squares = []
    xy_retract_image:numpy.ndarray = None
    ud_retract_image:numpy.ndarray = None
    use_subsquare_sampling = True

    def __init__(self, xy_retract_image, ud_retract_image):
        self.xy_retract_image = xy_retract_image
        self.ud_retract_image = ud_retract_image

    def set_square_list(self, square_list:List[Square]):
        self.squares = square_list

    def get_pixels_from_square_and_image(self, square:int):
        contour = self.squares[square].get_roi()
        pixel_list = []
        if square != ML and square != MR:
            t_pixel_list = self.image_to_roi_pixels(self.xy_retract_image, contour)
            pixel_list.extend(t_pixel_list)
        if square != UM and square != DM:
            t_pixel_list = self.image_to_roi_pixels(self.ud_retract_image, contour)
            pixel_list.extend(t_pixel_list)
        return pixel_list

    def image_to_roi_pixels(self, image, contour):
        x, y, w, h = cv2.boundingRect(contour)
        ROI:numpy.ndarray = image[y:y+h, x:x+w]
        t_pixel_list = self.array_to_pixel_list(ROI, w, h)
        return t_pixel_list

    def array_to_pixel_list(self, roi:numpy.ndarray, width, height):
        if self.use_subsquare_sampling == True:
            wc = round(width/2)
            hc = round(height/2)
            if wc < COLOR_SAMPLE_SQUARE_SIZE or hc < COLOR_SAMPLE_SQUARE_SIZE:
                raise Exception("Width of {} or hight of {} too small for subsquare of size {}".format(width, height, COLOR_SAMPLE_SQUARE_SIZE))
            roi = roi[hc - COLOR_SAMPLE_SQUARE_SIZE:hc + COLOR_SAMPLE_SQUARE_SIZE, wc - COLOR_SAMPLE_SQUARE_SIZE: wc + COLOR_SAMPLE_SQUARE_SIZE]
            width = 2 * COLOR_SAMPLE_SQUARE_SIZE
            height = 2 * COLOR_SAMPLE_SQUARE_SIZE
        return_list = []
        for i in range(height):
            for j in range(width):
                return_list.append(roi[i, j])

        return return_list



    