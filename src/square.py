

import logging
from optparse import BadOptionError
#from face import Face
import cv2
from cv2 import sqrt
import numpy

RL = 0
UD = 1
ROW_COL_INIT = -1
STANDARD_COLORS = {
        'white': [255, 255, 255],
        'black': [0, 0, 0],
        'blue': [255, 0, 0],
        'green': [0, 255, 0],
        'red': [0, 0, 255],
        'yellow': [0, 255, 255],
        'orange': [0, 215, 255]    
    }

logger = logging.Logger(__file__)
class Square():
    roi = None
    sort_rl_or_ud = RL
    row = ROW_COL_INIT
    col = ROW_COL_INIT
    local_colors = STANDARD_COLORS
    
    def __init__(self, c):
        self.roi = c

    def set_sort_ordering(self, order):
        self.sort_rl_or_ud = order

    def get_center(self):
        x, y, w, h = cv2.boundingRect(self.roi)
        return (x + w/2), (y + h/2)

    def set_row_order(self):
        self.row = -ROW_COL_INIT

    def set_col_order(self):
        self.row = ROW_COL_INIT
        self.col = -ROW_COL_INIT

    def get_roi(self):
        return self.roi

    def __lt__(self, other):
        if not isinstance(other, Square):
            raise BadOptionError("{} is not an instance of {}".format(other.__name__, Square.__class__))
        x1, y1 = self.get_center()
        x2, y2 = other.get_center()
        if self.row == ROW_COL_INIT:
            diff = y1 - y2
        else:
            diff = x1 - x2
        return diff < 0

    def find_best_color_match_by_distance(input_color, local_colors=None):
        if local_colors == None:
            local_colors = STANDARD_COLORS
        
        best = numpy.linalg.norm([255, 255, 255])
        best_index = -1
        index = 0
        logger.info("Input color is {}".format(input_color))
        for k, v in local_colors.items():
            diff = input_color - v
            diff_mag = numpy.linalg.norm(diff)
            logger.info("For {} diff is mag {:2f} vect {}".format(k, diff_mag, diff))
            if diff_mag < best:
                best = diff_mag
                best_name = k
                best_value = v 
        return best_name, best_value               

    def find_best_color_match(input_color, local_colors=None):
        if local_colors == None:
            local_colors = STANDARD_COLORS
        best = -1.0
        best_index = -1
        best_name = ""
        index = 0
        input_colormag = numpy.linalg.norm(input_color)
        norm_color = input_color/input_colormag
        for k, v in local_colors.items():
            colormag = numpy.linalg.norm(v)
            norm_value = v/colormag
            test_value = numpy.dot(norm_value, norm_color)
            if test_value > best:
                best = test_value
                best_index = index
                best_name = k
        if best_name == 'white':
            # 1.7 is sqrt(2) but I'm lazy
            bt = local_colors['white'][0]/2.0
            gt = local_colors['white'][1]/2.0
            rt = local_colors['white'][2]/2.0
            sum_white = numpy.linalg.norm([bt, gt, rt])
            if input_colormag < sum_white:
                best_name = 'black'
                local_colors[best_name] = [1, 1, 1]
        return best_name, local_colors[best_name]

