

from optparse import BadOptionError
#from face import Face
import cv2

RL = 0
UD = 1
ROW_COL_INIT = -1
class Square():
    roi = None
    sort_rl_or_ud = RL
    row = ROW_COL_INIT
    col = ROW_COL_INIT
    
    def __init__(self, c):
        self.roi = c

    def set_sort_ordering(self, order):
        self.sort_rl_or_ud = order

    def get_center(self):
        x, y, w, h = cv2.boundingRect(self.roi)
        return (x + w/2), (y + h/2)

    def set_row_order(self):
        self.row = ROW_COL_INIT

    def set_col_order(self):
        self.row = -ROW_COL_INIT
        self.col = ROW_COL_INIT

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

