from square import Square
from typing import List

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

LOCATION_ARRAY = [UL, UM, UR, ML, MM, MR, DL, DM, DR]

class Face():
    squares[] = None
    xy_retract_image = None
    ud_retract_image = None

    def __init__(self, xy_retract_image, ud_retract_image, contour_list):
        self.xy_retract_image = xy_retract_image
        self.ud_retract_image = ud_retract_image


    