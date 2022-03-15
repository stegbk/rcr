from face import Face
from square import Square
from typing import List

WHITE = 0
YELLOW = 1
RED = 2
BLUE = 3
ORANGE = 4
GREEN = 5

FACE_INDEXES = [WHITE, YELLOW, RED, BLUE, ORANGE, GREEN]

class Cube():
    white:Face = None
    yellow:Face = None
    red:Face = None
    blue:Face = None
    orange:Face = None
    green:Face = None
    face_array:List[Face] = [len(FACE_INDEXES)]

        