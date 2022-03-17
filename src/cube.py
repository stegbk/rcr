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
    faces = None

    def __init__(self):
        self.faces = [len(FACE_INDEXES)]

    #def add_face(self, image_rl, image_ud, squares):
        