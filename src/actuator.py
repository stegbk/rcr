import sys
from pprint import pprint as pp


class Actuator():
    pos:int
    rot:int
    e_home: int
    e_ret: int
    r_home: int
    cw: int
    ccw: int
    calib: dict

    E_HOME_FIELD = "e_home"
    E_RET_FIELD = "e_ret"
    R_HOME_FIELD = "r_home"
    CW_FIELD = "cw"
    CCW_FIELD = "ccw"
    
    def __init__(self, calibdict:dict):
        calib = calibdict
        self.e_home = calib[self.E_HOME_FIELD]
        self.e_ret = calib[self.E_RET_FIELD]
        self.r_home = calib[self.R_HOME_FIELD]
        self.cw = calib[self.CW_FIELD]
        self.ccw = calib[self.CCW_FIELD]
        self.home()
        
    def home(self):
        self.pos = self.e_home
        self.rot = self.r_home

    def ext(self):
        self.pos = self.e_home

    def ret(self):
        self.pos = self.e_ret

    def rcw(self):
        self.rot = self.cw

    def rccw(self):
        self.rot = self.ccw

    def rcent(self):
        self.rot = self.r_home

    
