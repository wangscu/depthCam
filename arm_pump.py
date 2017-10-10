import TY as ty
import numpy as np
from ctypes import *
import cv2
import matplotlib.pyplot as plt
import vtk
import pcl
from vtk.util import numpy_support
from transformations import *
from uf.wrapper.swift_api import SwiftAPI
from uf.utils.log import *
from time import sleep
from uf.utils.log import *

logger_init(logging.VERBOSE)
#logger_init(logging.DEBUG)
#logger_init(logging.INFO)


swift = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'})


p = [165, -66, 88]
swift.set_position(p[0], p[1], 200, speed=1500)
sleep(10)
swift.set_position(p[0], p[1], p[2], speed=1500)
sleep(10)
swift.set_pump(True)
sleep(2)
swift.set_position(p[0], p[1], 200, speed=1500)
sleep(10)
swift.set_position(203, 200, 100, speed=1500)
sleep(10)
swift.set_pump(False)