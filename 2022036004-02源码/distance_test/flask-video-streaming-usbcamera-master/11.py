import os
import urllib
import traceback
import time
import sys
import numpy as np
import cv2
import pandas as pd
import math

VID_PATH = "http://192.168.1.25:666/video_feed"
VID_PATH = 0

capture = cv2.VideoCapture(VID_PATH)

ret, img_1 = capture.read()

while ret:
    cv2.imshow('video', img_1)

    if cv2.waitKey(1) == ord('q'):
        cv2.destroyAllWindows()
        break

    ret, img_1 = capture.read()