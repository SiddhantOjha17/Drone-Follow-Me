import cv2
import time
import os.path
import os
from dronekit import connect
from file_utils import Logger
from polyphemus import process_stream

def get_vehicle():
    v = connect('/dev/ttyACM0', wait_ready=True)
    print("connected to vehicle")
    return v

def wait_for_arm(v):
    print ("Waiting for arming")
    while not v.armed:
        time.sleep(0.001)
    print ("ARMED")


def open_camera():
    # yuck - opencv has no way to count # of cameras, so do this hack of looking for /dev/video*
    # numCameras = len(filter(lambda s: s.startswith("video"), os.listdir("/dev")))

    c = cv2.VideoCapture(0)
    # # We start our search with higher numbered (likely external) cameras
    # for cnum in range(0, numCameras):
    #     c.open(numCameras - cnum - 1)
    #     if c.isOpened():
    return c

    #raise Exception('No cameras found')

print ("DroneScript - Visual-Follow Running")
v = get_vehicle()
mustarm = True
v.armed= True
while True:

    video_in = open_camera()
    homedir = os.path.expanduser("~")
    logger = Logger(path= homedir + "/Videos/")
    
    process_stream(video_in, logger, vehicle=v, require_arming=mustarm)

