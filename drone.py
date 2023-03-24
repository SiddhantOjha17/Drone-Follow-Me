import cv2
import time
import os.path
import math
import os
import sys
from dronekit import connect, VehicleMode
from file_utils import Logger
from gui import render_crosshairs
from pid import Pid, print_graph
from red_blob_detection import RedBlobDetector


def landing(vehicle):
	print("Setting LAND mode...")
	vehicle.mode = VehicleMode("LAND")
	time.sleep(1)
	
def get_vehicle():
    v = connect('/dev/ttyACM0', wait_ready=True)
    print("connected to vehicle")
    return v

print ("DroneScript - Visual-Follow Running")
vehicle = get_vehicle()
print("Connected to Vehicle!!")
mustarm = True
vehicle.armed= True

# Set to false if you don't want to show any windows
showGUI = True

controller = Pid(kp=0.2, ki=0.05, kd=0.2)

vision_algorithm = RedBlobDetector()


def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """

    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6


    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.5)

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

def move_camera(vehicle, pwm):
    if vehicle:
        pwm = max(pwm, 1) # Ensure we never ask for negative or zero pwm values
        print(pwm)
        msg = vehicle.message_factory.rc_channels_override_encode(1, 1, 0, 0, 0, 0, 0, pwm, 0, 0)
        print(msg)
        vehicle.send_mavlink(msg)
        vehicle.flush()

def disable_camera(vehicle):
    if vehicle:
        msg = vehicle.message_factory.rc_channels_override_encode(1, 1, 0, 0, 0, 0, 0, 0, 0, 0)
        vehicle.send_mavlink(msg)
        vehicle.flush()

def process_stream(video_in, logger, vehicle=None, require_arming=False, *,start):
    
    while True:
        frame = get_frame(video_in)

        arm_and_takeoff_nogps(1.2)
        
        set_attitude(duration=1)
        
        # process_frame(logger, frame, vehicle)
                
        ch = 0xFF & cv2.waitKey(5)
        if ch == 27:
            break        
        
        end = time.perf_counter()
        if end - start > 11:
                        print("done lol")
                        landing(vehicle)   
                        break       
    disable_camera(vehicle)

    print ("Done with stream")
    if logger:
        logger.close()
    cv2.destroyAllWindows()
    video_in.release()

def get_frame(videoInput):
	gotNewFrame, frame = videoInput.read()
	if not gotNewFrame:
		print ("Reached EOF or webcam disconnected")
		sys.exit(0) 
	return frame

def process_frame(logger, frame, vehicle):
    if logger:
        logger.log(frame,vehicle)
        
    target = vision_algorithm.detect_target(frame)
    
    camera_pid(target, vehicle)
   
    render_crosshairs(frame, target)    
    if showGUI:
        cv2.imshow("frame", frame)

def camera_pid(target, vehicle):
    if target != None:
        _, cy = target
            
        control = controller.compute(cy, 240)
        print(type(control))
        pwm = control+1500
        pwm = int(pwm)
        move_camera(vehicle, pwm)

        print_graph(cy,pwm )


while True:
    video_in = cv2.VideoCapture(0)
    homedir = os.path.expanduser("~")
    logger = Logger(path= homedir + "/Videos/")
    
    start = time.perf_counter()
    process_stream(video_in, logger, vehicle=vehicle, require_arming=mustarm, start= start)
    
