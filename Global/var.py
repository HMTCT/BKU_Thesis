#Global variables

import Jetson.GPIO as GPIO
import serial
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(9, GPIO.IN)
GPIO.setup(4, GPIO.OUT, initial=GPIO.HIGH)

L0 = 84.4
L1 = 8.14
L2 = 128.4
L3 = 138.0
L4 = 16.8

state_relay = 1
state_complete = 1
time_read = 0
left_right = 0

LEFT  = 1
RIGHT = -1

control_motor = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)

state_scroll = 0

HOME = 0
SEEK_LEFT = 1
SEEK_RIGHT = 2
WAIT_ITEM = 3
STOP = 4
TARGET_LEFT = 5
TARGET_RIGHT = 6
WAIT_TARGET = 7
MID = 8

flag_ok = True
motor_dir = 1
motor_state = HOME
flag_mid = False
flag_seek_left = None
flag_seek_right = None
flag_target_ok = False
motor_position = 212
time_out = time.time()
flag_detect = True

time_detect = time.time()
time_response = time.time()

state = None
jetmax = None

image_queue = None
yolov5 = None
image_pub = None 
enter_srv = None 
exit_srv = None
running_srv = None 
heartbeat_srv = None