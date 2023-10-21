import rospy
import queue
import numpy as np
import hiwonder
import threading
import sys

from yolov5_tensorrt import Yolov5TensorRT
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image

from Global.config import ROS_NODE_NAME, TRT_ENGINE_PATH, TRT_INPUT_SIZE, TRT_NUM_CLASSES
from Global.var import flag_ok, state, jetmax
from App.app1 import App1Runner

from Service.ROSCallback import enter_func, exit_func, set_running
from Service.ROSCallback import heartbeat_srv_cb

from Service.SlidingRailsControl import motor_wait, motor_cmd
from Service.ArmControl.Gripper import open_gripper
from Service.ArmControl.Moving import go_home

class FruitClassification:
    def __init__(self):
        self.lock = threading.RLock()
        self.is_running = False
        self.moving_box = None
        self.image_sub = None
        self.heartbeat_timer = None
        self.runner = None
        self.count = 0
        self.camera_params = None
        self.K = None
        self.R = None
        self.T = None
        self.is_target = False

    def reset(self):
        self.is_running = False
        self.moving_box = None
        self.image_sub = None
        self.heartbeat_timer = None
        self.runner = None
        self.count = 0

    def load_camera_params(self):
        self.camera_params = rospy.get_param('/camera_cal/card_params', self.camera_params)
        if self.camera_params is not None:
            self.K = np.array(self.camera_params['K'], dtype=np.float64).reshape(3, 3)
            self.R = np.array(self.camera_params['R'], dtype=np.float64).reshape(3, 1)
            self.T = np.array(self.camera_params['T'], dtype=np.float64).reshape(3, 1)

def Init():
    global state
    global jetmax
    global flag_ok
    global image_queue 
    global yolov5
    global image_pub, enter_srv, exit_srv ,running_srv, heartbeat_srv

    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    image_queue = queue.Queue(maxsize=2)
    state = FruitClassification()
    state.load_camera_params()
    if state.camera_params is None:
        rospy.logerr("Can not load camera parameters")
        sys.exit(-1)
    jetmax = hiwonder.JetMax()
    #self.sucker = hiwonder.Sucker()
    go_home(1)
    yolov5 = Yolov5TensorRT(TRT_ENGINE_PATH, TRT_INPUT_SIZE, TRT_NUM_CLASSES)
    #ROS_NODE_NAME2 = "fruit_classification"
    #image_pub2 = rospy.Publisher('/%s/image_result' % ROS_NODE_NAME2, Image, queue_size=1)  # register result image pub
    image_pub = rospy.Publisher('/%s/image_result' % ROS_NODE_NAME, Image, queue_size=1)  # register result image pub
    enter_srv = rospy.Service('/%s/enter' % ROS_NODE_NAME, Trigger, enter_func)
    exit_srv = rospy.Service('/%s/exit' % ROS_NODE_NAME, Trigger, exit_func)
    running_srv = rospy.Service('/%s/set_running' % ROS_NODE_NAME, SetBool)
    heartbeat_srv = rospy.Service('/%s/heartbeat' % ROS_NODE_NAME, SetBool)
    #move_to_pos((0,0,-105),3)
    open_gripper(0.5)

    #GPIO.output(4,GPIO.LOW) # turn on LED    
    motor_wait()
    flag_ok = False
    motor_cmd("HOME")

def Start():
    App1Runner()

