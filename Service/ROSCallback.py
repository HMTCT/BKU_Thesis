#Callback function for ROS

import rospy
import threading

from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from std_msgs.msg import Bool, SetBoolResponse, SetBoolRequest
from std_srvs.srv import Trigger, TriggerResponse
from jetmax_control.msg import SetServo

from Global.var import HOME, state, jetmax
from Global.config import ROS_NODE_NAME

from Service.ImageProcces import image_callback

def enter_func(msg):
    rospy.loginfo("enter")
    exit_func(msg)
    jetmax.go_home()
    state.reset()
    state.load_camera_params()
    state.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, image_callback)
    return TriggerResponse(success=True)


def exit_func(msg):
    global motor_state
    global flag_ok

    rospy.loginfo("exit")
    motor_state = HOME
    flag_ok = False
    state.is_running = False
    try:
        state.heartbeat_timer.cancel()
    except:
        pass
    if isinstance(state.runner, threading.Thread):
        state.runner.join()
    if isinstance(state.image_sub, rospy.Subscriber):
        rospy.loginfo('unregister image')
        state.image_sub.unregister()
    rospy.ServiceProxy('/jetmax/go_home', Empty)()
    rospy.Publisher('/jetmax/end_effector/sucker/command', Bool, queue_size=1).publish(data=False)
    rospy.Publisher('/jetmax/end_effector/servo1/command', SetServo, queue_size=1).publish(data=90, duration=0.5)
    
    
    return TriggerResponse(success=True)


def set_running(msg: SetBoolRequest):
    global motor_state

    motor_state == HOME
    if msg.data:
        rospy.loginfo("start running")
        state.is_running = True
    else:
        rospy.loginfo("stop running")
        state.is_running = False
    return SetBoolResponse(success=True)


def heartbeat_timeout_cb():
    rospy.loginfo('heartbeat timeout. exiting...')
    rospy.ServiceProxy('/%s/exit' % ROS_NODE_NAME, Trigger)


def heartbeat_srv_cb(msg: SetBoolRequest):
    rospy.logdebug("Heartbeat")
    try:
        state.heartbeat_timer.cancel()
    except:
        pass
    if msg.data:
        state.heartbeat_timer = threading.Timer(5, heartbeat_timeout_cb)
        state.heartbeat_timer.start()
    return SetBoolResponse(success=msg.data)