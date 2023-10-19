#Set position for JetMax Arm

import hiwonder
import rospy
import justkinematics

from Global.var import jetmax

def return_valid_x(angle_raw):
    alpha=6
    if angle_raw < 31:
        return (30- 30+alpha)
    if angle_raw > 209:
        return (210- 30+alpha)
    return (angle_raw - 30+alpha)

def set_pos_x(angle, duration):
    hiwonder.pwm_servo1.set_position(return_valid_x(angle), duration)

def set_pos(postion, duration):
    angles = justkinematics.inverse(L4, postion)
    jetmax.set_position(postion,duration)
    # set_pos_x(angles[0],duration*0.8)
    set_pos_x(angles[0],0.5)
    rospy.sleep(duration+0.1)

def set_pos_parallel(postion, duration):
    angles = justkinematics.inverse(L4, postion)
    jetmax.set_position(postion,duration)
    set_pos_x(angles[0],duration*0.6)