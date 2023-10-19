#Move JetMax Arm

import math
import hiwonder
import rospy
import justkinematics
import Jetson.GPIO as GPIO
from Global.var import *
from Global.config import *

from Service.ArmControl.SetPosition import set_pos, set_pos_parallel
from Service.ArmControl.Gripper import open_gripper, close_gripper
from Service.Camera import camera_to_world
from Service.SlidingRailsControl import motor_cmd


def go_home(duration):
    alpha = 6
    angles = justkinematics.inverse(L4, (0,-(L1 + L3 + L4), L0 + L2))
    jetmax.go_home(duration)
    set_pos_x(angles[0],duration)
    hiwonder.pwm_servo2.set_position(90,1)
    rospy.sleep(duration+0.1)

def move_to_target(duration,direct):
    global motor_dir
    global motor_state
    global flag_seek_left
    global flag_seek_right

    home_x = 0
    home_y = -(L1 + L3 + L4)
    home_z = L0 + L2
    delta_z = 0
    delta_x = home_y*0.7
    delta_y = home_y*0.7
    set_pos_parallel((delta_x*direct,delta_y,home_z+delta_z),duration)
    rospy.sleep(duration)

    state_relay = GPIO.input(9)
    if state_relay == 1:
        return move_and_open((0,0,0),0.5)

    delta_x = home_y
    delta_y = home_x
    delta_z = -50
    set_pos_parallel((delta_x*direct,delta_y,home_z),duration)
    rospy.sleep(duration*0.9)

    state_relay = GPIO.input(9)
    if state_relay == 1:
        return move_and_open((0,0,0),0.5)

    #set_pos_parallel(((delta_x-20)*direct,delta_y,home_z+delta_z),duration)
    #set_pos_parallel(((delta_x-20)*direct,delta_y,home_z+delta_z),duration)
    motor_dir = direct 
    rospy.sleep(duration)

def move_to_pos(position, duration):
    home_x = 0
    home_y = -(L1 + L3 + L4)
    home_z = L0 + L2
    delta_z = position[2]+20
    delta_y = position[1]
    delta_x = position[0]
    _duration = 2    
    set_pos((home_x+delta_x,home_y+delta_y,home_z+delta_z),_duration)

def move_and_open(position,_duration):
    open_gripper(0.5)
    move_to_pos(position,_duration)    
    rospy.sleep(0.1)

def moving():
    global flag_target_ok
    global motor_state
    global flag_seek_left
    global flag_seek_right

    try:
        c_x, c_y, waste_class_name = state.moving_box
        cur_x, cur_y, cur_z = jetmax.position

        x, y, _ = camera_to_world(state.K, state.R, state.T, np.array([c_x, c_y]).reshape((1, 1, 2)))[0][0]
        print("dist", x, y)
        rospy.loginfo("dist : " + str( x) + " ; " + str ( y) )

        t = math.sqrt(x * x + y * y + 140 * 140) / 140

        new_x, new_y = cur_x + x, cur_y + y
        nn_new_x = new_x + 15
        arm_angle = math.atan(new_y / new_x) * 180 / math.pi
        if arm_angle > 0:
            arm_angle = (90 - arm_angle)
        elif arm_angle < 0:
            arm_angle = (-90 - arm_angle)
        else:
            pass

        #-----Hung------

        # Move the gripper to item's position and open the gripper
        #hiwonder.pwm_servo2.set_position(45,0.5)
        #move_to_pos((x, y, -100),t)
        print("item :", waste_class_name )
        print("t1 : ",t)
        if motor_state == STOP:
            move_and_open((x, y-20, -110),t*0.5)
        else:
            flag_target_ok = False
            open_gripper(0.25)
            rospy.sleep(0.2)
            move_to_pos((0,0,0),1) # Hung
            rospy.sleep(1+0.1)
            print("GRIPPER: FAIL")
            return 

        # Wait 0.1 second
        rospy.sleep(0.1)
        # close gripper

        close_gripper(GRIPPER_ANGLES[waste_class_name],1)

        # Wait 0.5 second
        rospy.sleep(0.2)

        # Neu vat the bi rot / gap chua duoc vat the
        state_gripper = GPIO.input(9)
        if state_gripper == 0:
            print("state_gripper: pass")
            #move to target position
            target_pos = TARGET_POSITIONS[waste_class_name]
            cur_x, cur_y, cur_z = jetmax.position
            t = math.sqrt((cur_x - jetmax.ORIGIN[0]) ** 2 + (cur_y - jetmax.ORIGIN[1]) ** 2+ 140 * 140) / 140
            print("t2 : ",t)
            # Double-check
            state_gripper = GPIO.input(9)
            if state_gripper == 0:
                move_to_target(t*0.5,target_pos)
            else:
                #motor_state = STOP
                flag_target_ok = False
                open_gripper(0.25)
                rospy.sleep(0.2)
                move_to_pos((0,0,0),1) # Hung
                rospy.sleep(1+0.1)
                print("GRIPPER: FAIL")
                return

            # Wait 0.5 second
            rospy.sleep(0.1)
        
            if motor_state == STOP:
                state_gripper = GPIO.input(9)
                if state_gripper == 0:
                    
                    if TARGET_POSITIONS[waste_class_name] < 0:
                        motor_cmd("LEFT")
                        flag_target_ok == False
                        motor_state = TARGET_LEFT
                        flag_seek_left = None
                        flag_seek_right = None
                        print("TARGET_LEFT")
        
                    if TARGET_POSITIONS[waste_class_name] > 0:
                        motor_cmd("RIGHT")
                        flag_target_ok == False
                        motor_state = TARGET_RIGHT
                        flag_seek_left = None
                        flag_seek_right = None
                        print("TARGET_RIGHT")
        else: 
            open_gripper(0.25)
            rospy.sleep(0.2)
            move_to_pos((0,0,0),1) # Hung
            rospy.sleep(1+0.1)
            flag_seek_left = None
            flag_seek_right = None
            print("GRIPPER: FAIL")            

    finally:
        state.moving_box = None
        state.runner = None
        print("FINISH")
