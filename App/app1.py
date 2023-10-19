#Application 1 runner

import time
import Jetson.GPIO as GPIO
import rospy

from Global.var import *
from Service.ImageProcces import image_proc

def App1Runner():
    global flag_target_ok
    global flag_ok
    prev_state = HOME
    while True:
        try:            
            #state_relay = GPIO.input(9)

            if not (prev_state == motor_state ) or (motor_state == HOME) or (prev_state == MID) :
                time_out = time.time()
                if not ( prev_state == MID and motor_state == STOP ):
                    prev_state = motor_state

            if ( time.time() - time_out > 15)  :
                #flag_ok = False
                #motor_dir = 1
                #motor_state = HOME
                #flag_mid = False
                #flag_seek_left = None
                #flag_seek_right = None
                #flag_target_ok = False
                #motor_position = 212
                time_out = time.time()
                for i in range (3):
                    GPIO.output(4,GPIO.LOW)
                    rospy.sleep(0.1)
                    GPIO.output(4,GPIO.HIGH)
                    rospy.sleep(0.1)
                #open_gripper(0.5)
                #motor_cmd("HOME")
                #move_to_pos((0,0,0),1) # Hung
                #rospy.sleep(5)
                print("TIME OUT")              
            
            while control_motor.inWaiting() > 0:
                data = control_motor.readline().decode(encoding='utf-8')                 

                if str("STOP:") in data :
                    time_response = time.time()
                    motor_position = int((str(data).split(":"))[1])
                    print("Pos: ",motor_position)

                if str("IDLE") in data :
                    print("time_response: ",time.time() - time_response)
                    time_out = time.time()
                    flag_ok = True
                    print("IDLE")

                if str("TL") in data or ( str("TR") in data ):
                    flag_target_ok = True
                    print("flag_target_ok")

            image_proc()

            if rospy.is_shutdown():
                break

        except Exception as e:
            rospy.logerr(e)
            break
    #GPIO.output(4,GPIO.HIGH) # turn off LED