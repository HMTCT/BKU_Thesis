#Control gripper of JetMax Arm

import hiwonder
import rospy

def open_gripper(duration):
    hiwonder.pwm_servo2.set_position(45,duration)
    rospy.sleep(duration+0.1)

def close_gripper(angle,duration):
    #global state_relay
    if angle < 45:
        return
    del_angle = angle -45
    _time = 0.03
    for i in range (int(del_angle/2)):
        hiwonder.pwm_servo2.set_position(45+i*2,_time)
        
        rospy.sleep(_time+0.01)
        state_relay = GPIO.input(9)        
        rospy.sleep(0.01)
        if (state_relay == 0 ):
            hiwonder.pwm_servo2.set_position(45+i*2+5,_time)
            print("current_state : ", str(state_relay))
            break
        
    rospy.sleep(0.5)