#Sliding Rails Control with motor driver

import rospy
import threading
from Global.var import flag_ok, control_motor

def motor_write(cmd):
    control_motor.write(bytes(str(cmd), encoding='utf-8'))
    rospy.sleep(0.1)

def motor_cmd(cmd):
    _event = threading.Thread(target=motor_write,args=(str(cmd),), daemon=True)    
    _event.start()
    rospy.sleep(0.2)

def motor_wait():
    global flag_ok
    while flag_ok == False:
        data = ""
        while control_motor.inWaiting() > 0:
            data = control_motor.readline().decode(encoding='utf-8')
            if str("IDLE") in data :
                flag_ok = True
                print("IDLE")