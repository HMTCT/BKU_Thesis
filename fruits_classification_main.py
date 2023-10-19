#!/usr/bin/env python3

import threading
from Rte import Rte


#------------------------------------------------#
#---------------Unused functions?----------------#
#------------------------------------------------#

def scroll_ON():
    global state_scroll
    global motor_dir

    if ( state_scroll == 0):
        state_scroll = 1
        print("Bang chuyen: ON ")
        # send message by rs485
        if motor_dir > 0:
           _event = threading.Thread(target=motor_cmd,args=("RUN",), daemon=True)    
           _event.start()
           rospy.sleep(0.2)
           motor_dir = -1
           _event = threading.Thread(target=motor_cmd,args=("65500",), daemon=True)    
           _event.start()
           rospy.sleep(0.2)

        # send message by rs485
        if motor_dir < 0:
           _event = threading.Thread(target=motor_cmd,args=("RUN",), daemon=True)    
           _event.start()
           rospy.sleep(0.2)
           motor_dir = 1
           _event = threading.Thread(target=motor_cmd,args=("500",), daemon=True)    
           _event.start()
           rospy.sleep(0.2)

def scroll_OFF():
    global state_scroll
    global motor_dir

    if ( state_scroll == 1):
       state_scroll = 0
       print("Bang chuyen: OFF ")
    # send message by rs485
       _event = threading.Thread(target=motor_cmd,args=("0",), daemon=True)    
       _event.start()
       rospy.sleep(0.2)

def scroll_state():
    global state_scroll
    return state_scroll

#------------------------------------------------#
#---------------------End------------------------#
#------------------------------------------------#


#------------------------------------------------#
#---------------Main function--------------------#
#------------------------------------------------#

if __name__ == '__main__':
    Rte.Init()
    Rte.Start()
    
#------------------------------------------------#
#---------------------End------------------------#
#------------------------------------------------#

