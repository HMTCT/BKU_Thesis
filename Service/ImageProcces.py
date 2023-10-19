import cv2
import math
import rospy
import numpy as np
import threading
import queue

import Jetson.GPIO as GPIO
import time

from Global.var import *
from Global.config import *

from Service.ArmControl.Gripper import open_gripper
from Service.ArmControl.Moving import move_to_pos, moving
from Service.SlidingRailsControl import motor_cmd

def image_proc():
    global state_scroll
    global flag_ok
    global flag_mid
    global motor_dir
    global motor_state
    global flag_seek_left
    global flag_seek_right
    global flag_target_ok
    global time_detect
    global flag_detect

    home_x = 0
    home_y = -(L1 + L3 + L4)
    home_z = L0 + L2
    delta_x = home_y
    delta_y = home_x
    delta_z = -50

    ros_image = image_queue.get(block=True)

    if (motor_state == TARGET_LEFT) or (motor_state == TARGET_RIGHT) : 
        if flag_target_ok == True:
            flag_target_ok = False
            # Move to target
            if (motor_state == TARGET_LEFT):
                set_pos_parallel(((delta_x-20)*(-1),delta_y,home_z+delta_z),0.5)
            elif (motor_state == TARGET_RIGHT):
                set_pos_parallel(((delta_x-20)*(1),delta_y,home_z+delta_z),0.5)
            rospy.sleep(0.8)
            # release item
            open_gripper(0.25)        
            # Wait 0.5 second
            rospy.sleep(0.5)
            _t = 0.5
            move_to_pos((0,0,0),_t) # Hung
            rospy.sleep(_t+0.1)
            state.moving_box = None
            state.runner = None
            print("FINISHED")
            flag_seek_left = None
            flag_seek_right = None
            if (motor_state == TARGET_LEFT):
                motor_state = SEEK_RIGHT
                print("SEEK_RIGHT")
            elif (motor_state == TARGET_RIGHT):
                motor_state = SEEK_LEFT
                print("SEEK_LEFT")
    
            return

    if state.is_running is False or ( state.runner is not None ):
        image_pub.publish(ros_image)
        #image_pub2.publish(ros_image)
        return

    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
    cur_x, cur_y, cur_z = jetmax.position
    #boxes, confs, classes = []
    cards = []
    if -10 < cur_x < 10:
        outputs = yolov5.detect(image)
        boxes, confs, classes = yolov5.post_process(image, outputs, 0.65)
        width = image.shape[1]
        height = image.shape[0]
        #cards = []
        for box, cls_conf, cls_id in zip(boxes, confs, classes):
            x1 = int(box[0] / TRT_INPUT_SIZE * width)
            y1 = int(box[1] / TRT_INPUT_SIZE * height)
            x2 = int(box[2] / TRT_INPUT_SIZE * width)
            y2 = int(box[3] / TRT_INPUT_SIZE * height)
            waste_name = TRT_CLASS_NAMES[cls_id]
            waste_class_name = ''
            for k, v in WASTE_CLASSES.items():
                if waste_name in v:
                    waste_class_name = k
                    break
        
            if y1 > 330 and x1 >540:
                continue

            if not ((80 < (x1 + x2 ) / 2  < 530) and (104 < (y1+y2)/2 < 416)):
                continue

            cards.append((cls_conf, x1, y1, x2, y2, waste_class_name))
            image = cv2.putText(image, waste_name + " " + str(float(cls_conf))[:4], (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLORS[waste_class_name], 2)
            image = cv2.rectangle(image, (x1, y1), (x2, y2), COLORS[waste_class_name], 3)
    

    if len(cards) == 0 :
        state.count = 0
        state.moving_box = None

        # Control bang chuyen
        if (motor_state == HOME):
            motor_state = SEEK_LEFT
            print("HOME")

        elif (motor_state == SEEK_LEFT):
            flag_detect = True
            motor_cmd("SL")
            flag_ok = False
            flag_seek_left = False
            flag_seek_right = None
            motor_state = WAIT_ITEM
            print("WAIT_ITEM")

        elif (motor_state == WAIT_ITEM):
            if flag_ok == True:
                flag_ok = False

                if flag_seek_left == False and flag_seek_right == False:
                    motor_state = MID
                    print("MIDDLE")
                elif flag_seek_left == False:
                    motor_state = SEEK_RIGHT
                    print("SEEK_RIGHT")


        elif (motor_state == SEEK_RIGHT):
            flag_detect = True
            motor_cmd("SR")
            flag_seek_right = False
            motor_state = WAIT_ITEM

            if flag_seek_left == None:
                motor_state = SEEK_LEFT
                print("SEEK_LEFT")

        elif (motor_state == STOP):
            flag_seek_left = None
            flag_seek_right = None

        elif (motor_state == MID):
            flag_detect = True
            motor_cmd("MID")
            flag_ok = False
            motor_state = STOP
        
    else:
        #if flag_detect == False:
            #state.count = 0
            #state.moving_box = None

        if state.moving_box is None:
            moving_box = max(cards, key=lambda card: card[0])
            conf, x1, y1, x2, y2, waste_class_name = moving_box
            c_x, c_y = (x1 + x2) / 2, (y1 + y2) / 2
            state.moving_box = c_x, c_y, waste_class_name
        else:
            l_c_x, l_c_y, l_waste_class_name = state.moving_box
            moving_box = min(cards, key=lambda card: math.sqrt((l_c_x - card[1]) ** 2 + (l_c_y - card[2]) ** 2))

            conf, x1, y1, x2, y2, waste_class_name = moving_box
            c_x, c_y = (x1 + x2) / 2, (y1 + y2) / 2

            image = cv2.rectangle(image, (x1, y1), (x2, y2), (255, 255, 255), 6)
            image = cv2.circle(image, (int(c_x), int(c_y)), 1, (255, 255, 255), 10)

            if math.sqrt((l_c_x - c_x) ** 2 + (l_c_y - c_y) ** 2) > 30:
                state.count = 0
            else:
                c_x = l_c_x * 0.2 + c_x * 0.8
                c_y = l_c_y * 0.2 + c_y * 0.8
                state.count += 1
            state.moving_box = c_x, c_y, waste_class_name
            if state.count == 1:
                time_detect = time.time()

            if state.count > 15 and (motor_state == WAIT_ITEM or motor_state == HOME ):
                # TURN OFF "Bang chuyen"
                state.count = 0
                state.moving_box = None
                motor_cmd("0")
                motor_state = STOP

            if state.count > 35 and not (motor_state == SEEK_RIGHT or motor_state == SEEK_LEFT ) :
                print("time_detect: ", time.time() - time_detect)
                state.count = 0
                GPIO.output(4,GPIO.LOW)
                rospy.sleep(0.2)
                GPIO.output(4,GPIO.HIGH)
                flag_detect = False
                state.runner = threading.Thread(target=moving, daemon=True)
                state.runner.start()

    rgb_image = image.tostring()
    ros_image.data = rgb_image
    image_pub.publish(ros_image)


def image_callback(ros_image):
    try:
        image_queue.put_nowait(ros_image)
    except queue.Full:
        pass

