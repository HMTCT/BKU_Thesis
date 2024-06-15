#!/usr/bin/env python3

import time 
import numpy as np     
import serial
import platform
import rospy
import torch
from std_msgs.msg import String
import cv2
import sys
import os
import base64
import camera_var
from stockfish import Stockfish
import re

# At the first time every setup the state machine will not having any state: NULL
NULL = -1
# FSM states
IDLE = 0
IMAGE_PROC = 1
GRAP = 2
STORE = 3
HOME = 4
PUT = 5
PUMP_ON = 6
PUMP_OFF = 7

GO_TILL = 8
GO_HOME = 9
MID = 10
CALIBRATE = 11

# The lable for model 
b_ma = 0
b_phao = 1
b_si = 2
b_tot = 3
b_tuong = 4
b_voi = 5
b_xe = 6
r_binh = 7
r_ma = 8
r_phao = 9
r_si = 10
r_tuong = 11
r_voi = 12
r_xe = 13 

chess_dict = {b_ma: 'n', b_phao: 'c', b_si: 'a', b_tot: 'p', b_tuong: 'k', b_voi: 'e', b_xe: 'r', r_ma: 'N', r_phao: 'C', r_si: 'A', r_binh: 'P', r_tuong: 'K', r_voi: 'E', r_xe: 'R',}
class movingNode:
    def __init__(self):
        # Initial the node
        rospy.init_node('robot_moving', anonymous=True)
        # Subcribe to take action every time camera has image
        rospy.Subscriber('isCamOpened', String, self.callback, queue_size=1)
        # self.pub = rospy.Publisher('image_process', String, queue_size=1)
        self.pubSerial = rospy.Publisher('serial_data', String, queue_size=2)
        # Define initial states
        self.state = NULL
        self.objects_list = []
        self.id_list = []
        self.fen_board = ""
        self.ack = False
        self.cur_frame = None
        self.model = None
        self.serialObject = None
        self.accumulate = 0
        self.stockfish = Stockfish("/home/dofarm/catkin_ws/src/auto_mode/scripts/stockfish")
        self.stockfish.set_skill_level(5)
        self.stockfish.set_depth(5)
        self.stockfish._put("setoption name UCI_Variant value xiangqi")

        self.turn = "r"
        
    
    def callback(self, msg):
        if self.state == IDLE: 
            self.state = IMAGE_PROC
            self.accumulate = 0
            self.objects_list = []
            self.cur_objs = []
            self.fen_board = ""
         # self.cur_frame = cv2.imread("/home/dofarm/catkin_ws/src/auto_mode/camera/image.jpeg")
        decoded_data = base64.b64decode(msg.data)
        # Convert byte data to numpy array
        np_data = np.frombuffer(decoded_data, dtype=np.uint8)
        # Decode numpy array to image
        image = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
        self.cur_frame = image

    def init_model(self):
        if torch.cuda.is_available():
            rospy.loginfo("Confirm CUDA recognized")
            from ultralytics import YOLO  
            self.model = None  
        else:
            rospy.logerr("No CUDA available fail to load model")
            return False
        try:
            # Load a pretrained YOLOv8n model
            self.model = YOLO('/home/dofarm/catkin_ws/src/auto_mode/scripts/Chess.engine')
            # Initial image to load up the model into Vram
            initial_frame = cv2.imread("/home/dofarm/catkin_ws/src/auto_mode/camera/initial_image.jpeg")
            self.model(initial_frame, imgsz=640, conf=0.5, device = 0, save = False)
        except Exception as e:
                print(e)
                rospy.logerr("Initial inference step fail, please check the error log!s")
                return False
        return True

    def init_serial(self):
        serialPort = 'COM4'
        if platform.system() == 'Linux':
            serialPort = '/dev/ttyACM0'
            
        serialBaudrate = 115200
        ack = False
        debounce = 0
        stopFlag = False

        self.serialObject = serial.Serial(
            port = serialPort, 
            baudrate = serialBaudrate,
            bytesize = serial.EIGHTBITS,
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE, 
            timeout = 0.01
        ) 
        time.sleep(10)
        
    def init_auto_mode(self):
        self.init_model()
        self.init_serial()
        counter = 0

        self.serialObject.write(bytes(str("!Initon#"), encoding='utf-8'))
        while(counter < 40):
            data = self.serialObject.readline().decode(encoding='utf-8')
            if data == "ACK\r\n":
                rospy.loginfo("Finished starting the arm")
                counter = 0
                break
            counter += 1
            time.sleep(1)

        if counter == 40: 
            rospy.logerr("Time out initialization")
            return False
        
        self.serialObject.write(bytes(str("!goauto#"), encoding='utf-8'))
        while(counter < 40):
            data = self.serialObject.readline().decode(encoding='utf-8')
            if data == "ACK\r\n":
                rospy.loginfo("Switched to auto mode")
                counter = 0
                break 
            counter += 1
            time.sleep(1)
        
        if counter == 40: 
            rospy.logerr("Time out initialization")
            return False
        
        self.go_till("till")
        self.go_till("home")
        rospy.loginfo("Finished the init process!")
        return True
    
    def chess_to_board(self, chess_list, classes, till_const = 0):
        line_string = ""
        if till_const == 0: image = self.cur_frame
        if till_const == 0:
            for chess in chess_list:
                cv2.circle(image, (int(chess[0]), int(chess[1])), 5, (0, 255, 0), -1) 
            base_x = 320
            base_y = 240
            x_start = -2
            x_end = 4
            x_length = 0.48
            y_length = 0.4666
        else:
            base_x = 320
            base_y = 226
            x_start = 1
            x_end = 5
            x_length = 0.53
            y_length = 0.54

        for i in range(x_start, x_end):
            empty = 0
            for j in range(-4, 5):
                check = False
                x_range = base_x + j*(31/x_length)
                y_range = base_y + i*(28/y_length)
                if till_const == 0: 
                    cv2.circle(image, (int(x_range), int(y_range)), 5, (0, 255, 0), -1) 
                for piece, chess in zip(classes, chess_list):
                    x, y, w, h  = chess
                    if x - w/2 < x_range and x + w/2 > x_range and y - h/2 < y_range and y + h/2 > y_range: 
                        check = True
                        if empty != 0: 
                            line_string += str(empty)
                            empty = 0
                        line_string += chess_dict[piece]
                        print(x_range, y_range)
                        print(chess_dict[piece], x, y)
                if check == False: empty+=1
            if empty != 0: 
                line_string += str(empty)
                empty = 0
            line_string += "/"
        if till_const == 0: cv2.imwrite("/home/dofarm/catkin_ws/src/auto_mode/camera/chess.jpeg", image)
        return line_string
    
    def get_move(self):
        self.fen_board = self.fen_board[:-1] + " " + self.turn + "- - 0 1"
        if self.turn == "b": self.turn = "w"
        else: self.turn = "b"
        self.stockfish.set_fen_position(self.fen_board)
        self.fen_board = ""
        moves = self.stockfish.get_best_move()
        print(moves)
        print(self.stockfish.get_board_visual())

        pairs = re.findall(r'([a-zA-Z])(\d+)', moves)
    
        # Convert each pair of characters to coordinates
        coordinates = []
        for pair in pairs:
            x, y = pair[0], pair[1]
            # Convert characters to numeric values
            x_num = ord(x) - 101  # Subtract 96 to convert 'a' to -4, 'b' to -3, ...
            y_num = int(y)  # Subtract 48 to convert '0' to 0, '1' to 1, ...
            if y_num < 5: 
                print("Coordinate out of the arm range, can cannot move") 
                coordinates = None
                break
            else:
                y_num = (9 - y_num)
                y_coor = -15 + x_num*31
                x_coor = 175 + y_num*28
                coordinates.append((x_coor, y_coor))
        return coordinates

    def go_till(self, state):
        counter = 0
        if state == "till":
            string = "!214.5:0:231:90:180:-90#"
            print('send: ', string)
            self.serialObject.write(bytes(str(string), encoding='utf-8'))
            while(counter < 40):
                data = self.serialObject.readline().decode(encoding='utf-8')
                if data == "ACK\r\n":
                    counter = 0
                    break
                elif data != "":
                    #print("Pub serial data to mqtt node")
                    self.pubSerial.publish(String(data))
                counter += 1
            time.sleep(2)
            self.serialObject.write(bytes(str("!manual#"), encoding='utf-8'))
            time.sleep(2)
            
            for i in range(0,6):
                self.serialObject.write(bytes(str("!000020#"), encoding='utf-8'))
                while(1):
                    data = self.serialObject.readline().decode(encoding='utf-8')
                    if data == "ACK\r\n":
                        break
                    elif data != "":
                        print("Pub serial data to mqtt node")
                        self.pubSerial.publish(String(data))
            time.sleep(3)
        else:
            self.serialObject.write(bytes(str("!gohome#"), encoding='utf-8'))
            while(counter < 40):
                data = self.serialObject.readline().decode(encoding='utf-8')
                if data == "ACK\r\n":
                    counter = 0
                    self.pubSerial.publish(String("<gohome>"))
                    break
                counter += 1
                time.sleep(1)

            self.serialObject.write(bytes(str("!goauto#"), encoding='utf-8'))
            while(counter < 40):
                data = self.serialObject.readline().decode(encoding='utf-8')
                if data == "ACK\r\n":
                    counter = 0
                    break
                counter += 1
                time.sleep(1)
            time.sleep(3)
          

    def camera_to_world(self, cam_mtx, r, t, img_points):
        inv_k = np.asmatrix(cam_mtx).I
        r_mat = np.zeros((3, 3), dtype=np.float64)
        print(type(r))
        cv2.Rodrigues(r, r_mat)
        # invR * T
        inv_r = np.asmatrix(r_mat).I  # 3*3
        transPlaneToCam = np.dot(inv_r, np.asmatrix(t))  # 3*3 dot 3*1 = 3*1
        world_pt = []
        coords = np.zeros((3, 1), dtype=np.float64)
        for img_pt in img_points:
            coords[0][0] = img_pt[0][0]
            coords[1][0] = img_pt[0][1]
            coords[2][0] = 1.0
            worldPtCam = np.dot(inv_k, coords)  # 3*3 dot 3*1 = 3*1
            # print(worldPtCam)
            # [x,y,1] * invR
            worldPtPlane = np.dot(inv_r, worldPtCam)  # 3*3 dot 3*1 = 3*1
            # print(worldPtPlane)
            # zc
            scale = transPlaneToCam[2][0] / worldPtPlane[2][0]
            # zc * [x,y,1] * invR
            scale_worldPtPlane = np.multiply(scale, worldPtPlane)
            # [X,Y,Z]=zc*[x,y,1]*invR - invR*T
            worldPtPlaneReproject = np.asmatrix(scale_worldPtPlane) - np.asmatrix(transPlaneToCam)  # 3*1 dot 1*3 = 3*3
            pt = np.zeros((3, 1), dtype=np.float64)
            pt[0][0] = worldPtPlaneReproject[0][0]
            pt[1][0] = worldPtPlaneReproject[1][0]
            pt[2][0] = 0
            world_pt.append(pt.T.tolist())
        return world_pt

    def world_to_arm(self, arm_coors, img_coors):
        x, y, z = arm_coors
        trans_mtx = [[1, 0, 0, x],
                    [0, np.cos(np.deg2rad(180)), -np.sin(np.deg2rad(180)), y],
                    [0, np.sin(np.deg2rad(180)), np.cos(np.deg2rad(180)), z],
                    [0, 0, 0, 1]]
        des_coors = np.array(trans_mtx) @ np.array(img_coors)
        return des_coors 

    def ack_check(self):
        data = self.serialObject.readline().decode(encoding='utf-8')
        if data == "ACK\r\n": 
            print("acknowledage")
            if self.state == NULL: self.state = CALIBRATE
            self.ack = True
                    
        elif data == "KCA\r\n": 
            print("stopped")
            self.ack = True

        elif data != "" and len(data) != 0:
            print("Publish data to mqtt:")
            print(data)
            self.pubSerial.publish(String(data))
                
        return self.ack
       
    def FSM(self):
        if self.state == CALIBRATE:
            try:
                counter = 0
                self.serialObject.write(bytes(str("!164.5:0:241:90:180:-90#"), encoding='utf-8'))
                while(counter < 40):
                    data = self.serialObject.readline().decode(encoding='utf-8')
                    if data == "ACK\r\n":
                        counter = 0
                        break
                    counter += 1
                    time.sleep(1)
                time.sleep(10)
                self.state = IDLE
                # calibrateTime = False
                # widths = [150 ,250, 320, 350, 450]
                # min_value = None
                # y = None
                # while(calibrateTime == False):
                #     image = self.cur_frame
                #     image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                #     blur = cv2.GaussianBlur(image, (5,5), 0)
                #     thresh = cv2.adaptiveThreshold(blur, 255, 1, 1, 11, 2)
                #     contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                #     max_area = 0
                #     bes_cnt = None
                #     for i in contours:
                #             area = cv2.contourArea(i)
                #             if area > 1000:
                #                     if area > max_area:
                #                         max_area = area
                #                         best_cnt = i


                #     mask = np.zeros((image.shape),np.uint8)
                #     cv2.drawContours(mask,[best_cnt],0,255,-1)
                #     cv2.drawContours(mask,[best_cnt],0,0,2)
                #     calibrateTime-=1
                #     # Initialize lists to store the topmost and bottommost points
                #     topmost_points = []
                #     # Threshold for detecting significant change in pixel value
                #     threshold = 20
                #     # Iterate over the width levels
                #     for width in widths:
                #         # Get the pixel values along the current width
                #         column = mask[:, width]
                #         # Find the topmost point where the pixel value changes significantly
                #         prev_pixel_value = None

                #         for index, pixel_value in enumerate(column):
                #             # print(pixel_value)
                #             if prev_pixel_value is not None:
                #                 diff = np.int16(prev_pixel_value) -  np.int16(pixel_value)
                #                 if(diff < -230):
                #                     # print("diff:", diff)
                #                     topmost_points.append((width, index))
                #                     break
                #             prev_pixel_value = pixel_value


                #     # Draw lines connecting the topmost points first
                #     bound = False
                #     image_color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                #     for i in range(len(topmost_points) - 1):
                #         pt1 = topmost_points[i]
                #         pt2 = topmost_points[i + 1]
                #         cv2.line(image_color, pt1, pt2, (0, 255, 0), 2)
                #         bound = True

                #     if bound == True: 
                #         cv2.imwrite("/home/dofarm/catkin_ws/src/auto_mode/camera/calibrate.jpeg", image_color)


                #     # Calculate the y angle that the board is not align
                #     min_value = min(item[1] for item in topmost_points)
                #     min_value_item = next(item[0] for item in topmost_points if item[1] == min_value)
                #     result_list = np.mean([(item[1] - min_value)/np.abs(item[0] - min_value_item) for item in topmost_points if item[0] != min_value_item])
                #     y = round(result_list*164.5,1)

                #     temp = topmost_points[0][1] - topmost_points[len(topmost_points)-1][1]
                #     if temp <= 8 and temp >= -8:
                #         if min_value - 80 <= 5 and min_value - 80 >= -5:
                #             self.state = IDLE
                #             calibrate = True
                    # print(temp, min_value - 80)
            except Exception as e:
                # print out the error type and line
                rospy.logerr(f"An error occurred in the calibration: {str(e)}")
            
        elif self.state == HOME:
            string = "!164.5:0:241:90:180:-90#"
            print('send: ', string)
            self.serialObject.write(bytes(str(string), encoding='utf-8'))
            self.pubSerial.publish(String("<gohome>"))
            time.sleep(6)
            self.state = IDLE
            self.ack = False

        elif self.state == MID:
            string = "!164.5:0:241:90:180:-90#"
            print('send: ', string)
            self.serialObject.write(bytes(str(string), encoding='utf-8'))
            self.state = PUT
            self.ack = False
            time.sleep(1)
        
        elif self.state == PUMP_OFF:
            string = "!PUMP:OFF#"
            print('send: ', string)
            self.serialObject.write(bytes(str(string), encoding='utf-8'))
            self.ack = False
            self.state = HOME
            time.sleep(2)
        
        elif self.state == GO_TILL:
            self.go_till("till")
            time.sleep(2)
            self.state = IMAGE_PROC

        elif self.state == GO_HOME:
            self.go_till("home")
            time.sleep(2)
            if self.cur_objs is not None and len(self.cur_objs) != 0:
                print("Start to GRAP here")
                self.state = GRAP
            else: 
                print("Moving coordinate out of range")
                self.state = IDLE

        elif self.state == IMAGE_PROC:
            print("Processing image: ")
            output = []
            try: 
                # frame = cv2.undistort(self.cur_frame, camera_var.K_array, camera_var.Dis_array, None, camera_var.New_array)
                frame = self.cur_frame
                output = self.model.track(frame, imgsz=640, conf=0.8, device = 0, save = False)
            except Exception as e:
                # print out the error type and line
                rospy.logerr(f"An error occurred in the inference process: {str(e)}")

            check = False
            for result in output:
                try:
                    img_show = result.plot()
                    classes = result.boxes.cls.int().tolist()
                    objects = result.boxes.xywh.cpu().numpy()
                    ids = result.boxes.id.cpu().numpy()
                    if len(objects) != 0: check = True
                    probs = result.probs
                    new_list = list(zip(objects, classes))
                    avrg_objs = []
                    if len(self.objects_list) != 0:
                        compare = True
                        if len(self.objects_list) != len(new_list):
                            print(len(self.objects_list))
                            print(len(new_list))
                            compare = False
                        else:
                            for old, new in zip(self.objects_list, new_list):
                                old_object, old_class = old
                                new_object, new_class = new
                                if old_class == new_class and abs(old_object[0] - new_object[0]) < 8 and abs(old_object[1] - new_object[1]) < 8:
                                    new_x = old_object[0]*0.2 + new_object[0]*0.8
                                    new_y = old_object[1]*0.2 + new_object[1]*0.8
                                    new_w = old_object[2]*0.2 + new_object[2]*0.8
                                    new_h = old_object[3]*0.2 + new_object[3]*0.8
                                    avrg_objs.append((new_x, new_y, new_w, new_h))
                                else: 
                                    print(old_class, new_class)
                                    print(old_object)
                                    print(new_object)
                                    compare = False
                                    avrg_objs = objects
                                    break
                        if compare == True: self.accumulate +=1
                        else: self.accumulate = 0
                        self.objects_list = list(zip(avrg_objs, classes))  
                    else: self.objects_list = new_list
                    print("Accumulate: ")
                    print(self.accumulate)
                    self.cur_objs = []
                    if self.fen_board == "": cv2.imwrite("/home/dofarm/catkin_ws/src/auto_mode/camera/result_1.jpeg", img_show)
                    else: cv2.imwrite("/home/dofarm/catkin_ws/src/auto_mode/camera/result_2.jpeg", img_show)
                    if self.accumulate == 6:
                        if self.fen_board == "":
                            print("till to see")
                            print("Fen:")
                            self.fen_board += self.chess_to_board(objects, classes, 0)
                            print(self.fen_board)
                            self.state = GO_TILL
                            cv2.imwrite("/home/dofarm/catkin_ws/src/auto_mode/camera/result_1.jpeg", img_show)
                        else: 
                            print("go home from till")
                            print("Fen:")
                            self.fen_board += self.chess_to_board(objects, classes, 1)
                            print(self.fen_board)
                            self.state = GO_HOME
                            self.cur_objs = self.get_move()
                            cv2.imwrite("/home/dofarm/catkin_ws/src/auto_mode/camera/result_2.jpeg", img_show)
     
                        self.accumulate = 0
                        # self.cur_objs = []
                        # # Extract fen from here: 
                        # print(ids)
                        # for box in objects:
                        #     print(box)
                        #     # Calculate the values
                        #     *coordinates, _ = self.camera_to_world(camera_var.New_array, camera_var.R_array, camera_var.T_array, np.array([box[0:2]]).reshape((1, 1, 2)))[0][0]
                        #     arm_coordinates = self.world_to_arm([164.5, 0, 241], [[coordinates[0]], [coordinates[1]], [0], [1]])
                        #     # Append the tuple containing the values to the list
                        #     self.cur_objs.append(arm_coordinates)
                       
                except Exception as e:
                    print(e)
                    continue
            if check == False: self.state = IDLE
            
        elif self.state == GRAP:
            object = self.cur_objs
            print(object)
            y =  object[0][1]
            print(y)
            # y = np.round((y  - (-6.5))/31)*31 
            # y = np.round(y/31)*31 
            x = object[0][0]
            print(x)
            # x = 220 + np.round((x  - (220))/28)*28
            # x = 266 + np.round((x  - (266))/28)*28
            # x = 238 + np.round((x  - (238))/28)*28
            string = "!" + str(x) + ":" +  str(y) + ":60:90:180:-90"
            string += "#"
            print('send: ', string)
            self.serialObject.write(bytes(str(string), encoding='utf-8'))
            self.ack = False
            self.state = PUMP_ON
            time.sleep(5)
        
        elif self.state == PUT:
            object = self.cur_objs
            y = object[1][1]
            print(y)
            # y = np.round((y  - (-6.5))/31)*31 
            # y = np.round(y/31)*31 
            x = object[1][0]
            print(x)
            # x = 220 + np.round((x  - (220))/28)*28
            # x = 266 + np.round((x  - (266))/28)*28
            # x = 238 + np.round((x  - (238))/28)*28
            string = "!" + str(x) + ":" +  str(y) + ":75:90:180:-90"
            string += "#"
            print('send: ', string)
            self.serialObject.write(bytes(str(string), encoding='utf-8'))
            self.ack = False
            self.state = PUMP_OFF
            time.sleep(5)
        
        elif self.state == PUMP_ON:
            string = "!PUMP:ON#"
            print('send: ', string)
            self.serialObject.write(bytes(str(string), encoding='utf-8'))
            self.ack = False
            self.state = MID
            time.sleep(2)

if __name__ == '__main__':
    # Create an instance of node and start to process  
    node = movingNode()
    init_state = node.init_auto_mode()
    while(init_state):
        if node.ack_check() == True:
            node.FSM()
            



