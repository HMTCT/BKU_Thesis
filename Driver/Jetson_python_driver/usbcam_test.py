#!/usr/bin/env python3
import os
import sys
current_dir = os.path.dirname(os.path.dirname(__file__))
sys.path.append(current_dir)
from ultralytics import YOLO 
import torch

import requests
import time 
import torch
import numpy as np     
import cv2
import os
import sys
import serial

# The matrix is obtained at Z = 141
K_values = [[520.5878920236871,    0.0,               308.63922611555165],
 [  0.0,               519.8497596687193,  220.3444341942921 ],
 [  0.0,               0.0,                 1.0              ]]
# Values for R
R_values = [[-0.01689717926734],
 [ 0.06049339128915],
 [ 1.48868697380953]]

Distortion = [[-0.48897700370258,  0.32298112475634,  0.00273922139963,  0.0000355525706,
  -0.1388569310958 ]]

# New_cam = [[174.42768876274144,   0.0,              327.2321845803014 ],
#  [  0.0,               173.17936102922866, 209.16837237134735],
#  [  0.0,                 0.0,                 1.0              ]]
New_cam = [[467.3772858043902,    0.0,               342.4062370637581 ],
 [  0.0,               495.18207244180184, 232.61804686435005],
 [  0.0,                 0.0,                 1.0              ]]

# Values for T
T_values  = [[ -0.89702728290987],
 [-45.19828732433271],
 [227.66276204883042]]

K_array = np.array(K_values)
R_array = np.array(R_values)
T_array = np.array(T_values)
Dis_array = np.array(Distortion)
New_array = np.array(New_cam)
def camera_to_world(cam_mtx, r, t, img_points):
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
        print(worldPtCam)
        # [x,y,1] * invR
        worldPtPlane = np.dot(inv_r, worldPtCam)  # 3*3 dot 3*1 = 3*1
        print(worldPtPlane)
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

def world_to_arm(arm_coors, img_coors):
    x, y, z = arm_coors
    trans_mtx = [[1, 0, 0, x],
                 [0, np.cos(np.deg2rad(180)), -np.sin(np.deg2rad(180)), y],
                 [0, np.sin(np.deg2rad(180)), np.cos(np.deg2rad(180)), z],
                 [0, 0, 0, 1]]
    des_coors = np.array(trans_mtx) @ np.array(img_coors)
    return des_coors

serialPort = 'COM4'
serialBaudrate = 115200
ack = False
debounce = 0
stopFlag = False


serialObject = serial.Serial(
    port=serialPort, 
    baudrate=serialBaudrate,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE, 
    timeout=0.01
) 
time.sleep(10)
serialObject.write(bytes(str("!Initon#"), encoding='utf-8'))
time.sleep(10)
serialObject.write(bytes(str("!goauto#"), encoding='utf-8'))


cap = cv2.VideoCapture(0)

if torch.cuda.is_available():
    print("Confirm CUDA recognized")
    from ultralytics import YOLO
    model = None
else:
    print("No CUDA available fail to load model")

try:
# Load a pretrained YOLOv8n model
    # model = YOLO('fruit_check.engine')
    model = YOLO('./runs/detect/dishdetect2/weights/best_model.pt')
except Exception as e:
    print(e)


# Image center: 240 320
# Home coordinates: 164.5, 0.0, 141.0, 90.0, 180.0, -90.0
# chess board: left right corner 105 -121 105 go by 28 and 121 go by 31
objects_list = []
IMAGE_PROC = 1
GRAP = 2
STORE = 3
IDLE = 0
state = IMAGE_PROC
while cap.isOpened():
    data = serialObject.readline().decode(encoding='utf-8')
    if data == "ACK\r\n": 
        print("acknowledage")
        ack = True
    elif data == "KCA\r\n": 
        print("stopped")
        ack = True
    if ack == False: continue
    elif state == IDLE:
        print("here")    
        string = "!164.5:0:241:90:180:-90#"
        print('send: ', string)
        serialObject.write(bytes(str(string), encoding='utf-8'))
        state = IMAGE_PROC
        ack = False
        time.sleep(5)
        string = "!PUMP:OFF#"
        print('send: ', string)
        serialObject.write(bytes(str(string), encoding='utf-8'))
        time.sleep(2)
                           
    elif state == IMAGE_PROC:
        ret, frame = cap.read()
        frame = cv2.undistort(frame, K_array, Dis_array, None, New_array)
        if ret == True:
            start = time.time()
            try: 
                dot_radius = 3  # Adjust the radius of the dot as needed
                dot_color = (0, 255, 0)  # Green color, you can adjust this as needed
                thickness = -1  # Negative thickness to fill the circle
                #Enable 3 dot to map with the baord
                cv2.circle(frame, (320, 240), dot_radius, (0, 255, 0), thickness)
                cv2.circle(frame, (320 + 100, 240), dot_radius, (255, 255, 0), thickness)
                cv2.circle(frame, (320 - 100, 240), dot_radius, (0, 255, 255), thickness)
                #Dots used for checking you can enable for a full square
                # cv2.circle(frame, (320, 240 -100), dot_radius, (0, 255, 0), thickness)
                # cv2.circle(frame, (320 + 100, 240 -100), dot_radius, (0, 255, 0), thickness)
                # cv2.circle(frame, (320 - 100, 240 -100), dot_radius, (0, 255, 0), thickness)
                # cv2.circle(frame, (320, 240 +100), dot_radius, (0, 255, 0), thickness)
                # cv2.circle(frame, (320 + 100, 240 +100), dot_radius, (0, 255, 0), thickness)
                # cv2.circle(frame, (320 - 100, 240 +100), dot_radius, (0, 255, 0), thickness)
                output = model.track(frame, imgsz=640, conf=0.5, device = 0, save = False)
            except Exception as e:
                # print out the error type and line
                exc_type, exc_obj, exc_tb = sys.exc_info()
                fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                print(f"Exception type is: {exc_type.__name__}")
                print(f"Exception message: {str(e)}")
                print(f"File '{fname}', line {exc_tb.tb_lineno}")

            for result in output:
                try:
                    img_show = result.plot()
                    cls = result.boxes.cls
                    objects = result.boxes.xywh.cpu().numpy()
                    ids = result.boxes.id.cpu().numpy()
                    print(ids)
                    probs = result.probs
                    temp_list = []
                    for box in objects:
                        print(box)
                        *coordinates, _ = camera_to_world(New_array, R_array, T_array, np.array([box[0:2]]).reshape((1, 1, 2)))[0][0]
                        arm_coordinates = world_to_arm([164.5, 0, 241], [[coordinates[0]], [coordinates[1]], [0], [1]])
                        temp_list.append(arm_coordinates)
                        print("dist", arm_coordinates)
                    if len(objects_list) == 0: 
                        objects_list = temp_list
                    else: 
                        state = GRAP
                        print(temp_list)
                        print("Have to compared here")
                except Exception as e:
                    continue
            cv2.imshow('Camera Feed', img_show)
            # print("Object boxes:", boxes)
            # print("Object boxes type:", type(boxes))
            # print("fps", 1/(time.time() - start))

            if cv2.waitKey(1000) & 0xFF == ord('q'):
                break
    elif state == GRAP:
        object = objects_list.pop()
        angle = np.arctan(object[1][0]/object[0][0])
        y = object[1][0]
        print(y)
        print((y  - (-6.5))/31)
        y = -6.5 + np.round((y  - (-6.5))/31)*31 + np.sin(angle)*0
        x = object[0][0]
        print(x)
        print((x  - (215))/28)
        x = (220 + np.round((x  - (220))/28)*28) + np.cos(angle)*0
        #string = "!" + str(object[0][0]) + ":" +  str(object[1][0]) + ":45:90:180:-90"
        string = "!" + str(x) + ":" +  str(y) + ":60:90:180:-90"
        string += "#"
        print('send: ', string)
        serialObject.write(bytes(str(string), encoding='utf-8'))
        ack = False
        state = IDLE
        time.sleep(7)
        string = "!PUMP:ON#"
        print('send: ', string)
        serialObject.write(bytes(str(string), encoding='utf-8'))

    else: break
cap.release()
cv2.destroyAllWindows()
    
