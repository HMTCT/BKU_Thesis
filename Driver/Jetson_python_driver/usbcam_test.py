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
K_values = [[521.2983893968932 ,    0.0,               310.1165090086347],
 [  0.0,                520.4297206491902 , 220.21613563458055],
 [  0.0 ,                0.0    ,             1.0              ]]
# Values for R
R_values = [[-0.01470173416359],
 [ 0.05877006788942],
 [ 1.48890898522158]]

# Values for T
T_values  = [[ -1.53772720717909],
 [-24.1415269908699 ],
 [227.88669936031886]]

K_array = np.array(K_values)
R_array = np.array(R_values)
T_array = np.array(T_values)

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
        # [x,y,1] * invR
        worldPtPlane = np.dot(inv_r, worldPtCam)  # 3*3 dot 3*1 = 3*1
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


cap = cv2.VideoCapture(1)

if torch.cuda.is_available():
    print("Confirm CUDA recognized")
    from ultralytics import YOLO
    model = None
else:
    print("No CUDA available fail to load model")

try:
# Load a pretrained YOLOv8n model
    # model = YOLO('fruit_check.engine')
    model = YOLO('./runs/detect/dishdetect2/weights/best.pt')
except Exception as e:
    print(e)


# Image center: 240 320
# Home coordinates: 164.5, 0.0, 141.0, 90.0, 180.0, -90.0
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

    if state == IMAGE_PROC:
        ret, frame = cap.read()
        if ret == True:
            start = time.time()
            try: 
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
                        *coordinates, _ = camera_to_world(K_array, R_array, T_array, np.array([box[0:2]]).reshape((1, 1, 2)))[0][0]
                        arm_coordinates = world_to_arm([165, 0, 141], [[coordinates[0]], [coordinates[1]], [0], [1]])
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
    if state == GRAP:
        object = objects_list.pop()
        angle = np.arctan(object[1][0]/object[0][0])
        y = object[1][0]+ np.sin(angle)*30
        x = object[0][0]+ np.cos(angle)*30
        #string = "!" + str(object[0][0]) + ":" +  str(object[1][0]) + ":47:90:180:-90"
        string = "!" + str(x) + ":" +  str(y) + ":47:90:180:-90"
        string += "#"
        print('send: ', string)
        serialObject.write(bytes(str(string), encoding='utf-8'))
        ack = False
        state = IDLE
            
cap.release()
cv2.destroyAllWindows()
    
