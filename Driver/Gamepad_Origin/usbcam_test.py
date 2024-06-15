#!/usr/bin/env python3
import requests
import time 
import torch
import numpy as np     
import cv2
import os
import sys

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
    model = YOLO('dishdetect.pt')
except Exception as e:
    print(e)


# Image center: 240 320
while cap.isOpened():
    ret, frame = cap.read()
    if ret == True:
        start = time.time()
        try: 
            output = model.predict(frame, imgsz=640, conf=0.5, device = 0, save = False)
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(f"Exception type is: {exc_type.__name__}")
            print(f"Exception message: {str(e)}")
            print(f"File '{fname}', line {exc_tb.tb_lineno}")
        
        for result in output:
            img_show = result.plot()
            boxes = result.boxes.xywh.cpu().numpy()
            probs = result.probs
        cv2.imshow('Camera Feed', img_show)
        print("Object boxes:", boxes)
        print("Object boxes type:", type(boxes))
        
        print("fps", 1/(time.time() - start))
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    else: break
cap.release()
cv2.destroyAllWindows()
    
