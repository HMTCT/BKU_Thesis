#!/usr/bin/env python3
import torch 
import sys
import os
import cv2
import time
import requests
import time 
from PIL import Image, ImageOps
import numpy as np     




if torch.cuda.is_available():
    print("Confirm CUDA recognized")

from ultralytics import YOLO
print("here")
try:
# Load a pretrained YOLOv8n model
    model = YOLO('fruit_check.engine')
except Exception as e:
    print(e)
    
print("here")

# Define path to the image file

img_url = 'http://10.42.0.163/capture?'
img_frame_size = 'http://10.42.0.163/control?var=framesize&val=6'
counter = 0

requests.get(img_frame_size)
requests.get('http://10.42.0.163/control?var=flash&val=120')



# Open the camera
while True:
    print("Capturing...", counter)
    start = time.time()
    print(start)
    response = requests.get(img_url)
    
    if response.status_code:
        print('response: ', time.time())
        # fp = open('Downloads/testing_' + str(counter) + '.png', 'wb')
        # fp.write(response.content)
        image_np = np.frombuffer(response.content, dtype = np.uint8)
        image = cv2.imdecode(image_np, flags = cv2.IMREAD_COLOR)
        print('decode: ', time.time())
        try:
            # Run inference on 'bus.jpg' with arguments
            output = model.predict(image, imgsz=480, conf=0.2, device = 0, save = False)
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(f"Exception type is: {exc_type.__name__}")
            print(f"Exception message: {str(e)}")
            print(f"File '{fname}', line {exc_tb.tb_lineno}")
        print("fps", 1/(time.time() - start))
        for result in output:
            img_show = result.plot()
            boxes = result.boxes.xyxy.cpu().numpy()
            probs = result.probs
        cv2.imshow('Camera Feed', img_show)
        print(boxes)
    
        cv2.waitKey(1)
        # fp.close()
cv2.destroyAllWindows()       
