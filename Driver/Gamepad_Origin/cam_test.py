import requests
import time 
from PIL import Image, ImageOps
import numpy as np     
import cv2

img_url = 'http://10.42.0.59/capture?'
img_frame_size = 'http://10.42.0.59/control?var=framesize&val=9'
counter = 0

def image_detector():
    data = np.ndarray(shape=(1, 224, 224, 3), dtype = np.float32)
    image = Image.open('')

requests.get(img_frame_size)
requests.get('http://10.42.0.59/control?var=flash&val=120')
requests.get('http://10.42.0.59/control?var=flash&val=120')
while True:
    print("Capturing...", counter)
    
    response = requests.get(img_url)
    if response.status_code:
        # fp = open('Downloads/testing_' + str(counter) + '.png', 'wb')
        # fp.write(response.content)
        image_np = np.frombuffer(response.content, dtype = np.uint8)
        image = cv2.imdecode(image_np, flags = cv2.IMREAD_COLOR)
        print(image)
        cv2.imshow('AI cam', image)
        cv2.waitKey(2)
        # fp.close()
        
    
cv2.destroyAllWindows()