#!/usr/bin/env python3

import os
import sys
import time 
import numpy as np     
import cv2
import os
import sys
import rospy
from std_msgs.msg import String
import base64
import camera_var
from stockfish import Stockfish
import time

# added for VR
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge

cap = ""
class camNode:
    def __init__(self):
        rospy.init_node('cam_processing', anonymous=True)
        # rospy.Subscriber('image_process', String, self.callback)
        self.isCamOpened = rospy.Publisher('isCamOpened', String, queue_size=1)
#        self.rosImgPublish = rospy.Publisher('rosImage', Image, queue_size=1)
#        self.bridge = CvBridge()

    def run(self):
        try:
            cap = cv2.VideoCapture(0)
            rospy.loginfo("Find the camera, starting to streaming image")
        except Exception as e:
            rospy.logwarn("Cannot find the camera, start to stream sampling image to avoid error and testing")
            print(e)
        rate = rospy.Rate(1)  # Define the rate at which to check the camera status (1 Hz in this example)
        i = 0
        while not rospy.is_shutdown():
            if cap.isOpened():
                ret, frame = cap.read()
                if ret == True: 
                    try:
                        # Convert OpenCV image to ROS Image message
#                        ros_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
#                        self.rosImgPublish.publish(ros_image_msg) 

                        frame = cv2.undistort(frame, camera_var.K_array, camera_var.Dis_array, None, camera_var.New_array)
                        _, buffer = cv2.imencode('.jpg', frame)
                        image_string = base64.b64encode(buffer).decode('utf-8')

                        # Dots used for checking you can enable for a full square
                        dot_radius = 3  # Adjust the radius of the dot as needed
                        dot_color = (0, 255, 0)  # Green color, you can adjust this as needed
                        thickness = -1  # Negative thickness to fill the circle
                        # Enable 3 dot to map with the baord
                        cv2.circle(frame, (320, 240), dot_radius, (0, 255, 0), thickness)
                        cv2.circle(frame, (320 + 100, 240), dot_radius, (255, 255, 0), thickness)
                        cv2.circle(frame, (320 - 100, 240), dot_radius, (0, 255, 255), thickness)
                        cv2.imwrite("/home/dofarm/catkin_ws/src/auto_mode/camera/image.jpeg", frame)
                        cv2.imwrite("/home/dofarm/catkin_ws/src/auto_mode/video/image" + str(time.time()) + ".jpeg", frame)
                        
                        self.isCamOpened.publish(image_string)

                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                    except Exception as e: 
                        rospy.logerr("Camera publisher error: {}".format(e))
                # If assume camera not opened, send the sample image to test the function
                else: 
                    cap.release()
                    cap = cv2.VideoCapture(0)
            else:
                cap.release()
                cap = cv2.VideoCapture(0)
                frame = cv2.imread("/home/dofarm/catkin_ws/src/auto_mode/camera/sample_image.jpeg")
                _, buffer = cv2.imencode('.jpg', frame)
                image_string = base64.b64encode(buffer).decode('utf-8')
                self.isCamOpened.publish(image_string)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            rate.sleep()
        cap.release()
            
if __name__ == '__main__':
    node = camNode()
    node.run()  
    cap.release()
    cv2.destroyAllWindows()       
    
    
