#!/usr/bin/env python2

import cv2
import gi
import rospy
from std_msgs.msg import String
'''
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib
'''
import numpy as np
import base64
import signal
import time  # Import the time module

import paho.mqtt.client as mqtt
import serial

class MQTTNode:
    def __init__(self, mqtt_broker_ip, mqtt_broker_port):
        self.base64_sub = rospy.Subscriber('isCamOpened', String, self.base64_callback, queue_size=10)
        
        # Define MQTT topics
        self.servo_vr_topic = "servo_vr"
        self.servo_topic = "servo"
        self.robot_mode_topic = "robot_mode"
        self.robot_cam_topic = "robot_cam"

        # Define ROS publishers for the respective MQTT topics
        self.servo_vr_pub = rospy.Publisher('servo_vr_ros', String, queue_size=2)
        self.servo_pub = rospy.Publisher('servo_ros', String, queue_size=10)
        self.robot_mode_pub = rospy.Publisher('robot_mode_ros', String, queue_size=10)

        # Subscribe to the ROS topic 'serial_data'
        self.serial_sub = rospy.Subscriber('serial_data', String, self.serial_callback, queue_size=10)

        # Create an MQTT client instance
        self.client = mqtt.Client()
        self.client.on_message = self.on_mqtt_message

        # Connect to the MQTT broker
        self.client.connect(mqtt_broker_ip, mqtt_broker_port, 60)
        self.client.loop_start()

    def base64_callback(self, msg):
        decoded_data = base64.b64decode(msg.data)
        np_data = np.frombuffer(decoded_data, dtype=np.uint8)
        image = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
        self.frame = image
        print("Type of ", type(image))
        rospy.loginfo("Receive data frame")
        for i in range(5):
            self.client.publish(self.robot_cam_topic, msg.data)
        
    def serial_callback(self, msg):
        rospy.loginfo("Received data from serial node: %s" % msg.data)
        if (msg.data[0] == "<"):
            self.client.publish(self.servo_topic, msg.data)

    def on_mqtt_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode("utf-8")
        rospy.loginfo("Received data from MQTT broker on topic %s: %s" % (topic, payload))

        if topic == self.servo_vr_topic:
            # Publish joystick data from VR to ROS node
            rospy.loginfo("Publishing to ROS topic 'servo_vr_ros': %s" % payload)
            self.servo_vr_pub.publish(String(payload))
        elif topic == self.robot_mode_topic:
            # Publish mode data from VR to ROS node
            rospy.loginfo("Publishing to ROS topic 'robot_mode_ros': %s" % payload)
            self.robot_mode_pub.publish(String(payload))

    def start(self):
        self.client.subscribe(self.servo_vr_topic, 0)
        self.client.subscribe(self.servo_topic, 0)
        self.client.subscribe(self.robot_mode_topic, 0)

        rospy.spin()

    def stop(self):
        self.client.loop_stop()
        self.ser.close()

def main():
    #Gst.init(None)
    rospy.init_node('vr_node')
    #server = GstServer()
    mqtt = MQTTNode("127.0.0.1", 1883)
    
    mqtt.start()
    #loop = GLib.MainLoop()

    def signal_handler(sig, frame):
        print("Signal received, exiting...")
 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
