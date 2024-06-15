#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
import paho.mqtt.client as paho

broker_ip = "127.0.0.1"

def on_message(mosq, obj, msg): 
      mosq.publish('pong', 'ack', 0)
      received_data = msg.payload
      rospy.loginfo(received_data)

      # Convert bytes to string if needed
      if isinstance(received_data, bytes):
         received_data = received_data.decode('utf-8')
    
      pub.publish(String(received_data))
      print("publish ", received_data)


def talker():
      global pub
      pub = rospy.Publisher('chatter', String, queue_size=2)
      rospy.init_node('mqtt_listener', anonymous=True)

      client = paho.Client()
      client.on_message = on_message

      client.connect(broker_ip, 1883, 60)
      client.subscribe("servo_vr", 0)

      client.loop_start()

      rospy.spin()

if __name__ == '__main__':
      try:
          talker()
      except rospy.ROSInterruptException:
           pass
