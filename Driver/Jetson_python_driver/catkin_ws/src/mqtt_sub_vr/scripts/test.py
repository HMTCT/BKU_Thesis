#!/usr/bin/env python 
  
import rospy 
from std_msgs.msg import String 
  
  
def publisher(): 
      
    # define the actions the publisher will make 
    pub = rospy.Publisher("serial_data", 
                          String, queue_size=10) 
    # initialize the publishing node 
    rospy.init_node('test', anonymous=True) 
      
    # define how many times per second 
    # will the data be published 
    # let's say 10 times/second or 10Hz 
    rate = rospy.Rate(10) 
    # to keep publishing as long as the core is running 
    while not rospy.is_shutdown(): 
        data = "The data that you wish to publish."
          
        # you could simultaneously display the data 
        # on the terminal and to the log file 
        rospy.loginfo(data) 
          
        # publish the data to the topic using publish() 
        pub.publish((data))
          
        # keep a buffer based on the rate defined earlier 
        rate.sleep() 
  
  
if __name__ == '__main__': 
    # it is good practice to maintain 
    # a 'try'-'except' clause 
    try: 
        publisher() 
    except rospy.ROSInterruptException: 
        pass
