# rospy for the subscriber

import cv2
import numpy as np 
import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time 

bridge = CvBridge()
counter = 0
last_time = time.time()

def show_image(img, window_name): 
    img_res = cv2.resize(img, None, fx=0.3, fy=0.3)
    cv2.imshow(window_name, img_res)
    cv2.waitKey(1)



def image_callback(msg):
    global counter 
    global last_time 
    


    # Convert your ROS Image message to OpenCV2
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    actual_time = time.time()
    if(actual_time-last_time > 3.5):
       rospy.loginfo("Saving image")
       filename = "/home/esther/images_ros/image_{0}.jpg".format(counter)
       cv2.imwrite(filename, img)
       counter = counter + 1
       last_time = actual_time



   
def main():
    rospy.init_node('image_listener')

    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    
    
    
