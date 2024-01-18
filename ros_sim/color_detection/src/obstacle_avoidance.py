# rospy for the subscriber

import cv2
import numpy as np 
import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
import time 
import numpy as np 


bridge = CvBridge()
counter = 0
last_time = time.time()




def show_image(img, window_name): 
    img_res = cv2.resize(img, None, fx=0.3, fy=0.3)
    cv2.imshow(window_name, img_res)
    cv2.waitKey(1)

def get_zero_vel(vel): 
    vel.linear.x = 0.0
    vel.linear.y = 0.0
    vel.linear.z = 0.0

    vel.angular.x = 0.0
    vel.angular.y = 0.0
    vel.angular.z = 0.0
    
    return vel
    
    

def laser_callback(msg):
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    vel = Twist()
    vel = get_zero_vel(vel)
    
    
    mid = len(msg.ranges)//2

    right_side = np.array(msg.ranges[:mid])
    left_side = np.array(msg.ranges[mid:])
    center_side = np.array(msg.ranges[mid-20:mid+20])
   
    right_side = right_side[~np.isnan(right_side) & ~np.isinf(right_side)]
    left_side = left_side[~np.isnan(left_side) & ~np.isinf(left_side)] 
    center_side = center_side[~np.isnan(center_side) & ~np.isinf(center_side)] 
    
    right_side_mean = np.mean(right_side)
    left_side_mean = np.mean(left_side)
    center_side_mean = np.mean(center_side)

    print(left_side_mean)
    print(right_side_mean)
    dif = left_side_mean - right_side_mean

    print(center_side_mean)
    print(np.isnan(center_side_mean))
    print(dif)
        
    if(center_side_mean > 3):
       print("Objeto frontal")
       # Turn left
       vel.angular.z = 0.3
    
    elif(np.isnan(center_side_mean) or np.any(center_side > 3.4)):
      print("Objeto frontal backwards")
      # Go backwards
      vel.linear.x = -0.2
    
    else:
    
        if(dif > 1.0 or left_side_mean > 3): 
            # Turn right
            print("turn right")
            vel.angular.z = -0.3
            vel.linear.x = 0.1
            
        elif(dif < -1.0 or right_side_mean > 3): 
            # Turn left
            print("turn left")
            vel.angular.z = 0.3
            vel.linear.x = 0.1
        else:
            # Go straight
            print("go straight")
            vel.linear.x = 0.3

    #pub.publish(vel)



def image_callback(msg):
    global counter 
    global last_time 
    

    # Convert your ROS Image message to OpenCV2
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    #show_image(img, 'Image')
    
    actual_time = time.time()
    '''if(actual_time-last_time > 3.5):
       rospy.loginfo("Saving image")
       filename = "/home/esther/images_ros/image_{0}.jpg".format(counter)
       cv2.imwrite(filename, img)
       counter = counter + 1
       last_time = actual_time'''



   
def main():
    rospy.init_node('image_listener')

    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.Subscriber("/scan", LaserScan, laser_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    
    
    
    
    
    
    
    
