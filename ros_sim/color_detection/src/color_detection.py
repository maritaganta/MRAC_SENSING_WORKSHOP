# rospy for the subscriber

import cv2
import numpy as np 
import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
colors = ['red', 'green', 'blue', 'yellow']


def show_image(img, window_name): 
    img_res = cv2.resize(img, None, fx=0.3, fy=0.3)
    cv2.imshow(window_name, img_res)
    cv2.waitKey(1)


def get_color_range(color):

    if(color == 'red'):
        lower_range = (0, 200, 102)
        upper_range = (15, 255, 255) 
 
    elif(color == 'green'):
        lower_range = (110, 0, 0)
        upper_range = (140, 255, 255) 

    elif(color == 'blue'):
        lower_range = (110, 0, 0)
        upper_range = (140, 255, 255) 

    else: # Yellow color
        lower_range = (110, 0, 0)
        upper_range = (140, 255, 255) 

    
    return lower_range, upper_range

def detect_color(img, lower_range, upper_range):

    # Perform a Gaussian filter 
    image_gauss = cv2.GaussianBlur(img, (15,15), 0)

    # Convert image to HSV
    hsv_image = cv2.cvtColor(image_gauss, cv2.COLOR_BGR2HSV)

    # Get color mask
    mask = cv2.inRange(hsv_image, lower_range, upper_range)

    # Define rectangular kernel 5x5
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(25,25))

    # Apply openning
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Apply dilation
    #mask = cv2.dilate(mask, kernel, iterations = 1)


    return mask

def get_max_area(mask): 
    contours, hierarchy = cv2.findContours(mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    
    contour_max = []
    area_max = 1000
    center = (-1,-1)

    # For each contour
    for cnt in contours:
        # Get area of the contour 
        area = cv2.contourArea(cnt)

        # Filter per area size
        if(area > area_max):
            area_max = area 
            contour_max = cnt

            # Get center of the contour using cv2.moments
            M = cv2.moments(cnt)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            center = (cX, cY)

    return contour_max, area_max, center
    



def get_zero_vel(vel): 
    vel.linear.x = 0.0
    vel.linear.y = 0.0
    vel.linear.z = 0.0

    vel.angular.x = 0.0
    vel.angular.y = 0.0
    vel.angular.z = 0.0
    
    return vel


def image_callback(msg):
    
    rospy.loginfo("Image received")

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    vel = Twist()
    vel = get_zero_vel(vel)


    # Convert your ROS Image message to OpenCV2
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    show_image(img, "Robot camera")

    # Get half width of the image 
    w_2 = img.shape[1] / 2

    
    # Look for color 
    ##########################################

    # Get color range
    lower_range, upper_range = get_color_range('red')
    
    # Detect color 
    mask = detect_color(img, lower_range, upper_range) 

    # Show mask 
    show_image(mask, "Color detection")


    # Find contours and get max area
    contour, area, center = get_max_area(mask)
    print("Maximum area: ", area)

    # Draw contour and center of the maximum contour
    cv2.drawContours(img, contour, contourIdx=-1, color=(0, 255, 0), thickness=6, lineType=cv2.LINE_AA)
    cv2.circle(img, center, 5, (0,200,200), 10)
    
    show_image(img, "Contour detection")

    
    # If you can see the color 
    color_pixels = np.count_nonzero(mask)
    if(area > 30000):
        print("Cylinder detected")

        #######################################################################
        # If the color is between 0.5 and 0.9, it will stop moving -> vel = 0 #
        # If the color is < 0.5 -> it will move towards the color             #
        # If the color is > 0.9 -> it will backwards the color                # 
        #######################################################################

        # Too close -> go backwards
        if(area > 450000):
            print("The robot is too close to the cylinder, going backwards")
            vel.linear.x = -0.1

        # Too far -> move towards
        elif(area < 300000):
            print("Approaching the cylinder")
            vel.linear.x = 0.3

            # Move to the part of the image with more color pixels 
            ########################################################################
                
            # If the color detected is on the right part of the image with an offset
            print(center[0])
            print(w_2+100)
            print(w_2-100)
            if(center[0] >= w_2+100):
                # Spin to the right
                print("Spinning to the right")
                vel.angular.z = -0.3

            # If the color detected is on the left part of the image with an offse
            elif(center[0] <= w_2-100): 
                # Spin to the left
                print("Spinning to the left")
                vel.angular.z = 0.3
        else:
        	print("Stopping")
        	pass

    # If the color is not detected, spin 
    else:
        print("Looking for color: spinning to the left")
        # Move to the right 
        vel.angular.z = 0.6
        

    # Publish velocity
    print(vel)
    pub.publish(vel)


   
def main():
    rospy.init_node('image_listener')

    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
