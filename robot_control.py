#!/usr/bin/env python
import cv2
import numpy as np
import math
import rospy
from geometry_msgs.msg import Twist
import time

cap = cv2.VideoCapture(-4)

move=0 #initializing 

#defining function for publishing message to the robot via topic "cmd_vel"
def motion(v,theta):
    cmd_vel_topic= '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    velocity_message = Twist()
    if v==0.0:
        velocity_message.angular.z=theta
        rospy.loginfo("Gazebo rotates")
        velocity_publisher.publish(velocity_message)   
        time.sleep(0.03)

    else:
        velocity_message.linear.x=v
        rospy.loginfo("Gazebo moves")
        loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
        velocity_publisher.publish(velocity_message)
        
        loop_rate.sleep()


#rospy.init_node('turtlesim_motion_pose', anonymous=True)

# Running the loop for the detection of hand
while(cap.isOpened()):
    
    #if there is an error comes or does not detect anything this will try error statement
    ret, frame = cap.read()
    frame=cv2.flip(frame,1)
    if ret==True:
        pass
    #making the frame of specific dimension
    #frame =frame[450:1000,500:1180]

    #now using hsv we detect the color of skin
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #defining the range of skin color in hsv
    lower_skin = np.array([0, 58, 30], dtype = "uint8")
    upper_skin = np.array([33, 255, 255], dtype = "uint8")
    
    #applying mask to extract skin color object from the frame        
    mask = cv2.inRange(hsv, lower_skin, upper_skin)
    #now we dilate our skin color object to remove black spots or noise from it
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.dilate(mask,kernel,iterations = 3)
    #now we blur the image to smoothen edges in it 
    blur = cv2.bilateralFilter(mask,9,200,200)
    #blur = cv2.GaussianBlur(blur,(5,5),100) 
    
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= blur)
    #now converting the image into BGR -> GRAY 
    frame_gray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
    #thresholding the image
    ret, thresh = cv2.threshold(frame_gray, 98, 255,cv2.THRESH_TRUNC)
    
    #finding contours in the threshold image
    contours,_ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #selecting the contour of max area 
    cnt = max(contours, key = lambda x: cv2.contourArea(x))
    
    #making the convex hull around the hand 
    hull = cv2.convexHull(cnt)
    #finding the area of hull
    hullarea = cv2.contourArea(hull)
    #finding the area of max contour
    cntarea = cv2.contourArea(cnt)
    #making a rectangle around the hand
    x,y,w,h = cv2.boundingRect(hull)
    frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
    #finding the difference between the hull area and contour area
    reduction = (hullarea-cntarea)
    #taking the ratio of sum of hull area + contour area to the reduction
    ratio=(hullarea+cntarea)/reduction
    print("ratio:",ratio)
    
    #first check if contours are detected or not and find convexity defects in it
    
    # Now we are using no. of defects in the image and the ratio to classify the different
    # hand gestures and display the result 
    #cv2.imshow('frame',frame)
    img = cv2.drawContours(frame, hull, -2, (0,0,255), 10)
    
    try:
        if len(contours) > 0:
            hull = cv2.convexHull(cnt, returnPoints=False)
            defects = cv2.convexityDefects(cnt, hull)
            n = 0
                    
            #now we run the loop to find the angles of defects which for a triangle and 
            #avoid those with angles less than 90 degree
            for i in range(defects.shape[0]):
                    s,e,f,d = defects[i,0]
                    first = tuple(cnt[s][0])
                    second = tuple(cnt[e][0])
                    far = tuple(cnt[f][0])
                    cv2.line(img,first,second,[255,0,0],2)
                    cv2.circle(img,far,5,[0,0,255],-1)
                    
                    #now finding the sides of triangle formed by first,second and far point
                    a = math.sqrt((second[0] - first[0])**2 + (second[1] - first[1])**2)
                    b = math.sqrt((far[0] - first[0])**2 + (far[1] - first[1])**2)
                    c = math.sqrt((second[0] - far[0])**2 + (second[1] - far[1])**2)
                    
                    #now we find the angles between the sides of triangle
                    angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 57.295
                    if angle <= 90 :
                            n += 1
        
    except:
        pass
    print(n)
    font = cv2.FONT_HERSHEY_SIMPLEX
    try:
        if ratio<7 and n==1:
            move = 1
        elif  ratio>30:
            move = 2
        elif ratio<7 and n==4:
            move = 3
            
        elif ratio>13 and ratio<23:
            move = 4
            
        elif ratio>7 and ratio<17:
            move = 5
    except:
        cv2.putText(frame,'Try Again',(0,50), font, 2, (0,0,255), 4, cv2.LINE_AA)

    # now calling the motion function corresponding to the particular gesture
    while move==1:
        rospy.init_node('turtlesim_motion_pose', anonymous=True)
        cv2.putText(frame,'Forward',(0,50), font, 2, (0,0,255), 4, cv2.LINE_AA)
        motion (0.2,0.0)
        time.sleep(0.00000000005)
        break
    while move==2:
        rospy.init_node('turtlesim_motion_pose', anonymous=True)
        cv2.putText(frame,'Stop',(0,50), font,2, (0,0,255), 4, cv2.LINE_AA)
        motion (0.0,0.0)
        time.sleep(0.00000000005)
        break
    while move==3:
        rospy.init_node('turtlesim_motion_pose', anonymous=True)
        cv2.putText(frame,'Backward',(0,50), font,2, (0,0,255), 4, cv2.LINE_AA)
        motion (-0.2,0.0)
        time.sleep(0.00000000005)
        break
    while move==4:
        rospy.init_node('turtlesim_motion_pose', anonymous=True)
        cv2.putText(frame,'Right',(0,50), font, 2, (0,0,255), 4, cv2.LINE_AA)
        motion (0.0,-0.8)
        time.sleep(0.00000000005)
        break
    while move==5:
        rospy.init_node('turtlesim_motion_pose', anonymous=True)
        cv2.putText(frame,'Left',(0,50), font, 2, (0,0,255), 4, cv2.LINE_AA)
        motion (0.0,0.8)
        time.sleep(0.00000000005)
        break
    
    # displaying the frame
    cv2.imshow("frame",frame)
    if cv2.waitKey(1) & 0xFF ==ord('q'):
        break
#out.release()
#releasing the capture
cap.release()
# Destroying all the windows
cv2.destroyAllWindows()

    