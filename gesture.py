#!/usr/bin/env python3

import rospy 

from std_msgs.msg import UInt8

import subprocess

import numpy as np

import cv2 as cv

import time

import math

count=0;


if __name__=='__main__':

    try:
    
    
        rospy.init_node('gesture')
        
        pub = rospy.Publisher('/finger_count',UInt8, queue_size=10)
        
        rate_loop = rospy.Rate(1)  #wait 10 seconds
        
        while not rospy.is_shutdown():
            
            #cmd = "raspistill --width 1024 --height 768 -o pic.jpg"
            cmd = "raspistill --width 200 --height 200 -o pic.jpg"
            subprocess.call(cmd, shell=True)
            t1 = time.perf_counter()
            img = cv.imread('pic.jpg')
            print("Image Properties")
            print("- Number of Pixels: " + str(img.size))
            print("- Shape Dimensions: " + str(img.shape))

            #dim = (768,1024)
            #img = cv.resize(img,dim)
            #cv.imwrite('scaledimg.jpg',img)
            
            b,g,r = cv.split(img)
            
            #CONVERSION FROM RGB TO YCBCR
            convrtd = cv.cvtColor(img, cv.COLOR_BGR2YCrCb)
            #cv.imshow('YCbCr', convrtd)

            Y, Cr, Cb = cv.split(convrtd)
            
            # SKIN PIXEL DETECTION
            for b in range(img.shape[0]):
                for a in range(img.shape[1]):
                    if Y[b][a]>0 and Y[b][a]<234 and Cb[b][a]>=77 and Cb[b][a]<=127 and Cr[b][a]>=133 and Cr[b][a]<=173:
			        #if Cb[b][a]>=100 and Cb[b][a]<=150 and Cr[b][a]>=150 and Cr[b][a]<=200:
                        convrtd[b][a][0]=255
                        convrtd[b][a][1]=255
                        convrtd[b][a][2]=255
                    else: 
                        convrtd[b][a][0]=0
                        convrtd[b][a][1]=0
                        convrtd[b][a][2]=0
				        
 		    #MORPHOLOGICAL FILTERING TO REMOVE NOISE	

            #structel = np.ones((2,2),np.uint8) #structural element #square Structural Element

            structel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(6,6)) #Defining Structural Element for   Opening
            opened = cv.morphologyEx(convrtd, cv.MORPH_OPEN, structel) #opening operation
            #cv.imshow('After opening',opened)

            structel2 = cv.getStructuringElement(cv.MORPH_ELLIPSE,(10,10)) #Defining a bigger structural element for Closing
            closeaftropen = cv.morphologyEx(opened, cv.MORPH_CLOSE, structel2) #closing operation
            #cv.imshow('Closing After Opening',closeaftropen)

            #FINDING THE CONTOURS OF THE BLOB
            img2 = img
            closeaftropen = cv.cvtColor(closeaftropen, cv.COLOR_BGR2GRAY)

            contours, hierarchy = cv.findContours(closeaftropen, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
            lngth = len(contours)
            print(str(lngth)+" contours containing skin-pixels have been identified")
            if (lngth == 0) :
                finger_num = 0
            
            else: 
            #FINDING THE PERIMETER OF LARGEST CONTOUR 
                arcs = []
                for i in range(lngth):
                    arc = cv.arcLength(contours[i],True)
                    arcs.append(arc)

                c = max(arcs)
                indx = arcs.index(c)
                print("Contour at position"+str(indx)+"has longest length of"+str(c))
                #cont_len_thresh = 1000  #Might need to be adjusted
                cont_len_thresh = 200
                if (c<= cont_len_thresh):
                    finger_num = 0
                    print("Contour length is lesser than the threshold for a viable hand gesture- No hand gesture detected")
                
                else:
            
            
                    #cv.drawContours(img2, contours, -1, (0,0,255), 3)
                    #cv.imshow('Contour',img2)
                    #cv.waitKey(3000)
                    #cv.destroyAllWindows()
            
                #FINDING THE IMAGE CENTROID

                    M = cv.moments(contours[indx])
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    print(str(cX) + " is the x coordinate of hand centre") 
                    print(str(cY) + " is the Y coordinate of hand centre") 
            
                    dist=[]
                    cont = contours[indx] #longest skin-pixel-containing contour
                    num_points=len(cont)
                    print(num_points)
                    #print(cont)
                    maxd = 0
            
            #FINDING DISTANCE TO THE FARTHEST POINT ON CONTOUR FROM THE CENTER OF THE HAND
                    for p in range(num_points):
	
                        #dist.append((cX-cont[p][0][0])**2+(cY-cont[p][0][1])**2)
                        dist = np.sqrt((cX-cont[p][0][0])**2+(cY-cont[p][0][1])**2)
                        if (dist>maxd):
                            maxd = dist
                            max_x = cont[p][0][0]
                            max_y = cont[p][0][1]
                    #dist2 = np.sqrt(dist)
                    #maxd = max(dist2)
                    print(str(maxd)+"is the longest distance from center of hand to periphery")
                    #print(dist2) #the array containing distances from center to contour
                    print(str(max_x)+ " is the x pixel coordinate at maximum distance from centroid and " +str(max_y)+ " is the y coordinate")
                    diff_x = cX-max_x
                    diff_y = cY-max_y
                    ang = math.atan2(diff_y,diff_x)
                    ang_deg = math.degrees(ang)
                    print(str(ang_deg) + " is the orientation angle of the ellipse (in degrees).") 

                    x = img.shape[0]
                    y = img.shape[1]

            #SHIFTING FILLED HAND CONTOUR TO A BLACK IMAGE BACKGROUND
                    black_backgrnd = np.zeros((x,y,3), dtype="uint8")
                    cv.drawContours(black_backgrnd, [contours[indx]], -1, (255,255,255), thickness=-1)
            
            #CREATING A CIRCULAR MASK
                    mask = np.zeros((x,y,3), dtype="uint8")
                    #cv.circle(mask,(cX, cY),int(0.65*maxd),(255,255,255),4)
                    cv.ellipse(mask,(cX, cY),(int(0.65*maxd),int(1.6*0.32*maxd)),ang_deg,0.0,360.0,(255,255,255),2)
                    finger_count = cv.bitwise_and(black_backgrnd,mask)
            
            #COUNTING THE LINE SEGMENTS TO GET THE NUMBER OF FINGERS
                    finger_count = cv.cvtColor(finger_count, cv.COLOR_BGR2GRAY)
                    contours2, hierarchy2 = cv.findContours(finger_count, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
                    finger_num = len(contours2)
                    finger_num = finger_num-1 #Subtracting the segment detected for Wrist Section
            
            count = count + 1
            str1 = str(finger_num)+" Fingers were detected in Image Number: " + str(count)
            t2 = time.perf_counter()
            t = t2-t1
            print("It took " + str(t) + "seconds in recognizing the gesture.")
            rospy.loginfo(str1)
            pub.publish(finger_num)
            rate_loop.sleep()
            
    except rospy.ROSInterruptException:
        pass
