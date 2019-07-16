##skeleton 

import cv2
import numpy as np
import math
import sys
import RPi.GPIO as GPIO
from time import sleep
count = 1
        
# Pins for Motor Driver Inputs 
#Motor1A = 24
#Motor1B = 23
#Motor1E = 25
#Motor2A = 20  #pin no baki ahe 
#Motor2B = 21
#Motor2E = 16
#initialisation of motor

GPIO.setmode(GPIO.BOARD)				# GPIO Numbering
#GPIO.setup(Motor1A,GPIO.OUT)  # All pins as Outputs
#GPIO.setup(Motor1B,GPIO.OUT)
#GPIO.setup(Motor1E,GPIO.OUT)
#GPIO.setup(Motor2A,GPIO.OUT)  # All pins as Outputs
#GPIO.setup(Motor2B,GPIO.OUT)
#GPIO.setup(Motor2E,GPIO.OUT)
GPIO.setwarnings (False)
#setting the GPIO pin as Output for motor input 
GPIO.setup (37, GPIO.OUT)
GPIO.setup (35, GPIO.OUT)
GPIO.setup (36, GPIO.OUT)
GPIO.setup (38, GPIO.OUT)

def forward():
    print("Move Forward !")
    cv2.imshow('frame',frame)
    #set the GPIO pins of raspberry pi.
    GPIO.setmode (GPIO.BOARD)

    #GPIO.PWM( pin, frequency ) it generates software PWM
    left= GPIO.PWM(37, 70)
    right= GPIO.PWM(35, 70)
    left.start(0)
    right.start(0)
    GPIO.output(37,GPIO.HIGH)
    GPIO.output(38,GPIO.HIGH)
    GPIO.output(35,GPIO.HIGH)
    GPIO.output(36,GPIO.HIGH)
    sleep(2)
    GPIO.output(37,GPIO.LOW)
    GPIO.output(38,GPIO.LOW)
    GPIO.output(35,GPIO.LOW)
    GPIO.output(36,GPIO.LOW)
    cv2.imshow('frame',frame)

def right():
    print("Move Right !")
    cv2.imshow('frame',frame)
    #set the GPIO pins of raspberry pi.
    GPIO.setmode (GPIO.BOARD)
    #GPIO.setwarnings (False)
    #enable

    #setting the GPIO pin as Output for motor input 
    GPIO.setup (37, GPIO.OUT)
    GPIO.setup (35, GPIO.OUT)
    GPIO.setup (36, GPIO.OUT)
    GPIO.setup (38, GPIO.OUT)


    #GPIO.PWM( pin, frequency ) it generates software PWM
    left= GPIO.PWM(37, 70)


    left.start(0)

    GPIO.output(37,GPIO.HIGH)
    GPIO.output(38,GPIO.HIGH)

    sleep(0)
    GPIO.output(37,GPIO.LOW)
    GPIO.output(38,GPIO.LOW)
    cv2.imshow('frame',frame)




    

cap = cv2.VideoCapture(0) #captures the video
     
while(1):
        
    try:  #an error comes if it does not find anything in window as it cannot find contour of max area
          #therefore this try error statement
          
        ret, frame = cap.read()       #read the frame
        frame=cv2.flip(frame,1)      #morphological transformations
        kernel = np.ones((3,3),np.uint8)   
         				   #matrix of ones 
        #define region of interest
        roi=frame[100:300, 100:300]
        
        
        cv2.rectangle(frame,(100,100),(300,300),(0,255,0),0)    #roi is rectangle since we are considering our palm as roi
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV) 		
        
        
         
    # define range of skin color in HSV  
        lower_skin = np.array([0,20,70], dtype=np.uint8) 		#skin_color=1
        upper_skin = np.array([20,255,255], dtype=np.uint8)
        
     #extract skin colur image
        mask = cv2.inRange(hsv, lower_skin, upper_skin)
        
   
        
    #extrapolate the hand to fill dark spots within
        mask = cv2.dilate(mask,kernel,iterations = 4)
        
    #blur the image
        mask = cv2.GaussianBlur(mask,(5,5),100) 
        
        
        
    #find contours
        _,contours,hierarchy= cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
   #find contour of max area(hand) to remove noise aroud
        cnt = max(contours, key = lambda x: cv2.contourArea(x))
        
    #approx the contour a little
        epsilon = 0.0005*cv2.arcLength(cnt,True)
        approx= cv2.approxPolyDP(cnt,epsilon,True)
       
        
    #make convex hull around hand
        hull = cv2.convexHull(cnt)
        
     #define area of hull and area of hand
        areahull = cv2.contourArea(hull)
        areacnt = cv2.contourArea(cnt)#area of hand
        print "Area of contour:",(areacnt)
        print "Area of hull:", (areahull)
      
    #find the percentage of area not covered by hand in convex hull
        arearatio=((areahull-areacnt)/areacnt)*100
    
     #find the defects in convex hull with respect to hand , defects are the region that are not covered by my hand in the convex hull
        hull = cv2.convexHull(approx, returnPoints=False)
        defects = cv2.convexityDefects(approx, hull)
        
    # l = no. of defects
        l=0
        
    #code for finding no. of defects due to fingers
        for i in range(defects.shape[0]):
            s,e,f,d = defects[i,0]
            start = tuple(approx[s][0])
            end = tuple(approx[e][0])
            far = tuple(approx[f][0])
            pt= (100,180)
            
            
            # find length of all sides of triangle
            a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
            b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
            c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
            s = (a+b+c)/2
            ar = math.sqrt(s*(s-a)*(s-b)*(s-c))
            
            #distance between point and convex hull
            d=(2*ar)/a
            
            # apply cosine rule here
            angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 57
            
        
            # ignore angles > 90 and ignore points very close to convex hull(they generally come due to noise)
            if angle <= 90 and d>30:
                l += 1
                cv2.circle(roi, far, 3, [255,0,0], -1)
            
            #draw lines around hand
            cv2.line(roi,start, end, [0,255,0], 2)
            
            
        l+=0   #no of defects+1
        
        #print corresponding gestures which are in their ranges
        font = cv2.FONT_HERSHEY_SIMPLEX
        if l==1:
            if areacnt<2000:
                cv2.putText(frame,'Put hand in the box',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
                
            else:
                if arearatio<12:
                    cv2.putText(frame,'0',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
                elif arearatio<17.5:
                    cv2.putText(frame,'3',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
                    right()
                   
                else:
                    
                    cv2.putText(frame,'1',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
                    forward()
                       
                    
        elif l==2:
            
            cv2.putText(frame,'2',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
            #right()
            
        elif l==3:
         
              if arearatio<27:
                    
                    cv2.putText(frame,'3',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
                    #left()
              else:
                    
                    cv2.putText(frame,'ok',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
                    #right()
                    
        elif l==4:
            
            cv2.putText(frame,'4',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
            #right()
            
        elif l==5:
            cv2.putText(frame,'5',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
           
                    
        else :
            cv2.putText(frame,'Put hand in the box',(10,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
            
        #show the windows
        cv2.imshow('mask',mask)
        cv2.imshow('frame',frame)
    except:
        pass
        
    
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    
cv2.destroyAllWindows()
cap.release()
print("HELLO !")

