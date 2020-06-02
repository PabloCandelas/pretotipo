#!/usr/bin/env python

"""
This program detects the markers from aruco library.
The input is:
    *The video from the webcam (you must have your webcam enabled)
The outputs are:
    *The center of the marker in the video (pixels)
    *The radius of the circumscribed circle around the marker (pixels) 
    
Author:Pablo Candelas 01/june/2020
"""

# libraries
import cv2
import numpy as np

# Constants
DEBUG = True 
AXISX = 0
AXISY=1

# Functions
def esquinas(markerCorners,corners,axis):
    """This function receives the array of arrays "markerCorners" and the "axis" and it returns the array "corners" 
    only with the values from the desired axis from "markerCorners". """
    for i in range(0,4):
        corners[i] = markerCorners[0][0][i][axis]

#Main program
if __name__ == '__main__':
    # Recive the incoming video
    print('Press "q" to quit')
    capture = cv2.VideoCapture(0)
    if capture.isOpened():  # try to get the first frame
        frame_captured, frame = capture.read()
    else:
        frame_captured = False
        
    # Meanwhile the video is active
    while frame_captured:
    
        # DETECT THE MARKER
        # Load the dictionary that was used to generate the markers.
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Initialize the detector parameters using default values
        parameters =  cv2.aruco.DetectorParameters_create()
        # Detect the markers in the image
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)   
          
        # If there was at least one marker detected
        if markerIds >0:
            # Draw a rectangle around the detected marker
            cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)  
                  
            # Computing the center and radius
            xcorners = [0,0,0,0]
            esquinas(markerCorners,xcorners,AXISX)
            ycorners = [0,0,0,0]
            esquinas(markerCorners,ycorners,AXISY) 
            """ To calculate the center we know that the marker has a quadrangular shape
            # x coordinates will be the avarage of the minimun value and the maximun value of the corners in the "x" axis
            # y coordinates will be the avarage of the minimun value and the maximun value of the corners in the "y" axis """
            center =int((min(esqx) + max(esqx))/2) ,int((min(esqy) + max(esqy))/2)
            # The radious is computed used the distance from the center of the marker to any of its corners
            radius = np.sqrt((esqx[0]-center[0])*(esqx[0]-center[0]) + (esqy[0]-center[1])*(esqy[0]-center[1]))
            # Print the values of the variables "center" and "radius" 
            if DEBUG: 
                print("----------")
                print("center:")
                print(center) 
                print("radius:")  
                print(radius)  
                
            # Draw the circle thar circumscribes the marker
            cv2.circle(frame, (int(center[0]), int(center[1])), int(radius),
            (0, 128, 255), 2)
            # Draw the center with a small circle
            cv2.circle(frame, center, 5, (255, 255, 0), -1)     
            
        # Show the resulting image/video 
        cv2.imshow('Test Frame', frame)
        
        # Exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # Retake the video capture
        frame_captured, frame = capture.read()

    # When everything done, release the capture
    capture.release()
    cv2.destroyAllWindows() 
    
    # Farewell msg
    print("\n\n\tBYEEE\n\n")
