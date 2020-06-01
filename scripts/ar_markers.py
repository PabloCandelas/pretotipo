"""
This program generates the marker that will be detected and followed in future programs.
As an output the marker is saved as a png document("marker33.png") in the same directory.

Author:Pablo Candelas 01/jun/2020
"""
# import libraries
import cv2 as cv
import numpy as np

# Load the predefined dictionary
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

# Generate the 33rth marker from the aruco library
markerImage = np.zeros((200, 200), dtype=np.uint8)
markerImage = cv.aruco.drawMarker(dictionary, 33, 200, markerImage, 1);

# Generate de document
cv.imwrite("marker33.png", markerImage);

# Farewell msg
print("\n\n\tBYEEE\n\n")
