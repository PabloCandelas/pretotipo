#!/usr/bin/env python
#librerias
import cv2
import numpy as np

#constantes
DEBUG = True 
EJEX = 0
EJEY=1

def esquinas(markerCorners,esq,eje):
    for i in range(0,4):
        esq[i] = markerCorners[0][0][i][eje]
#main
if __name__ == '__main__':
    #empezar a tomar video
    print('Press "q" to quit')
    capture = cv2.VideoCapture(0)
    if capture.isOpened():  # try to get the first frame
        frame_captured, frame = capture.read()
    else:
        frame_captured = False
    #mientras se reciba video
    while frame_captured:
        #DETECTAR MARCADOR
        #Load the dictionary that was used to generate the markers.
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Initialize the detector parameters using default values
        parameters =  cv2.aruco.DetectorParameters_create()
        # Detect the markers in the image
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)     
        #Si se detecto un marcador
        if markerIds >0:
            #dibujar cuadro alrededor
            cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)        
            #calculo de centro y radio
            esqx = [0,0,0,0]
            esquinas(markerCorners,esqx,EJEX)
            esqy = [0,0,0,0]
            esquinas(markerCorners,esqy,EJEY) 
            center =int((min(esqx) + max(esqx))/2) ,int((min(esqy) + max(esqy))/2)
            radius = np.sqrt((esqx[0]-center[0])*(esqx[0]-center[0]) + (esqy[0]-center[1])*(esqy[0]-center[1]))
            if DEBUG: 
                #print(esqx)
                #print(esqy)
                print("----------")
                print("center:")
                print(center) 
                print("radius:")  
                print(radius)  
             #Dibujar circulo y centro
            cv2.circle(frame, (int(center[0]), int(center[1])), int(radius),
            (0, 128, 255), 2)
            cv2.circle(frame, center, 5, (255, 255, 0), -1)     
            #mostrar en pantalla     
        cv2.imshow('Test Frame', frame)
        #exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        #retomar capture
        frame_captured, frame = capture.read()

    # When everything done, release the capture
    capture.release()
    cv2.destroyAllWindows()
    print("")
    print("  BYEEE   ")
    print("")
