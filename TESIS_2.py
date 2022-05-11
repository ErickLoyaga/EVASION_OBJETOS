import numpy as np
import cv2
import time
import math
import sys
#from trackereu import *
#import board
#import busio
#import adafruit_lidarlite
cap = cv2.VideoCapture(2)
Kernel= np.ones((9,9), np.uint8)
detectado = 0
counter = 0
bbox =[]

# def evasion():
#     camara()
#     return
def inicio():
    while True:
        evasion()
        if detectado == 2:
            sys.exit("termino elproceso")  
    return
 
def deteccion(image): # get_area()
    W1 = image.shape[1]
    W2 = int(W1*0.5)
    H1 = image.shape[0]
    H2 = int(H1*0.5)

    frameHSV= cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h=[]
    s=[]
    v=[]
    # Obtener region en punto central de la imagen
    for i in range(20):
        for j in range(20):
            a=frameHSV[int(H2)-10+i][int(W2)-10+j]
            h.append(a[0])
            s.append(a[1])
            v.append(a[2])

    hc,sc,vc=np.mean(h),np.mean(s),np.mean(v)
    # print("hm :",hc," sm :",sc," vm: ",vc)
    h.clear()
    s.clear()
    v.clear()

    Bajo = np.array([hc-5, sc-15, vc-25], np.uint8)
    Alto = np.array([hc+5, sc+15, vc+25], np.uint8)
    mask = cv2.inRange(frameHSV,Bajo, Alto)
    #maskRedvis = cv2.bitwise_and(frame, frame, mask= mask)
    mask = cv2.Canny(mask, 20, 50)
    print(Kernel) # Verificar
    mask= cv2.dilate(mask, Kernel, iterations=1)
    #cv2.imshow("erosion1",mask)

    (contornos,jerarquia) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(image,contornos,-1,(0,0,255), 2

    distancias=[]
    index=[]
    #detection=[]
    for c in range(len(contornos)):
        cnt= contornos[c]

        if cv2.contourArea(contornos[c]) < 1500: 
            continue

        #Calculamos el centroide del contorno
        M = cv2.moments(cnt)
        cx = int(M["m10"]/M["m00"])
        cy = int(M["m01"]/M["m00"])
        #cv2.circle(image,(cx,cy),5,(0,0,255),-1)
        distancias.append(math.dist([W2,H2],[cx,cy]))
        index.append(c)

        #print("distancia", distancias)
        #print("index", index)
    try:
        minimo=min(distancias)
        #print ("minimo2",minimo)
        ind=distancias.index(minimo)
        #print("ind",ind)
        cnt=contornos[index[ind]]
        #Calculamos el centroide del contorno
        distancias.clear()
        index.clear()
        M = cv2.moments(cnt)
        cx = int(M["m10"]/M["m00"])
        cy = int(M["m01"]/M["m00"])
        #cv2.circle(image,(cx,cy),5,(0,0,255),-1)

        # Obtenemos el bounds del contorno, el rectángulo mayor que engloba al contorn
        (x, y, w, h) = cv2.boundingRect(cnt)
        # Dibujamos el rectángulo del bounds
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        global bbox
        bbox = (x,y,w,h)

        global detectado
        detectado= 1
        posicion = (W2)-cx
        print("detectado",detectado)
    except:
        #cv2.imshow("image", image)
        print ('----ERROR---')
        # set_paro_emergencia() verle bien




def evasion():
    global detectado
    global counter
    # ok, image = cap.read()
    # if ok == True: 
    #   image=cv2.resize(image,(640,480))
    # else:
    #    counter +=1
    #    if counter == 10:
    #        print("Camara no disponible")
    #        set_paro_emergencia()
    #        exit()

    if detectado == 0:
        # Chequear sin while
        while detectado == 0:
            ok,image = cap.read()
            if not ok:
                break

            # counter, si = 10, sys quit ()
            image=cv2.resize(image,(640,480))
            deteccion(image)
            #print('valor',detectado)
    if detectado == 1: 
        tracker = cv2.TrackerKCF_create()
        ok,image =cap.read()
        # siempre if ok == True.
        image=cv2.resize(image,(640,480))
        global bbox
        ok = tracker.init(image,bbox)

        while detectado == 1:
            ok,image=cap.read()
            print('valor', detectado)
            if not ok:
                break
            image=cv2.resize(image,(640,480))
            ok,bbox=tracker.update(image)

            if ok:
                (x,y,w,h)=[int(v) for v in bbox]
                cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2,1)
            else:
                cv2.putText(image,'Error',(100,0),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
                detectado=0

            cv2.imshow('Tracking',image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                detectado = 2
                break
        cv2.destroyAllWindows()


def lidar_distance():
	# Create library object using our Bus I2C port
	i2c = busio.I2C(board.SCL, board.SDA)
	# Default configuration, with only i2c wires
	sensor = adafruit_lidarlite.LIDARLite(i2c)
	try:
	    # We print tuples so you can plot with Mu Plotter
	    distancia= sensor.distance
	    print(distancia)

	except RuntimeError as e:
	    # If we get a reading error, just print it and keep truckin'
	    print(e)
	    time.sleep(0.01) # you can remove this for ultra-fast measurements!

	return distancia
    
        
 
 
if __name__ == '__main__':
    inicio()
   