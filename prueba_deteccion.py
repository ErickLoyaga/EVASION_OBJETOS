from picamera.array import PiRGBArray
from picamera import PiCamera
from pymavlink import mavutil
from datetime import datetime
import time
import cv2
from csv import writer
import yaml
import pandas as pd
import numpy as np
import board
import busio
import adafruit_lidarlite
import math
import sys

namefile = 0
Kernel= np.ones((9,9), np.uint8)
distancia = 0
giro = 0
final = 0
counter = 0
i=0
rawCapture= []
the_connection = []
activador = 0

CAMERA_PARAMETERS_INPUT_FILE = "cam1.yaml"

with open(CAMERA_PARAMETERS_INPUT_FILE) as f:
    loadeddict = yaml.safe_load(f)
    mtx = loadeddict.get('camera_matrix')
    dist = loadeddict.get('dist_coeff')
    mtx = np.array(mtx)
    mtx_inv = np.linalg.inv(mtx)
    dist = np.array(dist)
    

def error():
    global counter
    global the_connection
    counter = counter+1
    if counter == 10:
        counter = 0
        #Pasamos a modo guiado
        mode_id= 5
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
        the_connection.set_mode(mode_id)
        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print()
        print(msg)
        sys.exit("Error de camara o laser")

def registrar(): #Registramos nuevas filas al archivo csv del data frame
    global the_connection
    global giro
    global distancia
    global i
    global namefile
    msg0=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
    msg=the_connection.messages['LOCAL_POSITION_NED']
    msg0=the_connection.recv_match(type="ATTITUDE",blocking=True)
    giro=round(math.degrees(msg0.yaw))
    i+=1
    DATOS=[i,distancia,msg.x,msg.y,msg.vy,giro]
    with open(namefile,"a",newline="") as f:
        wo=writer(f)
        wo.writerow(DATOS)
        f.close()
        return

def lidar_distance():
    global distancia
    # Create library object using our Bus I2C port
    i2c = busio.I2C(board.SCL, board.SDA)
    # Default configuration, with only i2c wires
    sensor = adafruit_lidarlite.LIDARLite(i2c)
    try:
        # We print tuples so you can plot with Mu Plotter
        distancia= sensor.distance
    
    except RuntimeError as e:
        # If we get a reading error, just print it and keep truckin'
        distancia = -1
    
    time.sleep(0.005) # you can remove this for ultra-fast measurements!
    return distancia

def deteccion(dst): # get_area()
    global final
    global rawCapture
    
    image = cv2.resize(dst,(480,288))
    W1 = image.shape[1]
    W2 = int(W1*0.5)
    H1 = image.shape[0]
    H2 = int(H1*0.5)

    frameHSV= cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h=[]
    s=[]
    v=[]
    # Obtener region en punto central de la imagen
    for i in range(60):
        for j in range(60):
            a=frameHSV[int(H2)-30+i][int(W2)-30+j]
            h.append(a[0])
            s.append(a[1])
            v.append(a[2])

    hc,sc,vc=np.mean(h),np.mean(s),np.mean(v)
    # print("hm :",hc," sm :",sc," vm: ",vc)
    h.clear()
    s.clear()
    v.clear()

    Bajo = np.array([hc-5, sc-30, vc-0.8], np.uint8)
    Alto = np.array([hc+5, sc+30, vc+0.8], np.uint8)
    mask = cv2.inRange(frameHSV,Bajo, Alto)
    #maskRedvis = cv2.bitwise_and(frame, frame, mask= mask)
    mask = cv2.Canny(mask, 20, 70)
    #print(Kernel) # Verificar
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

        # Obtenemos el bounds del contorno, el rectángulo mayor que engloba al contorn
        (x, y, w, h) = cv2.boundingRect(cnt)
        # Dibujamos el rectángulo del bounds
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        cv2.imshow("objeto",image)
        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            cv2.destroyAllWindows()
            final = 1
    except:
        #cv2.imshow("image", image)
        error()
        rawCapture.truncate(0)
        print ('----ERROR---')
        # set_paro_emergencia() verle bien
        
               
def evasion():
    global distancia
    global giro
    global final
    global the_connection
    global activador
    
    the_connection = mavutil.mavlink_connection('/dev/serial0',baud=57600)
    the_connection.wait_heartbeat()
    distancia = lidar_distance()
    #registrar()
    
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 30, 100000, 0, 0, 0, 0, 0)        #CON ESTE COMANDO SE REQUIERE LA INFORMACION DE ALGUN MENSAJE CON FRECUENCIA EN US #245:EXTENDED_SYS_STATUS
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 32, 100000, 0, 0, 0, 0, 0)        #CON ESTE COMANDO SE REQUIERE LA INFORMACION DE ALGUN MENSAJE CON FRECUENCIA EN US #245:EXTENDED_SYS_STATUS
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 245, 100000, 0, 0, 0, 0, 0)        #CON ESTE COMANDO SE REQUIERE LA INFORMACION DE ALGUN MENSAJE CON FRECUENCIA EN US #245:EXTENDED_SYS_STATUS

    while True:
        print("Inicio")
        distancia = lidar_distance()
        print("distancia",distancia)
        registrar()
        
        if distancia >= 0 and distancia <= 400:
            print("distancia",distancia)
            #Pasamos a modo guiado
            mode_id= 4
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
            the_connection.set_mode(mode_id)
            msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
            print()
            print(msg)
            #LOITER GUIADO
            the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                                    the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111111000), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            
            inicio_camara()
        
        if activador == 1:
            activador = 0
            mode_id = 6
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
            the_connection.set_mode(mode_id)
            msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
            print()
            print(msg)
            break
    
    


        
def inicio_camara():
    global rawCapture
    global final
    global giro
    global the_connection
    global distancia
    global activador
    
    time.sleep(0.01)
    # allow the camera to warmup
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        img = frame.array
        dst=cv2.undistort(img,mtx,dist)
        distancia = lidar_distance()
        registrar()
        deteccion(dst)
        
        print("distancia",distancia)
        
        if final == 1:
            final = 0
            activador = 1
            rawCapture.truncate(0)
            break


#-------------------INICIO DEL CODIGO--------------------#

#CREAMOS OBJETO PARA LA CAMARA
camera = PiCamera()
camera.resolution = (480, 288)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(480, 288))

camera.start_preview()
# allow the camera to warmup
time.sleep(0.001)

#CSV
now=datetime.now()
day=now.strftime("%d_%m_%Hh%Mm")
namefile="/home/pi/TIC_LOYAGA_ERICK/pruebas/datos1_"+day+".csv"

lista=["tiempo","distancia_lidar","posx","posy","velocidad_y","yaw"] 
df=pd.DataFrame(columns=lista)
df.to_csv(namefile, index=False) #index=False para eliminar la columna unnamed:0 que se crea 

# llamamos evasion 
evasion()