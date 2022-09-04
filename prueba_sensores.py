from picamera.array import PiRGBArray
from picamera import PiCamera
from pymavlink import mavutil
from datetime import datetime

import adafruit_lidarlite
import time
import cv2
import csv
import yaml
import pandas as pd
import numpy as np
import board
import busio

import math
import sys

namefile = 0
Kernel= np.ones((9,9), np.uint8)
distancia = 0
giro = 0
i=0
activador = False
rawCapture= []
the_connection = []


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
    counter = counter+1
    if counter == 10:
        counter = 0
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
    giro=msg0.yaw
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
        print("distancia:",distancia)
    
    except RuntimeError as e:
        # If we get a reading error, just print it and keep truckin'
        distancia = -1
        print(e)
    
    time.sleep(0.005) # you can remove this for ultra-fast measurements!
    return distancia
               
def evasion():
    global distancia
    global giro
    global the_connection
    global camera

    #conectamos
    the_connection = mavutil.mavlink_connection('tcp:172.31.69.215:5762')
    the_connection.wait_heartbeat()

    #Pasamos a modo guiado
    mode_id=the_connection.mode_mapping()['GUIDED']
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                 mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
    the_connection.set_mode(mode_id)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print()
    print(msg)
    # ARMAMOS
    # Once connected, use 'the_connection' to get and send messages
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print()
    print(msg)

    # pedimos velocidad, posicion, yaw
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 30, 100000, 0, 0, 0, 0, 0)        #CON ESTE COMANDO SE REQUIERE LA INFORMACION DE ALGUN MENSAJE CON FRECUENCIA EN US #245:EXTENDED_SYS_STATUS
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 245, 100000, 0, 0, 0, 0, 0)
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 32, 100000, 0, 0, 0, 0, 0)

    msg = the_connection.recv_match(type="COMMAND_ACK",blocking=True)
    print()
    print(msg)
    
    while(True):
        distancia = lidar_distance()
        # MANDAMOS A REALIZAR LA MISION PROGRAMADA EN EL MISSION
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                             mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 1,0,0,0,0,0)
        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print()
        print(msg)
        
        msg=the_connection.recv_match(type="ATTITUDE",blocking=True)
        giro = round(math.degrees(msg.yaw))
        print('yaw',giro)
        time.sleep(0.01)

        registrar()
        inicio_camara()

        msg = the_connection.recv_match(type='EXTENDED_SYS_STATE', blocking=True) 
        print(msg.landed_state) #landed_state:(0: undefined)(1:landed on ground)(2:MAV is in air)(3:MAV currently taking off)(4:MAV currently landing)
        if(msg.landed_state==4):
            break

def inicio_camara():
    global rawCapture
    global giro
    global distancia


    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        img = frame.array
    
        dst=cv2.undistort(img,mtx,dist)
        distancia = lidar_distance()
        registrar()
        print("giro",giro)

        cv2.imshow("camara",dst)
        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            cv2.destroyAllWindows()
            break

#-------------------INICIO DEL CODIGO--------------------#

#CREAMOS OBJETO PARA LA CAMARA
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))

camera.start_preview()
# allow the camera to warmup
time.sleep(0.01)

#CSV
now=datetime.now()
day=now.strftime("%d_%m_%Hh%Mm")
namefile="/home/pi/TIC_LOYAGA_ERICK/pruebas/datos1_"+day+".csv"

lista=["tiempo","distancia_lidar","x","y","velocidad_y","yaw"] 
df=pd.DataFrame(columns=lista)
df.to_csv(namefile, index=False) #index=False para eliminar la columna unnamed:0 que se crea 

# llamamos evasion 
evasion()