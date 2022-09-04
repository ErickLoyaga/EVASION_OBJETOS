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
giro = 0
detectado = 0
final = 0
counter = 0
i=0
rawCapture= []
the_connection = []
activador= 0
key = 0
mision = 0
seguridad = 0

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
        #Pasamos a modo loiter
        mode_id= 6
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
    global i
    global namefile
    msg0=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
    #msg=the_connection.messages['LOCAL_POSITION_NED']
    msg=msg0
    msg0=the_connection.recv_match(type="ATTITUDE",blocking=True)
    giro=round(math.degrees(msg0.yaw))
    distancia = lidar_distance()
    i+=1
    DATOS=[i,distancia,msg.x,msg.y,msg.vy,giro]
    with open(namefile,"a",newline="") as f:
        wo=writer(f)
        wo.writerow(DATOS)
        f.close()
        return

def lidar_distance():
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
    
    time.sleep(0.01) # you can remove this for ultra-fast measurements!
    return distancia

def get_area(image):
    global rawCapture
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

    Bajo = np.array([hc-5, sc-30, vc-0.7], np.uint8)
    Alto = np.array([hc+5, sc+30, vc+0.7], np.uint8)
    mask = cv2.inRange(frameHSV,Bajo, Alto)
    #maskRedvis = cv2.bitwise_and(frame, frame, mask= mask)
    mask = cv2.Canny(mask, 20, 70)
    # print(Kernel) # Verificar
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
        #cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        bbox = (x,y,w,h)

        global detectado
        detectado= 1

        posicion = (W2)-cx

        return bbox,posicion
 
    except:
        error()
        print ('----ERROR---')
        rawCapture.truncate(0)
        bbox =[]
        posicion= 0
        return bbox,posicion
         
               
def evasion():
    global giro
    global final
    global the_connection
    global camera
    global activador
    global mision
    
    mision = 1
    #conectamos con el mission planner
    the_connection = mavutil.mavlink_connection('/dev/serial0',baud=57600)
    the_connection.wait_heartbeat()
    
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 30, 100000, 0, 0, 0, 0, 0)        #CON ESTE COMANDO SE REQUIERE LA INFORMACION DE ALGUN MENSAJE CON FRECUENCIA EN US #245:EXTENDED_SYS_STATUS
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 245, 100000, 0, 0, 0, 0, 0)
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 32, 100000, 0, 0, 0, 0, 0)
    
    # ARMAMOS
    # Once connected, use 'the_connection' to get and send messages
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print()
    print(msg)
    time.sleep(4)
    # MANDAMOS A REALIZAR LA MISION PROGRAMADA EN EL MISSION
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 1,0,0,0,0,0)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print()
    print(msg)
    
    input("Presion enter para comenzar:")
    
    while(True):

        if mision == 0:
            # MANDAMOS A REALIZAR LA MISION PROGRAMADA EN EL MISSION
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                                 mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 1,0,0,0,0,0)
            msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
            print()
            print(msg)
            mision = 1
        
        distancia = lidar_distance()
        print("distancia",distancia)
        registrar()
        
        if distancia > 0 and distancia <= 400:
            print("distancia",distancia)
            #Pasamos a modo guiado
            mode_id= 4
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
            the_connection.set_mode(mode_id)
            msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
            print()
            print(msg)
            #Recuperamos el yaw 
            msg=the_connection.recv_match(type="ATTITUDE",blocking=True)
            #print(math.degrees(msg.yaw))
            giro = round(math.degrees(msg.yaw))
            print('yaw',giro)
            #LOITER GUIADO
            the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                                    the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b101111000111), 0, 0, 0, -0.1, 0, 0, 0, 0, 0, 0, 0))
            #time.sleep(4)
            inicio_camara()
            
        #PARA ACABAR LA MISION
        msg = the_connection.recv_match(type='EXTENDED_SYS_STATE', blocking=True) 
        print(msg.landed_state) #landed_state:(0: undefined)(1:landed on ground)(2:MAV is in air)(3:MAV currently taking off)(4:MAV currently landing)
        
        if(msg.landed_state==4):
            break
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
    global detectado
    global activador
    global key
    global mision
    global seguridad
    iniciar = 0

    time.sleep(0.01)
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        img = frame.array
        dst=cv2.undistort(img,mtx,dist)
        distancia = lidar_distance()
        registrar()
        print("giro",giro)
        print("distancia",distancia)
        
        if detectado == 0:
            dst = cv2.resize(dst,(480,288))
            bbox,posicion= get_area(dst)
            print("detectado", detectado)
            print("bbox",bbox)
            print("posicion",posicion)

        if detectado == 1 and iniciar == 0:
            tracker = cv2.TrackerKCF_create()
            ok = tracker.init(dst,bbox)
            iniciar = 1
            print("Iniciar",iniciar)

        if iniciar == 1:
            dst = cv2.resize(dst,(480,288))
            ok,bbox = tracker.update(dst)
            if ok and distancia > 0 and distancia < 400:
                (x,y,w,h)=[int(v) for v in bbox]
                cv2.rectangle(dst,(x,y),(x+w,y+h),(0,255,0),2,1)
                print("bbox",bbox)
                cv2.imshow("camara",dst)
                key = cv2.waitKey(1) & 0xFF
                # clear the stream in preparation for the next frame
                rawCapture.truncate(0)
                if posicion < 0:
                    #Movemos a la derecha
                    seguridad = 1
                    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b101111000111), 0, 0, 0, 0, 0.3, 0, 0, 0, 0, 0, 0)) #CONTROL DE MOVIMIENTO CON VELOCIDAD, AQUI NO SE CAMBIA EL SENTIDO (YAW)
                if posicion > 0:
                    #Movemos a la izquierda
                    seguridad = 2
                    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b101111000111), 0, 0, 0, 0, -0.3, 0, 0, 0, 0, 0, 0)) #CONTROL DE MOVIMIENTO CON VELOCIDAD, AQUI NO SE CAMBIA EL SENTIDO (YAW)
 
            else:
                cv2.putText(dst,'Error',(100,0),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
                if seguridad == 1:
                    #Movemos a la derecha
                    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b101111000111), 0, 0, 0, 0, 0.3, 0, 0, 0, 0, 0, 0)) #CONTROL DE MOVIMIENTO CON VELOCIDAD, AQUI NO SE CAMBIA EL SENTIDO (YAW)
                if seguridad == 2:
                    #Movemos a la izquierda
                    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b101111000111), 0, 0, 0, 0, -0.3, 0, 0, 0, 0, 0, 0)) #CONTROL DE MOVIMIENTO CON VELOCIDAD, AQUI NO SE CAMBIA EL SENTIDO (YAW)
                detectado=0
                iniciar = 0
                final = 1
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            cv2.destroyAllWindows()
            detectado=0
            iniciar = 0
            activador = 1
            final =1
            seguridad = 0

        if final == 1:
            cv2.destroyAllWindows()
            final = 0
            mision = 0
            seguridad = 0
            rawCapture.truncate(0)
            break
       
        


#-------------------INICIO DEL CODIGO--------------------#
if __name__ == "__main__":
    #CSV
    now=datetime.now()
    day=now.strftime("%d_%m_%Hh%Mm")
    namefile="/home/pi/TIC_LOYAGA_ERICK/pruebas/datos1_"+day+".csv"
    lista=["tiempo","distancia_lidar","x","y","velocidad_y","yaw"]
    df=pd.DataFrame(columns=lista)
    df.to_csv(namefile, index=False) #index=False para eliminar la columna unnamed:0 que se crea
    
    #CREAMOS OBJETO PARA LA CAMARA
    tracker = cv2.TrackerKCF_create()
    camera = PiCamera()
    camera.resolution = (480, 288)
    camera.framerate = 30
    rawCapture = PiRGBArray(camera, size=(480, 288))

    camera.start_preview()
    # allow the camera to warmup
    time.sleep(0.01)

    # llamamos evasion
    evasion()


