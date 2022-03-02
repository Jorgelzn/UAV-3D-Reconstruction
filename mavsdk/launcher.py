

import json
import time                         # To sleep a time period (for mutex to work OK)
from logger import *
import threading
import subprocess
import os
import asyncio
from missionsHandler import missionsHandler
import math

arrUAVs = {"a": "a"}
logFolder = os.path.dirname(__file__)

# Compilando PX4 y lanzando PX4+MavSDK (para evitar lanzarlo a mano también)
outputFile   = logFolder+"/px4_mavsdk_multiple.log"
if os.path.exists(outputFile):
    os.remove(outputFile)
else:
    if not os.path.isdir(logFolder):
        os.mkdir(logFolder)
command     = "~/AirSim/run/px4_mavsdk_multiple.sh "+str(len(arrUAVs))+" >> "+outputFile # Path donde se encuentra el fichero que lanza PX4+MavSDK (1 solo UAV)
bashCommand =["bash", "-c", command]                                                     # create the bash command
process     = subprocess.Popen(bashCommand,stdin=subprocess.PIPE,stdout=subprocess.PIPE) # Lanza pero no espera al resultado (porque al abrir MavSDK bloquea dicho proceso)

# Abrir el fichero de log de px4_mavsdk_multiple y esperar a encontrar el ALL OK
while True:
    try:
        with open(outputFile,'r') as f:
            fileContent = f.readlines()
            lastLine = fileContent[-1]
            f.close()	# After performing the operation, close the file
            if lastLine == "All running!\n":
                break
    except:
        print(outputFile+" file does not exist yet.")
    time.sleep(0.5)

# Crear enlaces simbolicos de las carpetas de instance_x para tenerlas accesibles desde logFolder?
for i in range(len(arrUAVs)):
    folderInstanceOr   = os.path.expanduser('~')+"/"+"PX4-Autopilot/build/px4_sitl_default/instance_"+str(i+1)+"/"
    folderInstanceDest = logFolder + "instance_"+str(i+1)
    if os.path.isdir(folderInstanceDest):	# If linked previously, remove that one and create it again
        os.unlink(folderInstanceDest)
    os.symlink(folderInstanceOr, folderInstanceDest)
# se abren los ficheros de px4 hasta encontrar los mensajes de Home Location en todos los drones
for i in range(len(arrUAVs)):
    while True:
        try:
            filePX4 = logFolder + "instance_"+str(i+1)+"/"+"out.log"
            with open(filePX4,'r') as f:
                fileContent = f.readlines()
                lastLine = fileContent[-1]
                f.close()	# After performing the operation, close the file
                if "INFO  [tone_alarm] home set\n" in fileContent:
                    break
        except:
            print(outputFile+" file does not exist yet.")
        time.sleep(0.5)

# Run the main process
#await missionsHandler(arrUAVs)
missionHandler = missionsHandler(arrUAVs)
missionHandler.name = "MissionHandler" #drone['General']['nameDrone']
missionHandler.start()