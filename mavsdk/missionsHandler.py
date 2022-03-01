#!/usr/bin/env python3

#############################################################
#                        IMPORTS                            #
#############################################################
from logger import *
from droneEightFigure import droneEightFigure
import os
import threading
import json
import asyncio

#############################################################
#                        MAIN                               #
#############################################################
class missionsHandler(threading.Thread):
    #######################################################################
    # Prints log messages
    def printLog(self, message="", typeMessage="info"):
        if typeMessage == "info":
            logging.info(message)
        elif typeMessage == "error":
            logging.error(message)

    #######################################################################
    # Mission handler Init
    def __init__(self, arrUAVs):
        threading.Thread.__init__(self)
        print("AAA")
        outputFile   = "mission_handler.log"
        config_root_logger(log_file=outputFile)
        print("BBBB")
        #self.printLog(message="Logger initialized", typeMessage="info")
        self.arrUAVs    = arrUAVs
        #self.channel_layer = get_channel_layer()
        #self.groupName = "uav"
        print("CCCC")


    def run(self):
        self.printLog(message="Run executed", typeMessage="info")
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.mission())
        self.loop.close()


    async def mission(self):
        print("EEE")
        self.printLog(message="AAAA", typeMessage="info")
        #message = {
        #    'type': 'test',
        #    'message': "event_trigered_from_views"
        #}
        #await self.channel_layer.group_send(self.groupName, message)
        #self.printLog(message="AAVVV", typeMessage="info")
        print("FFF")

        try:
            self.printLog(message="Starting mission handler", typeMessage="info")
            # initialize all Drone processes
            self.printLog(message="Creating control primitives", typeMessage="info")
            barrierArm       = threading.Barrier(len(self.arrUAVs)) # Barrier where all thread will wait to continue sincronized
            barrierMission   = threading.Barrier(len(self.arrUAVs)) # Barrier where all thread will wait to continue sincronized
            e                = threading.Event()               # Event used as signal where some threads will continue its execution
            lock             = threading.Lock()                # Lock to let each variable to securely access global variables
            self.threads     = []

            self.printLog(message="Creating each drone mission", typeMessage="info")
            for i, drone in enumerate(self.arrUAVs):
                # Se define la misión según la selección en la web
                """
                if drone['General']['missionDrone'] == "Eight figure":
                    print("Mission Eight figure")
                    droneName       = drone['General']['nameDrone']
                    camerasName = []
                    for j, cameraName in enumerate(drone['Cameras']):
                        camerasName.append(drone['Cameras'][cameraName]['General']['CameraName'])
                    FlightAltitude  = float( drone['Mission']['FlightAltitude'] )
                    Rate            = float( drone['Mission']['Rate'] )
                    Radius          = float( drone['Mission']['Radius'] )
                    SecondsPerCycle = float( drone['Mission']['SecondsPerCycle'] )
                    # Threading variables
                    #barrierArm       = threading.Barrier(len(arrUAVs)) # Barrier where all thread will wait to continue sincronized
                    #barrierMission   = threading.Barrier(len(arrUAVs)) # Barrier where all thread will wait to continue sincronized
                    #e             = threading.Event()           # Event used as signal where some threads will continue its execution
                    #lock          = threading.Lock()            # Lock to let each variable to securely access global variables
                """
                mission = droneEightFigure(drone_id=i+1, droneName="Drone1", camerasName=["front"], FLIGHT_ALTITUDE=30, RATE=10, RADIUS=20, CYCLE_S=5)
                #else:
                #    mission = drone_lazaro(drone_id=i+1, barrierArm=barrierArm, barrierMission=barrierMission, lock=lock, e=e)
                
                mission.name = "Drone"+str(i) #drone['General']['nameDrone']
                self.threads.append(mission)

            self.printLog(message="Starting missions: ", typeMessage="info")
            for mission in self.threads:
                mission.start()
                mission.join()
            self.printLog(message="Processes Joined: ", typeMessage="info")

        except Exception as e:
            self.printLog(typeMessage="error", message=PrintException())