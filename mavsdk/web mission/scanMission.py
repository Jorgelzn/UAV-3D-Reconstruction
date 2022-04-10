
#!/usr/bin/env python3

#############################
# author: damigo@inf.uc3m.es
# an UAV creates an eight figure using MavSDK Python Offboard mode
# This code is an Python port of https://gitlab.com/voxl-public/modal-pipe-architecture/voxl-vision-px4/-/blob/master/src/offboard_figure_eight.c
#############################

import asyncio
from importlib.resources import path
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, PositionGlobalYaw)
import math
import time

#############################################################
#                        IMPORTS                            #
#############################################################
import asyncio
from sys import argv
from time import sleep
from mavsdk import System
import os
from mavsdk.action import OrbitYawBehavior, ActionError
import threading
from logger import *
import asyncio
import json
from datetime import datetime
import numpy as np
import cv2

import airsim

import pprint
import tempfile
import os
import time

# Struct simulating a mavlink_set_position_target_local_ned_t
# it doesn't look necessary though
class Position():
  "Stores name and place pairs"
  def __init__(self):
    self.time_boot_ms     = 0
    self.coordinate_frame = 0
    self.type_mask        = 0
    self.type_mask        = 0
    self.target_system    = 0
    self.target_component = 0
    self.x                = 0
    self.y                = 0
    self.z                = 0
    self.vx               = 0
    self.vy               = 0
    self.vz               = 0
    self.afx              = 0
    self.afy              = 0
    self.afz              = 0
    self.yaw              = 0
    self.yaw_rate         = 0

#############################################################
#                        THREADS CLASS                      #
#############################################################

class scanMission(threading.Thread):

  # Status variables of this Drone
  connected   = False # info of this Drone

  # Variable to communicate with MavSDK-server
  drone        = None
  airsimClient = None

  # variables
  position   = None
  flightMode = None
  in_air     = None
  is_armed   = None
  posandvel  = None
  time       = None
  heading    = None
  quaternion = None
  lastTime   = None
  velocityNED   = None
  #imu        = None
  #rotation   = None
  #odometry   = None
  #gps_info   = None
  #rawImu     = None
  #scaledImu  = None
  

  arrPosition   = []
  arrFlightMode = []
  arrIn_air     = []
  arrIs_armed   = []
  arrPosandvel  = []
  arrTime       = []
  arrHeading    = []
  arrQuaternion = []
  #arrImu        = []
  #arrRotation   = []
  #arrOdometry   = []
  #arrGps_info   = []
  #arrVelocityNED= []
  #arrRawImu     = []
  #arrScaledImu  = []

  #######################################################################
  # Prints log messages
  def printLog(self, message="", typeMessage="info"):
    if typeMessage == "info":
      logging.info(message)
    elif typeMessage == "error":
      logging.error(message)

  ############################################
  ############################################
  def __init__(self, drone_id, FLIGHT_ALTITUDE = 1.5, RATE = 50, RADIUS = 5.0, CYCLE_S = 8, barrier=None, lock=None, droneName="", camerasName=[],origin=[],dest=[]):
    threading.Thread.__init__(self)
    outputFile   = os.path.dirname(__file__)+"/mission_handler.log"
    config_root_logger(log_file=outputFile)
    self.printLog(message="Logger initialized", typeMessage="info")
    # Drone info
    self.drone_id      = drone_id
    self.droneName     = droneName
    self.camerasName   = camerasName
    
    self.origin_x = origin[0]
    self.origin_y = origin[1]
    
    self.dest_x = dest[0]
    self.dest_y = dest[1]

    # variables to connect with Javascript
    #self.channel_layer   = get_channel_layer()
    #self.groupName       = "uav"

    # global static variables
    self.FLIGHT_ALTITUDE = -1 * FLIGHT_ALTITUDE     # fgdf
    self.RATE            = RATE                     # loop rate hz
    self.RADIUS          = RADIUS                   # radius of figure 8 in meters
    self.CYCLE_S         = CYCLE_S                  # time to complete one figure 8 cycle in seconds
    self.STEPS           = int( self.CYCLE_S * self.RATE )
    self.running         = 0
    self.en_debug        = 0

  ############################################
  ############################################
  def run(self):
    self.printLog(message="Run executed", typeMessage="info")
    self.loop = asyncio.new_event_loop()
    asyncio.set_event_loop(self.loop)
    self.loop.run_until_complete(self.initialize())   # Arm and takeoff the Drone
    self.loop.create_task(self.getTelemetry())        # Infinite loop getting the telemetry values each N seconds
    self.loop.create_task(self.mission())             # Mission process
    self.loop.run_forever()                           # Or until the process is killed? To determine

  #####
  ########################################## http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/plugins/telemetry.html#mavsdk.telemetry.Telemetry.velocity_ned
  async def getPosition(self):
    async for pos in self.drone.telemetry.position():
      self.position = pos
  #####
  async def getFlightMode(self):
    async for flight_mode in self.drone.telemetry.flight_mode():
      self.flightMode = flight_mode
  #####
  async def getRotation(self):
    async for uav_att in self.drone.telemetry.attitude_euler():
        self.uav_attitude = uav_att  
  #####
  async def getQuaternion(self):
    async for quaternion in self.drone.telemetry.attitude_quaternion():
        self.quaternion = quaternion  
  #####
  async def getInAir(self):
    async for in_air in self.drone.telemetry.in_air():
      self.in_air = in_air
  #####
  async def getIsArmed(self):
    async for is_armed in self.drone.telemetry.armed():
      self.is_armed = is_armed
  #####
  async def getPosVel(self):
    async for posandvel in self.drone.telemetry.position_velocity_ned():
      self.posandvel = posandvel
  #####
  async def getTime(self):
    async for time in self.drone.telemetry.unix_epoch_time():
      self.time = time
  #####
  async def getHeading(self):
    async for heading in self.drone.telemetry.heading():
      self.heading = heading
  #####
  async def getTelemetry(self):
    print("getTelemetry")
    while True:
        await asyncio.sleep(2)

        await self.camera1()

        # Store the telemetry values at its corresponding array
        self.arrPosition.append(   self.position )
        self.arrFlightMode.append( self.flightMode )
        self.arrIn_air.append(     self.in_air )
        self.arrIs_armed.append(   self.is_armed )
        self.arrPosandvel.append(  self.posandvel )
        self.arrTime.append(       self.time )
        self.arrHeading.append(    self.heading )
        self.arrQuaternion.append( self.quaternion )

        # Prints at the lof file the telemetry
        self.printLog(message="Drone "+str(self.drone_id)+" position: "  +str(self.position),      typeMessage="info")
        self.printLog(message="Drone "+str(self.drone_id)+" flightMode: "+str(self.flightMode),    typeMessage="info")
        self.printLog(message="Drone "+str(self.drone_id)+" in_air: "    +str(self.in_air),        typeMessage="info")
        self.printLog(message="Drone "+str(self.drone_id)+" is_armed: "  +str(self.is_armed),      typeMessage="info")
        self.printLog(message="Drone "+str(self.drone_id)+" posandvel: " +str(self.posandvel),     typeMessage="info")
        self.printLog(message="Drone "+str(self.drone_id)+" time: "      +str(self.time),          typeMessage="info")
        self.printLog(message="Drone "+str(self.drone_id)+" heading: "   +str(self.heading),       typeMessage="info")
        self.printLog(message="Drone "+str(self.drone_id)+" quaternion: "+str(self.quaternion),    typeMessage="info")
        self.printLog(message="",      typeMessage="info")

        # Extract the info from the complex variables
        if self.position is not None:
          latitude_deg        = self.position.latitude_deg
          longitude_deg       = self.position.longitude_deg
          absolute_altitude_m = self.position.absolute_altitude_m
          relative_altitude_m = self.position.relative_altitude_m
        else:
          latitude_deg        = None
          longitude_deg       = None
          absolute_altitude_m = None
          relative_altitude_m = None
        if self.heading is not None:
          heading_deg = self.heading.heading_deg    
        else:
          heading_deg = None
        if self.posandvel is not None:
          posNorth = self.posandvel.position.north_m
          posEast  = self.posandvel.position.east_m
          posDown  = self.posandvel.position.down_m
          velNorth = self.posandvel.velocity.north_m_s
          velEast  = self.posandvel.velocity.east_m_s
          velDown  = self.posandvel.velocity.down_m_s
        else:
          posNorth = None
          posEast  = None
          posDown  = None
          velNorth = None
          velEast  = None
          velDown  = None
        if self.quaternion is not None:
          wQuaternion = self.quaternion.w
          xQuaternion = self.quaternion.x
          yQuaternion = self.quaternion.y
          zQuaternion = self.quaternion.z
          timestampQuaternion = self.quaternion.timestamp_us
        else:
          wQuaternion = None
          xQuaternion = None
          yQuaternion = None
          zQuaternion = None
          timestampQuaternion = None

        # Send the info to Javascript (first to mainMap, and then to JS)
        message = {
          'type':       'test',
          'message':    'Telemetry',
          'drone':      self.drone_id,
          'positionGlobal': {
            'latitude_deg': latitude_deg,
            'longitude_deg': longitude_deg,
            'absolute_altitude_m': absolute_altitude_m,
            'relative_altitude_m': relative_altitude_m,
          },
          'positionNED':  {
            'north': posNorth,
            'east':  posEast,
            'down':  posDown,
          },
          'velocityNED':  {
            'north': velNorth,
            'east':  velEast,
            'down':  velDown,
          },
          'quaternion': {
            'wQuaternion': wQuaternion,
            'xQuaternion': xQuaternion,
            'yQuaternion': yQuaternion,
            'zQuaternion': zQuaternion,
            'timestampQuaternion': timestampQuaternion,
          },
          'flightMode': str( self.flightMode ),
          'in_air':     str( self.in_air ),
          'is_armed':   str( self.is_armed ),
          'time':       self.time,
          'heading':    heading_deg,
        }
        await self.channel_layer.group_send(self.groupName, message)

  ##################################
  

  async def camera1(self):
    #self.camerasName[0] = "FPV"
    print("1")
    responses = self.airsimClient.simGetImages([airsim.ImageRequest(self.camerasName[0], airsim.ImageType.Scene, False, False)], vehicle_name=self.droneName)
    print("2")
    img_rgb_1d = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8) 
    print("3")
    img_rgb    = img_rgb_1d.reshape(responses[0].height, responses[0].width, 3)
    print("4")
    # for saving, you can do :
    cv2.imwrite(time.strftime("%Y%m%d-%H%M%S") + ".png", img_rgb)
    print("5")

    #print("HAGO FOTO en vehículo: "+self.droneName+" (id: "+str(self.drone_id)+", camara: "+self.camerasName[0]+")")
    #raw_image = self.airsimClient.simGetImage(camera_name=self.camerasName[0], image_type=0, vehicle_name=self.droneName)
    #print("Foto hecha, la guardo en algún lado")
    #airsim.write_png(str(len(raw_image))+ '.png', raw_image)
#
    #self.airsimClient.simGetCameraInfo(self.camerasName[0])
    #self.printLog(message=a, typeMessage="info")

  ##########################################
  ##########################################
  async def initialize(self):
    self.printLog(message="START initialize", typeMessage="info")
    # Cálculo de la trayectoria y del punto inicial
    self.path, self.home = self.init_path()

    # AirSim Client init
    self.printLog(message="Init AirSim client", typeMessage="info")
    self.airsimClient = airsim.MultirotorClient(os.environ['PX4_SIM_HOST_ADDR'])
    self.airsimClient.confirmConnection()

    # Conexión del UAV y ajuste de su home point
    self.printLog(message="START mission 2", typeMessage="info")

    # Connect this Drone to the MavSDK-Server
    portBase  = 50050
    portBase2 = 14550
    port = portBase2 + self.drone_id

    self.printLog(message="Drone"+str(self.drone_id)+" connecting to port "+str(port)+"...", typeMessage="info")

    self.drone = System()
    await self.drone.connect(system_address="udp:#:"+str(port))
    self.printLog(message="-- Arming "+str(self.drone_id), typeMessage="info")
    await self.drone.action.arm()
    self.printLog(message="-- Setting initial setpoint "+str(self.drone_id), typeMessage="info")
    await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0)) # Tras esto no se puede hacer sleep parece, sino da error
    self.printLog(message="-- Starting offboard "+str(self.drone_id), typeMessage="info")
    try:
        await self.drone.offboard.start()
    except OffboardError as error:
        self.printLog(message="-- Starting offboard mode failed with error code "+str(self.drone_id), typeMessage="info")
        self.printLog(message="-- error._result.result "+str(self.drone_id), typeMessage="info")
        self.printLog(message="-- Landing "+str(self.drone_id), typeMessage="info")
        await self.drone.action.land()
        await asyncio.sleep(5)
        self.printLog(message="-- Disarming "+str(self.drone_id), typeMessage="info")
        await self.drone.action.disarm()
        return

    # El inicio de la trayectoria, despega y se coloca
    self.printLog(message="-- Setting second setpoint "+str(self.drone_id), typeMessage="info")
    await self.drone.offboard.set_position_ned(PositionNedYaw(self.home.x, self.home.y, self.home.z, self.home.yaw))
    await asyncio.sleep(5)

    self.lastTime = datetime.now().second

    self.printLog(message="END initialize", typeMessage="info")

  ##########################################
  ##########################################
  async def mission(self):
    self.printLog(message="START mission", typeMessage="info")

    # Start the tasks
    asyncio.ensure_future( self.getPosition()    )
    asyncio.ensure_future( self.getFlightMode()  )
    asyncio.ensure_future( self.getQuaternion()  )
    asyncio.ensure_future( self.getInAir()       )
    asyncio.ensure_future( self.getIsArmed()     )
    asyncio.ensure_future( self.getPosVel()      )
    asyncio.ensure_future( self.getTime()        )
    asyncio.ensure_future( self.getHeading()     )

    # now begin figure 8 path
    i=0
    running = True
    while running == True:
      #print("Se mueve a: "+str(self.path[i].x)+" "+str(self.path[i].y)+" "+str(self.path[i].z)+" "+str(self.path[i].yaw))

      # This code doesn't use the vx, vy, vz nor ax, ay, az nor yaw_rate
      # maybe other command can exploit them
      await self.drone.offboard.set_position_ned(PositionNedYaw(self.path[i].x, self.path[i].y, self.path[i].z, self.path[i].yaw))

      i = i + 1
      if i >= self.STEPS:
          i = 0

      await self.usleep()             # Se supone que calculado según el ratio indicado
      
    # Proceso de desconexión
    print("-- Stopping offboard")
    try:
        await self.drone.offboard.stop()
        print("-- Landing")
        await self.drone.action.land()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("exiting offboard figure eight thread\n")

    self.printLog(message="END mission", typeMessage="info")


  ##########################################
  ## Aproximación para hacer el sleep exacto
  async def usleep(self):
      current_time = time.time_ns() # Tiempo actual en nanosegundos

      # try to maintain output data rate
      next_time = current_time + (1000000.0/self.RATE)

      a = (next_time-current_time) / 1000000.0
      #print("Duerme: "+str(a)+" seconds.")
      time.sleep(a)

  ##########################################
  # generate a path following Bernoulli's lemiscate as a parametric equation
  # note this is in ENU coordinates since mavros will convert to NED x right, y forward, z up.
  def init_path(self):
    i    = 0
    dt   = 1.0/self.RATE
    dadt = math.pi*2 / self.CYCLE_S # first derivative of angle with respect to time
    r    = self.RADIUS
    path = []
    for i in range(0, self.STEPS):

        position = Position()
        # basic fields in the message
        position.time_boot_ms = 0
        #position.coordinate_frame = MAV_FRAME_LOCAL_NED
        position.type_mask = 0 # use everything!!
        # position.type_mask =     POSITION_TARGET_TYPEMASK_AX_IGNORE |
        #                         POSITION_TARGET_TYPEMASK_AY_IGNORE |
        #                         POSITION_TARGET_TYPEMASK_AZ_IGNORE
        position.target_system = 0 # will reset later when sending
        #position.target_component = PX4_COMPID
        y_dist = -abs(self.origin_y - self.dest_y) * 111000
        x_dist = -abs(self.origin_x - self.dest_x) * 111000 * math.cos(self.origin_y)

        # Position
        # https:#www.wolframalpha.com/input/?i=%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
        position.x = x_dist
        # https:#www.wolframalpha.com/input/?i=%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
        position.y =  y_dist
        position.z =  self.FLIGHT_ALTITUDE

        # Velocity
        # https:#www.wolframalpha.com/input/?i=derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
        position.vx =   0.0
        # https:#www.wolframalpha.com/input/?i=derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
        position.vy =  0.0
        position.vz =  0.0

        # Acceleration
        # https:#www.wolframalpha.com/input/?i=second+derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
        position.afx =  0.0
        # https:#www.wolframalpha.com/input/?i=second+derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
        position.afy =  0.0
        position.afz =  0.0

        # calculate yaw as direction of velocity
        position.yaw = math.atan2(position.vy, position.vx)

        # Se añade esta posición al array
        path.append(position)
      

    # calculate yaw_rate by dirty differentiating yaw
    for i in range(0, self.STEPS):
      next = path[(i+1)%self.STEPS].yaw
      curr = position.yaw
      # account for wrap around +- PI
      if((next-curr) < -math.pi):
          next = next + math.pi*2
      if((next-curr) >  math.pi):
          next = next - math.pi*2
      position.yaw_rate = (next-curr)/dt
      
    en_debug = False
    if en_debug == True:
        # dump out the trajectory for debugging.
        print("===========================================================\n")
        print("   X     Y\n")
        for i in range(0,self.STEPS):
            print("x: "+str(position.x)+" y: "+str(position.y))
        
        print("===========================================================\n")
        print("   vx    dx/dt     vy    dy/dt\n")
        for i in range(0,self.STEPS):
            dx = (path[(i+1)%self.STEPS].x - position.x)/dt
            dy = (path[(i+1)%self.STEPS].y - position.y)/dt
            print("vx:%7.3f dx/dt:%7.3f  vy:%7.3f dy/dt:%7.3f\n", position.vx, dx, position.vy, dy)
        
        print("===========================================================\n")
        print("   ax    d^2x/dt     ay    d^2y/dt\n")
        for i in range(0,self.STEPS):
            d2x = (path[(i+1)%self.STEPS].vx - position.vx)/dt
            d2y = (path[(i+1)%self.STEPS].vy - position.vy)/dt
            print("Ax:%7.3f d2x/dt:%7.3f  Ay:%7.3f d2y/dt:%7.3f\n", position.afx, d2x, position.afy, d2y)
        
        print("===========================================================\n")
        print("   yaw     yaw_rate\n")
        for i in range(0,self.STEPS):
            print("yaw:%7.1f deg  yaw_rate: %7.1f deg/s\n", (position.yaw)*180.0/math.pi, (position.yaw_rate)*180.0/math.pi)
        
        print("===========================================================\n")
      

    # now set home position
    home_position = Position()
    home_position.time_boot_ms = 0
    home_position.coordinate_frame = path[0].coordinate_frame
    home_position.y = 0.0
    home_position.x = 0.0
    home_position.z = path[0].z
    home_position.yaw = path[0].yaw
    home_position.target_system = 0 # will reset later when sendin
    return path, home_position