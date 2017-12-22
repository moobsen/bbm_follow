#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Copyright {2017} {Bluebird Mountain | Moritz Obermeier}
built on the follow me example distributed with dronekit source

original copyright notice
© Copyright 2015-2016, 3D Robotics.
followme - Tracks GPS position of your computer (Linux only).

This example uses the python gps package to read positions from a GPS attached 
to your laptop and sends a new vehicle.simple_goto command every two seconds to
move the vehicle to the current point.

When you want to stop follow-me, either change vehicle modes or type Ctrl+C to
exit the script.

Example documentation: http://python.dronekit.io/examples/follow_me.html
"""
#################### PARAMETERS ################################################
#GPIOs (using BCM style)
BUTTON_LEFT_PIN  = 17
BUTTON_RIGHT_PIN = 18
LED_GREEN_PIN  = 23
LED_RED_PIN    = 24
#other parameters
START_ALTITUDE = 6# in meters
FLY_ALTITUDE = 6  # in meters
GPS_REFRESH = 0.1 # in seconds, NEEDS to be smaller than 1, by gpsd design
MIN_DISTANCE = 1  # in metersminimum distance between new and old gps location
#KILL TIME NEEDS TO BE SMALLER THAN STOP TIME! 
STOP_TIME = 4 #time in seconds til drone lands
KILL_TIME = 2 #time in seconds til drone drops from the sky
################################################################################

import dronekit 
import RPi.GPIO as GPIO
import gps
import socket
import time
import sys
import math
import traceback
import logging
import argparse  
from geopy.distance import vincenty

#TODO sort and modulize, e.g. proper main function
#TODO cleanup denglish

GPIO.setmode(GPIO.BCM)
parser = argparse.ArgumentParser(
  description='Tracks GPS position of your computer (Linux only).')
parser.add_argument('--connect', 
          help="vehicle connection target string.")
parser.add_argument('--log',
          help="logging level")
args = parser.parse_args()
if args.connect:
  connection_string = args.connect
else:
  print("no connection specified via --connect, exiting")
  sys.exit()
if args.log:
  logging.basicConfig(filename='log-follow.log', level=args.log.upper())
else:
  print('No loging level specified, using WARNING')
  logging.basicConfig(filename='log-follow.log', level='WARNING')
logging.warning('################## Starting script log ##################')
logging.info("System Time:" + time.strftime("%c"))
logging.info("Flying altitude: %s" % FLY_ALTITUDE)
logging.info("Time between GPS points: %s" % GPS_REFRESH)

def make_LED(color):
  if color == "RED":
    GPIO.output(LED_GREEN_PIN, GPIO.LOW)
    GPIO.output(LED_RED_PIN,   GPIO.HIGH)
  elif color == "GREEN":
    GPIO.output(LED_GREEN_PIN, GPIO.HIGH)
    GPIO.output(LED_RED_PIN,   GPIO.LOW)
  elif color == "ORANGE":
    GPIO.output(LED_GREEN_PIN, GPIO.HIGH)
    GPIO.output(LED_RED_PIN,   GPIO.HIGH)
  else: #everything else is off
    GPIO.output(LED_GREEN_PIN, GPIO.LOW)
    GPIO.output(LED_RED_PIN,   GPIO.LOW)
  return

def setup_LED():
  GPIO.setup(LED_GREEN_PIN, GPIO.OUT)
  GPIO.setup(LED_RED_PIN,   GPIO.OUT)
  return

def setup_buttons():
  GPIO.setup(BUTTON_LEFT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  GPIO.setup(BUTTON_RIGHT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

setup_LED()
setup_buttons()
make_LED("ORANGE")

# Callback-Funktion
def interrupt_button_1(channel):
  time.sleep(KILL_TIME)
  if GPIO.input(BUTTON_RIGHT_PIN) == 0 and GPIO.input(BUTTON_LEFT_PIN) == 0:
    #killswitch
    vehicle.mode = dronekit.VehicleMode("STABILIZE")
    vehicle.armed = False
    logging.warning("KILL Button detected, disarming")
    make_LED("RED")
  time.sleep( STOP_TIME - KILL_TIME )
  if GPIO.input(BUTTON_LEFT_PIN) == 0:
    #land mode
    logging.warning("Land Button detected, Setting LAND mode")
    vehicle.mode = dronekit.VehicleMode("LAND")
    make_LED("ORANGE")

# Interrupt-Event hinzufuegen
GPIO.add_event_detect(BUTTON_LEFT_PIN, GPIO.FALLING,
            callback = interrupt_button_1, bouncetime = 300)

def arm_and_takeoff(aTargetAltitude):
  """
  Arms vehicle and fly to aTargetAltitude.
  """
  logging.info("Basic pre-arm checks")
  # don't let the user try to arm until autopilot is ready
  #while not vehicle.is_armable:
  #  logging.info(" Waiting for vehicle to initialise...")
  #  time.sleep(1)
  logging.info("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode = dronekit.VehicleMode("GUIDED")
  vehicle.armed = True  
  while not vehicle.armed:    
    logging.info(" Waiting for arming...")
    time.sleep(1)
  # vehicle.airspeed = 15 #m/s
  vehicle.groundspeed = 15 #m/s
  logging.info("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude)
  # check if height is safe before going anywhere
  while True:
    logging.info(" Altitude: %s"%vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.7: 
      #Trigger just below target alt.
      logging.info("Reached target altitude")
      make_LED("GREEN")
      break
    time.sleep(1)

def cleanup():
  # close vehicle object before exiting script
  logging.info("Closing vehicle object & clean up GPIO")
  make_LED("OFF")
  try:
    vehicle.close()
  except:
    GPIO.cleanup()
    logging.warning("Cleanup done, but there was no vehicle connection")
  else:
    GPIO.cleanup()
    logging.info("Done")

try:
  # connect to the vehicle
  logging.info('Connecting to vehicle on: %s' % connection_string)
  vehicle = dronekit.connect(connection_string, wait_ready=True)
  cmds = vehicle.commands
  cmds.download()
  cmds.wait_ready()
except Exception as e:
  logging.error("Exception caught. Most likely connection to vehicle failed.")
  logging.error(traceback.format_exc())
  cleanup()
  sys.exit(1)

try:
  # TODO: let gpsd run in it's own thread like (like dan.mandle.me)
  gpsd = gps.gps(mode=gps.WATCH_ENABLE)
  arm_and_takeoff(START_ALTITUDE)
  last_alt=0
  last_dest = vehicle.home_location
  # main loop
  while True:
    if vehicle.mode.name != "GUIDED":
      logging.warning("User has changed flight mode - aborting follow-me")
      break
    gpsd.next()
    # once we have a valid location (see gpsd documentation) we can start
    if (gpsd.valid & gps.LATLON_SET) != 0:
      dest = dronekit.LocationGlobal(gpsd.fix.latitude, 
          gpsd.fix.longitude, gpsd.fix.altitude)
      alt_gpsbox = dest.alt
      delta_z=0
      if math.isnan(alt_gpsbox):
        # NO new Z info from box (altitude)
        if last_alt==0:
          dest = dronekit.LocationGlobalRelative(dest.lat, dest.lon, 
                                                 FLY_ALTITUDE)
        else:
          dest.alt = last_alt
      else:
        # new Z info from box exists
        # before it's accepted, check altitude error
        alt_gpsbox_dilution = gpsd.fix.epv
        # check altitude and dilution reported from drone
        alt_drone = vehicle.location.global_frame.alt 
        alt_drone_dilution = vehicle.gps_0.epv
        logging.info('GPS box alt: %d +- %d | Drone alt: %d +- %d'
          % (alt_gpsbox, alt_gpsbox_dilution, alt_drone, alt_drone_dilution) )
        # check if the altitude "uncertainty bubbles" touch
        if alt_gpsbox + alt_gpsbox_dilution < alt_drone - alt_drone_dilution: 
          delta_z = last_alt - (dest.alt+FLY_ALTITUDE)
          last_alt = alt_gpsbox+FLY_ALTITUDE
          dest = dronekit.LocationGlobal(dest.lat, dest.lon, last_alt)
        else:
          if last_alt==0:
            dest = dronekit.LocationGlobalRelative(dest.lat, dest.lon, 
                                                   FLY_ALTITUDE)
          else:
            dest.alt = last_alt
      # check the distance between current and last position
      distance = vincenty( (last_dest.lat, last_dest.lon),
                           (dest.lat, dest.lon) ).meters
      distance = math.sqrt (distance*distance + delta_z*delta_z)
      logging.debug('Distance between points[m]: %s' % distance)
      # only send new point if distance is large enough
      if distance > MIN_DISTANCE:
        logging.info('Going to: %s' % dest)
        vehicle.simple_goto(dest)
        last_dest=dest
      time.sleep(GPS_REFRESH) #this needs to be smaller than 1s (see above)
    else:
      logging.warn("Currently no good GPS Signal")
      time.sleep(GPS_REFRESH)
  # broke away from main loop
  cleanup()
      
except socket.error:
  logging.error ("Error: gpsd service does not seem to be running, "
       "plug in USB GPS or run run-fake-gps.sh")
  cleanup()
  sys.exit(1)

except (KeyboardInterrupt, SystemExit):
  logging.info("Caught keyboard interrupt")
  cleanup()
  sys.exit(0)


except Exception as e:
  logging.error("Caught unknown exception, see trace")
  logging.error(traceback.format_exc())
  cleanup()
