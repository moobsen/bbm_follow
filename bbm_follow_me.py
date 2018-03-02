#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Copyright {2018} {Bluebird Mountain | Moritz Obermeier}

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

#################### PARAMETERS ################################################
#GPIOs (using BCM style)
BUTTON_LEFT_PIN  = 17
BUTTON_RIGHT_PIN = 18
LED_GREEN_PIN  = 23
LED_RED_PIN    = 24
#other parameters
START_ALTITUDE = 6# in meters
FLY_ALTITUDE = 10  # in meters
GPS_REFRESH = 0.5 # in seconds
MIN_DISTANCE = 1  # in metersminimum distance between new and old gps location
DESCENT_ANGLE = 20 # in degrees
#KILL TIME NEEDS TO BE SMALLER THAN STOP TIME! 
STOP_TIME = 2 #time in seconds til drone lands (button 1)
KILL_TIME = 2 #time in seconds til drone drops from the sky (both buttons)
################################################################################

import dronekit 
import RPi.GPIO as GPIO
import gps
import threading
import socket
import time
import sys
import math
import traceback
import logging
import argparse  
import os
import signal
from geopy.distance import vincenty

#TODO cleanup denglish

def make_LED(color):
  setup_LED()
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

def setup_LED():
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(LED_GREEN_PIN, GPIO.OUT)
  GPIO.setup(LED_RED_PIN,   GPIO.OUT)

def setup_buttons():
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(BUTTON_LEFT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  GPIO.setup(BUTTON_RIGHT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    global gpsd #bring it in scope
    gpsd = gps.gps(mode=gps.WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true
 
  def run(self):
    global gpsd
    while self.running:
      gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer

# Callback-Funktion
def arm_and_takeoff(aTargetAltitude):
  """
  Arms vehicle and fly to aTargetAltitude.
  """
  logging.info("Basic pre-arm checks")
  # don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    logging.info(" Waiting for vehicle to initialise...")
    time.sleep(1)
  logging.info("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode = dronekit.VehicleMode("GUIDED")
  vehicle.armed = True  
  while not vehicle.armed:    
    logging.info(" Waiting for arming...")
    time.sleep(1)
  # vehicle.airspeed = 15 #m/s
  vehicle.groundspeed = 30 #m/s
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

# this function sets the drone to throw mode and arms it
# if the drone is armed and not in throw mode, throw is assummed completed
# 
def set_throw_wait(vehicle):
  if vehicle.armed:
    if vehicle.mode.name == 'THROW':
      logging.info('Wating for throw')
      make_LED('GREEN')
      return False
    else:
      # the drone should be flying, follow can start
      logging.info('Flight mode: %s' % vehicle.mode.name)
      return True
  else:
    if vehicle.mode.name != 'THROW':
      vehicle.mode = dronekit.VehicleMode('THROW') 
    # double check flight mode, commands to drone can get lost
    #while not vehicle.is_armable:
    #  logging.warning('Can not arm! Compass/GPS?')
    #  time.sleep(1)
    if vehicle.mode.name == 'THROW':
      vehicle.armed=True
    return False

def writePidFile():
  pid = str(os.getpid())
  f = open('/tmp/pid_bbmfollow', 'w')
  f.write(pid)
  f.close()


def connect(connection_string):
  try:
    # connect to the vehicle
    logging.info('Connecting to vehicle on: %s' % connection_string)
    vehicle = dronekit.connect(connection_string, wait_ready=True)
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    return vehicle
  except Exception as e:
    logging.error("Exception caught. Most likely connection to vehicle failed.")
    logging.error(traceback.format_exc())
    return 'None'

def end_gps_poller(gpsp):
  if gpsp == 'None':
    return
  logging.info('Killing GPS Thread...')
  gpsp.running = False
  gpsp.join() # wait for the thread to finish what it's doing

def main():
  #this is called if button 1 is pressed
  def interrupt_button_1(channel):
    time.sleep(KILL_TIME)
    if GPIO.input(BUTTON_RIGHT_PIN) == 0 and GPIO.input(BUTTON_LEFT_PIN) == 0:
      #killswitch
      vehicle.mode = dronekit.VehicleMode("STABILIZE")
      vehicle.armed = False
      logging.warning("KILL Button detected, disarming")
    time.sleep( STOP_TIME - KILL_TIME )
    if GPIO.input(BUTTON_LEFT_PIN) == 0:
      #land mode
      logging.warning("Land Button detected, Setting LAND mode")
      vehicle.mode = dronekit.VehicleMode("LAND")

  def signal_term_handler(signal, frame):
    logging.error('Caught SIGTERM (kill signal)')
    cleanup()
    end_gps_poller(gpsp)
    sys.exit(1)

  def cleanup():
    # close vehicle object before exiting script
    logging.info("Closing connection to vehicle object & safety landing")
    try:
      vehicle.mode = dronekit.VehicleMode("LAND")
      vehicle.close()
    except:
      logging.warning("Cleanup done, but there was no vehicle connection")
    else:
      logging.info("Done. Bye! :-)")

  writePidFile()
  gpsp = 'None'
  signal.signal(signal.SIGTERM, signal_term_handler)
  GPIO.setwarnings(False)
  setup_LED()
  setup_buttons()
  try:
    # Interrupt-Event hinzufuegen
    GPIO.add_event_detect(BUTTON_LEFT_PIN, GPIO.FALLING,
      callback = interrupt_button_1, bouncetime = 300)
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
    logging.info("Version: descent_angle v0.1")
    logging.info("System Time:" + time.strftime("%c"))
    logging.info("Flying altitude: %s" % FLY_ALTITUDE)
    logging.info("Time between GPS points: %s" % GPS_REFRESH)

    gpsp = GpsPoller()
    gpsp.start()
    make_LED('ORANGE')
    vehicle = 'None'
    while vehicle == 'None':
      vehicle = connect(connection_string)
    #arm_and_takeoff(START_ALTITUDE)
    last_alt=0
    launch_dest = 'None'
    while not set_throw_wait(vehicle):
      time.sleep(1)
    # main loop
    while True:
      if vehicle.mode.name != "GUIDED":
        logging.warning("Flight mode changed - aborting follow-me")
        break
      if (gpsd.valid) != 0:
        dest = dronekit.LocationGlobalRelative(gpsd.fix.latitude, 
            gpsd.fix.longitude, 0)
        if launch_dest == "None":
          launch_dest = vehicle.location.global_frame
        # altitude is ONLY determined by distance to launch point 
        distance = vincenty( (launch_dest.lat, launch_dest.lon),
                             (dest.lat, dest.lon) ).meters
        altitude = (math.tan(math.radians(DESCENT_ANGLE)) * distance)
        dest = dronekit.LocationGlobal(dest.lat, dest.lon, 
          launch_dest.alt-altitude)
        logging.debug('Distance between points[m]: %s' % distance)
        logging.info('Going to: %s' % dest)
        vehicle.simple_goto(dest, None, 30)
        time.sleep(GPS_REFRESH) #this needs to be smaller than 1s (see above)
      else:
        logging.warn("Currently no good GPS Signal")
        time.sleep(GPS_REFRESH)
    # broke away from main loop
    make_LED('ORANGE')
    cleanup()
    end_gps_poller(gpsp) 
    sys.exit(0)
        
  except socket.error:  
    make_LED("RED")
    logging.error ("Error: gpsd service does not seem to be running, "
         "plug in USB GPS or run run-fake-gps.sh")
    cleanup()
    end_gps_poller(gpsp) 
    sys.exit(1)

  except (KeyboardInterrupt, SystemExit):
    make_LED("RED")
    logging.info("Caught keyboard interrupt")
    cleanup()
    end_gps_poller(gpsp) 
    sys.exit(0)

  except Exception as e:
    make_LED("RED")
    logging.error("Caught unknown exception, trace follows:")
    logging.error(traceback.format_exc())
    cleanup()
    end_gps_poller(gpsp) 
    sys.exit(1)

if __name__ == "__main__":
  main()
