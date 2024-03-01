from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

import argparse  
parser = argparse.ArgumentParser()
#parser.add_argument('--connect', default='127.0.0.1:14550')    
args = parser.parse_args()

# Connect to the Vehicle.
# The argument '127.0.0.1:14550' is the address to connect to the vehicle's flight controller over the ttyTHS2 port.
# You should replace '127.0.0.1:14550' with the appropriate connection string for your setup.
# For example, it might be 'serial port:baud rate', such as '/dev/ttyTHS2:57600' for the Pixhawk on a serial port with a 57600 baud rate.

print ('Connecting to vehicle on: serial port ttyTHS2')
vehicle = connect("/dev/ttyTH2:57600", baud=57600, wait_ready=True)

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print ("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print ("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print (" Waiting for arming...")
    time.sleep(1)

  print ("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print (" Altitude: ", vehicle.location.global_relative_frame.alt) 
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print ("Reached target altitude")
      break
    time.sleep(1)

    
def Land():
##This function ensures that the vehicle has landed (before vechile.close is called)

  print("Landing")
  ##thread_distance.join()
  time.sleep(1)
  vehicle.mode = VehicleMode("LAND")
  while vehicle.armed:
    time.sleep(1)
  vehicle.close()

def emergencyLand():
  ##This function ensures that when drone's battery is bellow 5%, it stops evrything and lands
  battery = vehicle.battery
  batteryPercentage = battery.level

  if batteryPercentage <= 5.0:
    print("Battery low: %.2f%%, Emergency Landing initiated ..." %batteryPercentage)
    #stop communication
    #land
    Land()
  elif batteryPercentage > 5.0:
    print("Battery sufficient: %.2f%%, Emergency Landing canceled ..." %batteryPercentage)
  else:
    print("Unable to fetch battery Percentage")
    #lower altitude for security 
    target_altitude = 1.0

    currentMode = vehicle.mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.simple_goto((vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, target_altitude))

    # Wait for the drone to reach the target altitude
    while True:
        time.sleep(10)
        current_altitude = vehicle.location.global_relative_frame.alt

        #check if target altitude reached
        if current_altitude in range(target_altitude -0.1, target_altitude +0.1): # Add a small margin to account for variations
            break
        
    vehicle.mode = currentMode
