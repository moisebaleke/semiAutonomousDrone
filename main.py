from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from time import sleep
import threading
from PlanSimpleMission import planSimpleMission
import serial
import math

# Assuming the RPLIDAR's SDK provides a class named RPLidar that handles communication with the LIDAR
from rplidar import RPLidar

takeoff_altitude = 1
fetch_data = True

# Connect to the Vehicle.7
# The argument "/dev/ttyTH2:57600" is the address to connect to the vehicle's flight controller over the ttyTHS2 port.

print ('Connecting to vehicle on: serial port ttyTHS2')
vehicle = connect("/dev/ttyTH2:57600", baud=57600, wait_ready=True)

# Function to connect to your drone
def connect_drone():
    vehicle = connect("/dev/ttyTH2:57600", baud=57600, wait_ready=True)
    return vehicle

currentPositionX = vehicle.current_location.long
currentPositionY = vehicle.current_location.lat

# Function to fetch the current position
def fetch_current_position(myVehicle):
    """
    Fetches the current position of the drone.

    Returns:
        dict: A dictionary containing the latitude, longitude, and altitude of the drone.
    """
    # Get the current location
    current_location = myVehicle.location.global_relative_frame
    return {
        'latitude': current_location.lat,
        'longitude': current_location.lon,
        'altitude': current_location.alt
    }

tempReturnLocation = fetch_current_position(vehicle)
returnLocation = vehicle.LocationGlobalRelative(tempReturnLocation['latitude'], tempReturnLocation['longitude'], tempReturnLocation['altitude'])
'''
# Arm the vehicle and takeoff
vehicle.armed = True
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

print("Taking off!")
vehicle.simple_takeoff(takeoff_altitude)

# Wait until the vehicle reaches a safe height
while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= takeoff_altitude * 0.95:
        print("Reached target altitude")
        break
    time.sleep(1)

'''

# Function to take off
def arm_and_takeoff(takeoff_altitude, vehicle):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(takeoff_altitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= takeoff_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Function to detect obstacles and adjust the path
def avoid_obstacles(vehicle):
    # Assuming `read_lidar_distance` is a function that reads the distance from the LIDAR sensor
    distance_to_obstacle = read_lidar_distance()

    if distance_to_obstacle < 1.0:  # Obstacle detected within 1 meter
        print("Obstacle detected! Taking evasive action.")
        # Example maneuver: ascend 5 meters, move forward 10 meters, descend 5 meters
        current_location = vehicle.location.global_relative_frame
        detour_location = LocationGlobalRelative(current_location.lat, current_location.lon, current_location.alt + 5)
        vehicle.simple_goto(detour_location)
        time.sleep(10)  # Adjust time based on the speed of your drone and distance to cover

# Replace this function with your method of reading from the LIDAR
def read_lidar_distance():
    # Code to read from your LIDAR sensor
    # This is a placeholder function. You'll need to implement this according to your LIDAR's API.
    return 1.5  # Example distance, replace with actual reading

def automatic_mission(target_location):
    """
    Plans and executes an automatic mission to a specific geographical location.
    """
    # Example: Fly to a specific location
    vehicle.simple_goto(target_location)

    # Wait until the vehicle reaches the destination
    # You might want to implement a more sophisticated condition check here
    while vehicle.location.global_relative_frame != target_location:
        avoid_obstacles(vehicle)
        time.sleep(1)

    # Execute post-mission actions
    post_mission_actions()

def obstacle_avoidance():
    """
    TO BE COMPLETED
    Runs a machine learning algorithm to dynamically avoid obstacles.
    """
    while True:
        # Your obstacle avoidance logic here
        time.sleep(1)

def safety_monitor():
    """
    TO BE COMPLETED
    Monitors battery level and other safety parameters.
    """
    while True:
        if vehicle.battery.level < 20:  # Example threshold
            print("Battery low, landing...")
            vehicle.mode = VehicleMode("LAND")
            break
        time.sleep(5)

def weather_and_stability_monitor():
    """
    TO BE COMPLETED
    Checks for adverse weather or instability and lands the drone if necessary.
    """
    while True:
        # Your weather and stability checks here
        time.sleep(5)

def post_mission_actions():
    """
    TO BE COMPLETED
    Executes a set of actions after reaching the destination.
    """
    print("Mission completed. Executing post-mission actions...")
    # Your post-mission logic here

def lidar_data_fetcher(lidar, data_queue):
    """
    Fetch data from the LIDAR and put it into the queue.
    If an object is detected within 1 meter in the front, put (1, size) into the queue.
    """
    try:
        print("Starting LIDAR data fetcher")
        object_detected = False
        first_angle = last_angle = None
        min_distance = float('inf')
        
        while True:
            for scan in lidar.iter_scans():
                for (_, angle, distance) in scan:
                    if distance < 1000:  # If object detected within 1 meter
                        if not object_detected:
                            # Starting to detect an object
                            object_detected = True
                            first_angle = last_angle = angle
                            min_distance = distance
                        else:
                            # Continue to track the object
                            last_angle = angle
                            min_distance = min(min_distance, distance)
                    elif object_detected:
                        # No longer detecting the object, process the information
                        angular_width = (last_angle - first_angle) % 360
                        if angular_width > 180:  # Correcting the angle if it crosses the 0-degree mark
                            angular_width = 360 - angular_width
                        # Estimate the object size (width)
                        object_size = 2 * (min_distance * math.tan(math.radians(angular_width / 2)))
                        data_queue.put((1, object_size))
                        
                        # Reset variables for next object detection
                        object_detected = False
                        first_angle = last_angle = None
                        min_distance = float('inf')

            sleep(0.1)
    finally:
        lidar.stop()
        lidar.disconnect()

if __name__ == "__main__":
    arm_and_takeoff(10, vehicle)  # Take off to 10 meters altitude

    # Starting all threads
    threads = []
    threads.append(threading.Thread(target=automatic_mission))
    threads.append(threading.Thread(target=obstacle_avoidance))
    threads.append(threading.Thread(target=safety_monitor))
    threads.append(threading.Thread(target=weather_and_stability_monitor))

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()

    print("Mission completed !")

def return_to_launch():
    """
    TO BE COMPLETED
    Commands the drone to return to its departure point.
    """
    print("Returning to launch")
    vehicle.mode = VehicleMode("RTL")

    vehicle.simple_goto(returnLocation)

    # Wait for the vehicle to land
    while vehicle.location.global_relative_frame.alt > 1:
        print("Descending...")
        time.sleep(1)
    
    print("Landed.")




