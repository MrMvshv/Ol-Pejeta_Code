# Fix from https://stackoverflow.com/questions/70943244/attributeerror-module-collections-has-no-attribute-mutablemapping
import sys
if sys.version_info.major == 3 and sys.version_info.minor >= 10:
    import collections
    setattr(collections, "MutableMapping", collections.abc.MutableMapping)

import time
import itertools
import requests
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
import math

import param as PARAM
from dataTypes import geoLoc, geoCircle





class Drone:
    def __init__(self, sysID, IP, portNumber, takeoff = True):
        self.sysID = sysID
        self.IP = IP
        self.portNumber = portNumber
        self.takeoff = takeoff

        self.name = f"Drone{sysID}"
        self.spinner = itertools.cycle(['-', '/', '|', '\\'])
        self.rhinosFound = 0
        self.isLastWait = False

        self.printInfo(f"Connecting to {IP}:{portNumber}")
        self.startConnection()

    def startConnection(self):
        self.vehicle = connect(f"tcp:{self.IP}:{self.portNumber}", wait_ready=True)
        self.printInfo(f"Drone {self.sysID} ready to takeoff")
        if self.takeoff:
            self.printInfo(f"Drone {self.sysID} trying to takeoff")
            while not self.vehicle.is_armable:
                self.printInfo(" Waiting for vehicle to initialise...", wait=True)
                time.sleep(1)
            
            self.arm()
            self.take_off(PARAM.takeOffAltitude)
            self.printInfo(f"Drone {self.sysID} ready to search")
        self.printInfo(f"Drone {self.sysID} skipped takeoff and is ready to search")
    def printInfo(self, msg, wait=False):
        if wait:
            if not self.isLastWait:
                print()
            sys.stdout.write("\033[F\033[K")
            sys.stdout.write(f"\r{time.strftime('%Y-%m-%d %H:%M:%S')} - {self.name}: {next(self.spinner)} {msg}\n")
            self.isLastWait = True
        else:
            print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {self.name}: {msg}")        
            self.isLastWait = False

    def arm(self):
        self.printInfo("Switch to GUIDED and arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
    
        # Wait for vehicle to arm
        while not self.vehicle.armed:
            self.printInfo(" Waiting for arming...", wait=True)
            time.sleep(1)

    def take_off(self, altitude):
        self.printInfo("Taking off!")
        self.vehicle.simple_takeoff(altitude) 

        while True:
            self.printInfo(f" Altitude: {self.vehicle.location.global_relative_frame.alt}", wait=True)
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= altitude * PARAM.takeOffThreshold:
                self.printInfo("Reached target altitude")
                break
            time.sleep(1) 

    def get_position(self) -> geoLoc:
        pos = self.vehicle.location.global_relative_frame
        return geoLoc(pos.lat, pos.lon, pos.alt)

    def send_to_waypoint(self, waypoint : geoLoc):
        if waypoint.alt is None:
            waypoint.alt = self.vehicle.location.global_relative_frame.alt
        point = LocationGlobalRelative(waypoint.lat, waypoint.lon, waypoint.alt)
        self.vehicle.simple_goto(point)

    def is_waypoint_reached(self, waypoint : geoLoc, threshold=10):
        # @@@ TASK 1 @@@: Check if the drone reached the drone has reached the waypoint.
        # This function should return True if the drone is closer than the threshold to the waypoint
        pos = self.get_position() # The get_position() function request the position of the drone.
        dist = pos.distTo(waypoint) # The distTo() function computes the distance in meters between the drone and the waypoint
        # @@@ 1 LINE IS MISSING HERE @@@ 
        return dist < threshold # REMOVE
    
    def gotoWP(self, waypoint : geoLoc):
        # @@@ TASK 2 @@@: Write a function that sends the drones to a waypoint and then waits until it reaches it.
        self.send_to_waypoint(waypoint) # The send_to_waypoint() function sends the drone towards a waypoint.
        # @@@ 2 LINE IS MISSING HERE @@@: write a while loop that checks every second if the waypoint is reached (use the time.sleep() function)
        while not self.is_waypoint_reached(waypoint): # REMOVE
            time.sleep(1)

        self.printInfo(f"Waypoint reached {waypoint}")


class ManualSearch:
    def __init__(self, drone : Drone):
        self.drone = drone

    def search(self, step=PARAM.foundThreshold):
        print(f"@@@@ Manual search\n\tDrone ID: {self.drone.sysID}\n\tPress a,s,d,w to move the drone\n\tPress e to sense\n\tPress q to quit")
        while True:
            key = input()
            if key == "q":
                break
            elif key == "e":
                print(f"@@@@ Sensing with drone {self.drone.sysID}")
                 # Sense
                response = requests.post(PARAM.URL_SENSE(drone.IP), json={"drone_id": self.drone.sysID}).json()["sense_status"]
                print(f"\t\t{response}")
            elif key == "a":
                print(f"@@@@ Moving drone {self.drone.sysID} west {step}m")
                pos = self.drone.get_position()
                pos = pos.offset(east=-step, north=0)
                self.drone.gotoWP(pos)
            elif key == "d":
                print(f"@@@@ Moving drone {self.drone.sysID} east {step}m")
                pos = self.drone.get_position()
                pos = pos.offset(east=step, north=0)
                self.drone.gotoWP(pos)
            elif key == "w":
                print(f"@@@@ Moving drone {self.drone.sysID} north {step}m")
                pos = self.drone.get_position()
                pos = pos.offset(east=0, north=step)
                self.drone.gotoWP(pos)
            elif key == "s":
                print(f"@@@@ Moving drone {self.drone.sysID} south {step}m")
                pos = self.drone.get_position()
                pos = pos.offset(east=0, north=-step)
                self.drone.gotoWP(pos)
            else:
                continue


class LawnmowerSearch:
    def __init__(self, drone : Drone):
        self.drone = drone

    def search(self):
        pNW = geoLoc(PARAM.limit_north_lawn, PARAM.limit_west_lawn)
        pNE = geoLoc(PARAM.limit_north_lawn, PARAM.limit_east_lawn)
        pSW = geoLoc(PARAM.limit_south_lawn, PARAM.limit_west_lawn)

        # @@@ TASK 5 @@@: Explain what the next 4 lines do.
        latStepNbr = int(pNW.distTo(pNE) // (PARAM.foundThresholdLawn / 2**0.5) + 1)
        lonStepNbr = int(pNW.distTo(pSW) // (PARAM.foundThresholdLawn / 2**0.5) + 1)
        lat_points = np.linspace(PARAM.limit_south, PARAM.limit_north, latStepNbr).tolist()
        lon_points = np.linspace(PARAM.limit_west, PARAM.limit_east, lonStepNbr).tolist()

        grid_points = [geoLoc(lat, lon, PARAM.takeOffAltitude) for lat in lat_points for lon in (lon_points if lat_points.index(lat) % 2 == 0 else lon_points[::-1])]
        start_index = np.random.randint(len(grid_points))
        grid_points = grid_points[start_index:] + grid_points[:start_index]
        for point in grid_points:
            self.drone.gotoWP(point)
            print(f"Drone {self.drone.sysID} reached point {point}")    
        print("Lawnmower search complete")
        print("Going home")
        self.drone.gotoWP(geoLoc(PARAM.home_lat, PARAM.home_long, PARAM.takeOffAltitude))


class SpiralSearch:
    def __init__(self, drone: Drone):
        self.drone = drone

    def search(self):
        print("Spiral commenced")
        #center = geoLoc(PARAM.limit_south + (PARAM.limit_north-PARAM.limit_south)/2 , PARAM.limit_west + (PARAM.limit_east - PARAM.limit_west)/2, PARAM.takeOffAltitude) # Start at the center
        center = geoLoc(PARAM.spiral_lat , PARAM.spiral_long, PARAM.takeOffAltitude)
        step_size = PARAM.foundThreshold  # Adjust step size as needed
        angle_step = 75  # Adjust angle step for smoother spiral

        radius = 0
        angle = 0

        while True:  # We will add a break condition based on the bounds of the area
             #convert angle to radians
            angle_radians = math.radians(angle)
            # Calculate new coordinates using polar to cartesian conversion
            x_offset = radius * math.cos(angle_radians)
            y_offset = radius * math.sin(angle_radians)

            #offset from center
            new_location = center.offset(east=x_offset,north=y_offset)
            print(f"New spiral radius: {radius} and angle: {angle}")
            self.drone.gotoWP(new_location)
            radius += step_size
            angle += angle_step

            print(f"compare to params? : {new_location.lat}, {new_location.lon}")
            if (new_location.lat > PARAM.limit_north or new_location.lat < PARAM.limit_south or new_location.lon > PARAM.limit_east or new_location.lon < PARAM.limit_west):
                print("spiral search complete. Out of bounds")
                break
                


if __name__ == "__main__":
    # @@@ TASK 3 @@@: Fill in the network details for your own group.
    # Add system ID, i.e. the drone number assigned to your group.
    SYS_ID = 1
    # Add the drone IP address (same for all groups) as a string.
    IP = "127.0.0.1"
    # Add port number (specific to your groupe).
    PORT_NUMBER = 14550
    
    drone = Drone(SYS_ID, IP, PORT_NUMBER) # Connect to the simulated drone.

    # @@@ TASK 4 @@@: Change the task number (TASK_NUMBER) to 4 and run this script to test TASK 1, 2 and 3.
    # This will run the manual search algorithm

    # @@@ TASK 6 @@@: Change the task number (TASK_NUMBER) to 6 and run this script to test TASK 5.
    # This will run the lawnmower search algorithm

    # @@@ TASK 8 @@@: Change the task number (TASK_NUMBER) to 8 and run this script to test TASK 7.
    # This will run the triangulation search algorithm
    
    # @@@ TASK 9 @@@: Change the task number (TASK_NUMBER) to 9 and run this script to test the spiral search algorithm
    
    TASK_NUMBER = 9

    if TASK_NUMBER == 4:
        ManualSearch(drone)
    elif TASK_NUMBER == 9:
        print("task 9")
        SpiralSearch(drone).search()
        LawnmowerSearch(drone).search()
    else:
        raise ValueError(f"Unkown TASK_NUMBER: {TASK_NUMBER}")