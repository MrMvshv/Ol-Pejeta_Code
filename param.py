# MAP LIMITS (LATITUDE AND LONGITUDE) 4 SPIRAL
limit_north =  0.0254381
limit_south = 0.0243759
limit_west = 36.9045568
limit_east = 36.9056135

# MAP LIMITS (LATITUDE AND LONGITUDE) 4 LAWNMOWER
limit_north_lawn =  0.0254381
limit_south_lawn = 0.0243759
limit_west_lawn = 36.9045568
limit_east_lawn = 36.9056135

#spiral center location
spiral_lat = 0.0249821
spiral_long = 36.9051039

#home location
home_lat = 0.024739398211447956
home_long = 36.903472182958915


# GAME PARAMETERS
rhinoNbr = 10
droneNbr = 5
sensorRange = 400
foundThreshold = 2
foundThresholdLawn = 10

# NETWORK PARAMETERS
IP = "localhost"
END_POINT_HANDSHAKE = "handshake"
END_POINT_SENSE = "sense"

URL_HANDSHAKE = lambda ip: f"http://{ip}:8080/{END_POINT_HANDSHAKE}"
URL_SENSE = lambda ip: f"http://{ip}:8080/{END_POINT_SENSE}"

PORT_LISTERNER = 14550
PORT_MASTER = 5762

# DRONE PARAMETERS
takeOffAltitude = 100
takeOffThreshold = 0.01 # Percentage of the takeOffAltitude waited before considering takeoff complete