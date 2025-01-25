# MAP LIMITS (LATITUDE AND LONGITUDE)
limit_north = 0.03749
limit_south = 0.01852
limit_west = 36.89164
limit_east = 36.91846

# GAME PARAMETERS
rhinoNbr = 10
droneNbr = 5
sensorRange = 400
foundThreshold = 50

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