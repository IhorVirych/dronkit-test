import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

connection_string = 'tcp:127.0.0.1:5762'
#---target B coords
lat_B = 50.443326
lon_B = 30.448078

# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)


#---nav functions

#---define distance between two point

def distance(lat1,lon1,lat2,lon2):
    R = 6371000 # earth radius
    f1 = math.radians(lat1)
    f2 = math.radians(lat2)
    df = f2 - f1
    dh = math.radians(lon2 - lon1)
    a = math.pow(math.sin(df / 2),2) + math.cos(f1) * math.cos(f2) * math.pow(math.sin(dh / 2),2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

#---define arm and takeoff
def arm_and_takeoff(altitude):
    print("Basic pre-arm cheks")

    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.mode.name=='GUIDED' and not vehicle.armed:
        print("Getting ready to take off ...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(altitude) 

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Reached target altitude at point A")
            break
        time.sleep(1)

        if not vehicle.armed: break # del in release

def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


arm_and_takeoff(100)

print("Set default/target airspeed to 10")
vehicle.airspeed = 10

print("Going towards point B ...")
pointB = LocationGlobalRelative(lat_B, lon_B, 100)
vehicle.simple_goto(pointB)

lat_cur = vehicle.location.global_relative_frame.lat
lon_cur = vehicle.location.global_relative_frame.lon

d =  d = distance(lat_cur,lon_cur,lat_B,lon_B)

while d > 10:
    lat_cur = vehicle.location.global_relative_frame.lat
    lon_cur = vehicle.location.global_relative_frame.lon
    d = distance(lat_cur,lon_cur,lat_B,lon_B)
    print(" Distance to point B %sm" %"{:.1f}".format(d))
    time.sleep(10)

print("arriving point B ...")
time.sleep(10)
#vehicle.mode = VehicleMode("ALTHOLD")
print("turning to yaw absolute 350 ...")
condition_yaw(350,False)
time.sleep(10)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()