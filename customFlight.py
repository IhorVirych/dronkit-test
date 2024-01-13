from tkinter import Y
from turtle import distance
import os
from dronekit import connect, VehicleMode
import time
import math

connection_string = 'tcp:127.0.0.1:5763'
vehicle = connect(connection_string, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        os.system('cls')    
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def get_distance_metres(location):
    R = 6371000 # радіус землі
    f1 = math.radians(vehicle.location.global_relative_frame.lat)
    f2 = math.radians(location[0])
    df = f2 - f1
    dh = math.radians(location[1] - vehicle.location.global_relative_frame.lon)
    a = math.pow(math.sin(df / 2),2) + math.cos(f1) * math.cos(f2) * math.pow(math.sin(dh / 2),2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c


def get_bearing(location):
    off_x = location[1] - vehicle.location.global_relative_frame.lon
    off_y = location[0] - vehicle.location.global_relative_frame.lat
    bearing = math.atan2(off_x, off_y) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing

def maintain_altitude(target_altitude):
        current_altitude = vehicle.location.global_relative_frame.alt
        altitude_difference = target_altitude - current_altitude

        # Пропорційний контроль вісоти
        throttle_output = altitude_difference * 70  # поправковий коефіціент

        # встановлюємо значення в канал управління Throttle (канал 3)
        vehicle.channels.overrides['3'] = int(1500 + throttle_output)
        time.sleep(1)

def speed_controller(distance_to_target,factor):
    max_speed = 30  
    min_speed = 1

    # коефіціент швидкості
    speed_change_factor = factor

    current_speed = vehicle.groundspeed
    # Плавное изменение скорости в зависимости от расстояния и текущей скорости
    
    # пропорційна швидкість від дистанції в межах від 20 до 100м
    target_speed = min_speed + (max_speed - min_speed) * (1 - distance_to_target / 100)
    
    # встановлення граничних значень швидкості в залежності від дистанції
    if distance_to_target > 100:target_speed = max_speed
    if distance_to_target < 20:target_speed = min_speed
    

    # перетворення значення швидкості в значення для каналу Pitch
    if target_speed > 0: speed_channel_value = int(1480 - target_speed * speed_change_factor)
    if target_speed < 0: speed_channel_value = int(1520 + target_speed * speed_change_factor)

    # обмеження значення, в межах від 1000 до 2000
    speed_channel_value = max(min(speed_channel_value, 2000), 1000)

    # якщо до цілі менше 1 метра - нікуди не летимо
    if distance_to_target < 1 :speed_channel_value = int(1500)
    vehicle.channels.overrides['2'] = speed_channel_value
    return speed_channel_value
    

def custom_turn(yaw,target_point):
    while True:
        yaw_diff = yaw - vehicle.heading
        if yaw_diff > 180:
            yaw_diff -= 360
        elif yaw_diff < -180:
            yaw_diff += 360

       
        # пропорційний контроль каналами RC
        yaw_rate = max(-30, min(30, yaw_diff))
        if yaw_diff < 0: vehicle.channels.overrides['4'] = int(1480 + yaw_rate)
        if yaw_diff > 0: vehicle.channels.overrides['4'] = int(1520 + yaw_rate)
        if abs(yaw_diff) < 1:break
        maintain_altitude(target_point[2])
        time.sleep(1)

def move_to_point(target_point):
    while True:
        distance = get_distance_metres(target_point)

        bearing = get_bearing(target_point)

        os.system('cls')
        print("Відстань до цілі: ", distance)
        print("Кут до цілі: ", bearing)

        yaw_diff = bearing - vehicle.heading
        if yaw_diff > 180:
            yaw_diff -= 360
        elif yaw_diff < -180:
            yaw_diff += 360

        print("Yaw різниця: ", yaw_diff)
        base_value = 1500
        # пропорційний контроль каналами RC
        if distance > 1 :
            turn_speed_limit = 30
            turn_factor = 1
            

            if yaw_diff < 0: base_value = 1480
            if yaw_diff > 0: base_value = 1520
            
        
        maintain_altitude(target_point[2])
        speed_factor = 10

        if distance < 20:
            print("Наближаємось до цілі...")
            turn_speed_limit = 30
            speed_factor = 5
            turn_factor = 2


        if distance < 1:
            print("Досягли цілі")
            vehicle.channels.overrides['2'] = int(1500)
            vehicle.channels.overrides['4'] = int(1500)
            break
            
        yaw_rate = max(-turn_speed_limit, min(turn_speed_limit, yaw_diff))
        print("Yaw rate: ", yaw_rate)
        vehicle.channels.overrides['4'] = int(base_value + yaw_rate * turn_factor)
        vehicle.channels.overrides['2'] = int(1500)

        if abs(yaw_diff) < 1:
            print("speed_channel_value: ", speed_controller(distance,speed_factor))
        time.sleep(1)
        

def set_speed_none():
    pitch_value = int(1500)
    vehicle.channels.overrides['2'] = pitch_value


def wait_for_maneuver_completion(target_point):
    while True:
        if vehicle.velocity == [0, 0, 0]:
            print("Маневр завершено.")
            maintain_altitude(target_point[2])
            break
        time.sleep(5)



# Взліт на висоту 100 метрів
target_altitude = 100
arm_and_takeoff(target_altitude)

# Переміщення коптера из точки А в точку Б в режимі AltHold
vehicle.mode = VehicleMode("ALT_HOLD")
point_b = (50.443326, 30.448078, target_altitude)
move_to_point(point_b)

# поворот на 350 абсолютних згідно з завданням
print("поворот на 350 абсолютних згідно з завданням...")
custom_turn(350,point_b)
print("Завдання виконане")
time.sleep(3)

# вмикання режиму посадки
vehicle.mode = VehicleMode("LAND")

# очікування завершення посадки
while vehicle.armed:
    print("Очікування посадки...")
    time.sleep(1)

# Закриття з'єднання з коптером
vehicle.close()