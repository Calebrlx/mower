import time
import math
import smbus
from mpu6050 import mpu6050  # External library for MPU6050
from gps import gps, WATCH_ENABLE  # External library for GPS
from motor_driver import MotorDriver  # Placeholder for your motor control library

# Initialize peripherals
mpu = mpu6050(0x68)  # I2C address for MPU6050
gpsd = gps(mode=WATCH_ENABLE)  # GPS daemon
motor = MotorDriver()  # Initialize motor driver (implement as needed)

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, target, current):
        error = target - current
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


def move_forward(inches, pid, target_angle):
    wheel_circumference = 10 * math.pi  # Adjust for your wheels in inches
    pulses_per_revolution = 100  # Encoder specs
    target_pulses = int((inches / wheel_circumference) * pulses_per_revolution)
    
    motor.reset_encoders()
    while motor.get_encoder() < target_pulses:
        # Read current orientation
        gyro_data = mpu.get_gyro_data()
        current_angle = gyro_data['z']
        
        # Calculate correction
        correction = pid.compute(target_angle, current_angle)
        
        # Adjust motor speeds for straight movement
        motor.set_speed(left_speed=50 - correction, right_speed=50 + correction)
        time.sleep(0.1)
    
    motor.stop()



def rotate_to_angle(target_angle, pid):
    while True:
        gyro_data = mpu.get_gyro_data()
        current_angle = gyro_data['z']
        error = abs(target_angle - current_angle)
        
        if error < 1:  # Threshold for precision
            break
        
        # Calculate rotation speed
        correction = pid.compute(target_angle, current_angle)
        motor.set_speed(left_speed=-correction, right_speed=correction)
        time.sleep(0.1)
    
    motor.stop()



def navigate_to_waypoint(target_lat, target_lon, pid):
    def haversine(lat1, lon1, lat2, lon2):
        R = 6371000  # Earth radius in meters
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    while True:
        report = gpsd.next()
        if report['class'] == 'TPV':
            current_lat = report.lat
            current_lon = report.lon
            
            # Calculate heading and distance
            distance = haversine(current_lat, current_lon, target_lat, target_lon)
            if distance < 1:  # Reached waypoint
                break
            
            target_heading = math.degrees(math.atan2(target_lon - current_lon, target_lat - current_lat))
            gyro_data = mpu.get_gyro_data()
            current_heading = gyro_data['z']
            
            # Correct heading and move forward
            correction = pid.compute(target_heading, current_heading)
            motor.set_speed(left_speed=50 - correction, right_speed=50 + correction)
            time.sleep(0.1)
    
    motor.stop()