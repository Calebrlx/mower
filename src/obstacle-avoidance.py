import serial
import time
import Jetson.GPIO as GPIO
from threading import Thread

# Radar Configuration
RADAR_PORT = '/dev/ttyTHS1'
RADAR_BAUDRATE = 256000
OBSTACLE_THRESHOLD = 500  # Distance in mm to consider an obstacle

# Motor Pin Configuration
MOTOR_PWM_LEFT = 33
MOTOR_PWM_RIGHT = 32
DIR_LEFT = 11
DIR_RIGHT = 13

# GPIO Setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR_PWM_LEFT, GPIO.OUT)
GPIO.setup(MOTOR_PWM_RIGHT, GPIO.OUT)
GPIO.setup(DIR_LEFT, GPIO.OUT)
GPIO.setup(DIR_RIGHT, GPIO.OUT)

# Initialize PWM
pwm_left = GPIO.PWM(MOTOR_PWM_LEFT, 1000)  # 1kHz
pwm_right = GPIO.PWM(MOTOR_PWM_RIGHT, 1000)
pwm_left.start(0)
pwm_right.start(0)

# Decode Radar Data
def decode_packet(raw_data):
    try:
        if len(raw_data) < 10:
            return {"Angle": None, "Distance (mm)": None, "Speed (mm/s)": None}

        angle = int.from_bytes(raw_data[4:6], byteorder='little', signed=False)
        distance = int.from_bytes(raw_data[6:8], byteorder='little', signed=False)
        speed = int.from_bytes(raw_data[8:10], byteorder='little', signed=False)

        return {
            "Angle": angle,
            "Distance (mm)": distance,
            "Speed (mm/s)": speed,
        }
    except Exception as e:
        print(f"Error decoding packet: {e}")
        return {"Angle": None, "Distance (mm)": None, "Speed (mm/s)": None}

# Motor Control Functions
def stop():
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)

def move_forward(speed=50):
    GPIO.output(DIR_LEFT, GPIO.HIGH)
    GPIO.output(DIR_RIGHT, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)

def turn_left(speed=50):
    GPIO.output(DIR_LEFT, GPIO.LOW)
    GPIO.output(DIR_RIGHT, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)

def turn_right(speed=50):
    GPIO.output(DIR_LEFT, GPIO.HIGH)
    GPIO.output(DIR_RIGHT, GPIO.LOW)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)

def reverse(speed=50):
    GPIO.output(DIR_LEFT, GPIO.LOW)
    GPIO.output(DIR_RIGHT, GPIO.LOW)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)

# Obstacle Avoidance Logic
def obstacle_avoidance():
    try:
        with serial.Serial(RADAR_PORT, RADAR_BAUDRATE, timeout=1) as ser:
            print(f"Listening on {RADAR_PORT} at {RADAR_BAUDRATE} baud...")

            while True:
                if ser.in_waiting:
                    raw_data = ser.read(ser.in_waiting)
                    decoded_data = decode_packet(raw_data)

                    distance = decoded_data["Distance (mm)"]
                    angle = decoded_data["Angle"]

                    if distance and distance < OBSTACLE_THRESHOLD:
                        print(f"Obstacle detected! Distance: {distance} mm, Angle: {angle}")
                        stop()
                        if angle < 180:  # Obstacle on the left
                            turn_right()
                        else:  # Obstacle on the right
                            turn_left()
                        time.sleep(1)  # Turn for a short period
                        move_forward()
                    else:
                        move_forward()
                
                time.sleep(0.1)

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nStopped obstacle avoidance.")
    finally:
        stop()
        GPIO.cleanup()

# Main Function
if __name__ == "__main__":
    obstacle_thread = Thread(target=obstacle_avoidance)
    obstacle_thread.start()

    try:
        while True:
            time.sleep(1)  # Main loop does nothing; obstacle avoidance runs in the thread
    except KeyboardInterrupt:
        print("\nExiting program.")
        GPIO.cleanup()