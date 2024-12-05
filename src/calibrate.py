import time
import Jetson.GPIO as GPIO
from mpu6050 import mpu6050
from gps import gps, WATCH_ENABLE

# Pin setup
MOTOR_PWM_LEFT = 33
MOTOR_PWM_RIGHT = 32
DIR_LEFT = 11  # Adjust based on wiring
DIR_RIGHT = 13
ENCODER_LEFT = 15
ENCODER_RIGHT = 16

# Initialize GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR_PWM_LEFT, GPIO.OUT)
GPIO.setup(MOTOR_PWM_RIGHT, GPIO.OUT)
GPIO.setup(DIR_LEFT, GPIO.OUT)
GPIO.setup(DIR_RIGHT, GPIO.OUT)
GPIO.setup(ENCODER_LEFT, GPIO.IN)
GPIO.setup(ENCODER_RIGHT, GPIO.IN)

# Initialize PWM
pwm_left = GPIO.PWM(MOTOR_PWM_LEFT, 1000)  # 1kHz frequency
pwm_right = GPIO.PWM(MOTOR_PWM_RIGHT, 1000)
pwm_left.start(0)
pwm_right.start(0)

# Initialize sensors
mpu = mpu6050(0x68)
gpsd = gps(mode=WATCH_ENABLE)

# Calibration functions
def calibrate_mpu6050():
    print("Calibrating MPU6050...")
    accel_data = mpu.get_accel_data()
    gyro_data = mpu.get_gyro_data()
    print(f"Accelerometer: {accel_data}")
    print(f"Gyroscope: {gyro_data}")

def calibrate_gps():
    print("Calibrating GPS...")
    for i in range(5):
        report = gpsd.next()
        if report['class'] == 'TPV':
            print(f"Latitude: {report.lat}, Longitude: {report.lon}")
        time.sleep(1)

def calibrate_motors():
    print("Testing motor movement...")
    pwm_left.ChangeDutyCycle(50)
    pwm_right.ChangeDutyCycle(50)
    GPIO.output(DIR_LEFT, GPIO.HIGH)
    GPIO.output(DIR_RIGHT, GPIO.HIGH)
    time.sleep(2)
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)

def calibrate_encoders():
    print("Testing encoder signals...")
    for i in range(10):
        left_state = GPIO.input(ENCODER_LEFT)
        right_state = GPIO.input(ENCODER_RIGHT)
        print(f"Left Encoder: {left_state}, Right Encoder: {right_state}")
        time.sleep(0.5)

# Main calibration function
def run_calibration():
    try:
        print("Starting calibration...")
        calibrate_mpu6050()
        calibrate_gps()
        calibrate_motors()
        calibrate_encoders()
        print("Calibration complete.")
    except KeyboardInterrupt:
        print("Calibration interrupted.")
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    run_calibration()