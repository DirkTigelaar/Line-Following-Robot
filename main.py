# THIS CODE READS ALL THE SENSOR VALUES
# LINE FOLLOWING BAHAVIOUR

from machine import Pin, I2C, PWM
from time import sleep_us, sleep_ms, ticks_us, ticks_diff, sleep
from math import atan2, sqrt, cos

# --- Motor Control Setup ---
# L9110S Motor Driver Pins
# Motor 1 (Left Wheel)
motor1_in1_pin = Pin(12, Pin.OUT)
motor1_in2_pin = Pin(13, Pin.OUT)
motor1_pwm = PWM(motor1_in1_pin)
motor1_pwm.freq(1000)

# Motor 2 (Right Wheel)
motor2_in1_pin = Pin(14, Pin.OUT)
motor2_in2_pin = Pin(27, Pin.OUT)
motor2_pwm = PWM(motor2_in1_pin)
motor2_pwm.freq(1000)

BASE_SPEED = 500

def set_motor_speed(motor_pwm_obj, motor_in2_pin_obj, speed):
    if speed >= 0: # Forward
        motor_in2_pin_obj.value(0)
        motor_pwm_obj.duty(min(speed, 1023))
    else: # Backward
        motor_in2_pin_obj.value(1)
        motor_pwm_obj.duty(min(abs(speed), 1023))

def stop_motors():
    motor1_pwm.duty(0)
    motor2_pwm.duty(0)
    motor1_in2_pin.value(0)
    motor2_in2_pin.value(0)

# ----- MPU6050 Setup -----
class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')  # Wake up
        self.gyro_offset = {'x': 0, 'y': 0, 'z': 0}

    def _read_raw_data(self, reg):
        data = self.i2c.readfrom_mem(self.addr, reg, 6)
        return (
            self._to_int16(data[0], data[1]),
            self._to_int16(data[2], data[3]),
            self._to_int16(data[4], data[5])
        )

    def _to_int16(self, high, low):
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value

    def get_accel(self):
        ax, ay, az = self._read_raw_data(0x3B)
        return {
            'x': ax / 16384.0,
            'y': ay / 16384.0,
            'z': az / 16384.0
        }

    def get_gyro(self):
        gx, gy, gz = self._read_raw_data(0x43)
        ox = self.gyro_offset['x']
        oy = self.gyro_offset['y']
        oz = self.gyro_offset['z']
        return {
            'x': (gx - ox) / 131.0,
            'y': (gy - oy) / 131.0,
            'z': (gz - oz) / 131.0
        }

    def calibrate_gyro(self, samples=100):
        print("Calibrating gyro... hold still")
        sum_x = sum_y = sum_z = 0
        for _ in range(samples):
            gx, gy, gz = self._read_raw_data(0x43)
            sum_x += gx
            sum_y += gy
            sum_z += gz
            sleep(0.01)
        self.gyro_offset = {
            'x': sum_x / samples,
            'y': sum_y / samples,
            'z': sum_z / samples
        }
        print("Gyro calibration complete.")

# ----- Ultrasonic Sensor -----
def read_distance(trig, echo):
    trig.off()
    sleep_us(2)
    trig.on()
    sleep_us(10)
    trig.off()
    start = 0
    end = 0
    # Wait for echo to go high
    timeout_us = 25000  # 25 ms timeout for 4m range (approx)
    start_time_wait = ticks_us()
    while echo.value() == 0:
        start = ticks_us()
        if ticks_diff(start, start_time_wait) > timeout_us:
            return -1  # Timeout
    # Wait for echo to go low
    while echo.value() == 1:
        end = ticks_us()
        if ticks_diff(end, start) > timeout_us:
            return -1  # Timeout
    duration = ticks_diff(end, start)
    distance_cm = duration / 58.0
    return distance_cm

# ----- Setup Pins -----

# MPU6050 I2C
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
mpu = MPU6050(i2c)
mpu.calibrate_gyro()

# Global variables for yaw calculation
yaw_angle = 0.0
last_time = ticks_ms()

# Ultrasonic sensor
trig = Pin(19, Pin.OUT)
echo = Pin(23, Pin.IN)

# Button
button = Pin(18, Pin.IN, Pin.PULL_UP)

# Electromagnet
electromagnet = Pin(5, Pin.OUT)
electromagnet.off()

# IR line sensors
ir_pins = [
    Pin(39, Pin.IN),
    Pin(4, Pin.IN),
    Pin(35, Pin.IN),
    Pin(34, Pin.IN),
    Pin(36, Pin.IN)
]

# ----- Encoder Pins and Setup -----
pin_a1 = Pin(25, Pin.IN, Pin.PULL_UP)
pin_b1 = Pin(26, Pin.IN, Pin.PULL_UP)

pin_a2 = Pin(17, Pin.IN, Pin.PULL_UP)
pin_b2 = Pin(16, Pin.IN, Pin.PULL_UP)

position1 = 0
last_state1 = pin_a1.value()

position2 = 0
last_state2 = pin_a2.value()

def update_position1(pin):
    global position1, last_state1
    state = pin_a1.value()
    b_state = pin_b1.value()
    if state != last_state1:
        if b_state != state:
            position1 += 1
        else:
            position1 -= 1
    last_state1 = state

def update_position2(pin):
    global position2, last_state2
    state = pin_a2.value()
    b_state = pin_b2.value()
    if state != last_state2:
        if b_state != state:
            position2 += 1
        else:
            position2 -= 1
    last_state2 = state

pin_a1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_position1)
pin_a2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_position2)

# ----- Line Following Specific Paramters ---
KP = 100
MAX_CORRECTION = 200

# ----- Line Following Control Loop -----
def run_line_follower():
    global yaw_angle, last_time # Declare global to modify
    try:
        while True:
            # MPU6050
            # accel = mpu.get_accel() # Not used for yaw, so commenting out
            gyro = mpu.get_gyro()
            
            # Integrate gyro z to estimate yaw angle
            current_time = ticks_ms()
            dt = (current_time - last_time) / 1000.0
            last_time = current_time

            yaw_angle += gyro['z'] * dt  # degrees
            
            # Ultrasonic distance
            dist = read_distance(trig, echo)
            
            # Button state
            button_pressed = button.value() == 1
            
            # Activate electromagnet
            if button.value() == 0:
                electromagnet.off()
            else:
                electromagnet.on()
            
            ir_values = [pin.value() for pin in ir_pins]
            error = 0
            weights = [-2, -1, 0, 1, 2]

            num_active_sensors = 0
            weighted_sum = 0

            for i, sensor_value in enumerate(ir_values):
                if sensor_value == 0: # Assuming 0 means line detected (dark line on light background)
                    weighted_sum += weights[i]
                    num_active_sensors += 1

            if num_active_sensors > 0:
                error = weighted_sum / num_active_sensors
            else:
                stop_motors()
                print("Line lost! Stopping.")
                sleep_ms(500)
                continue

            correction = int(error * KP)

            left_speed = BASE_SPEED - correction
            right_speed = BASE_SPEED + correction

            left_speed = max(0, min(left_speed, 1023))
            right_speed = max(0, min(right_speed, 1023))

            set_motor_speed(motor1_pwm, motor1_in2_pin, left_speed)
            set_motor_speed(motor2_pwm, motor2_in2_pin, right_speed)

            print("\n--- Line Following ---")
            print("IR Sensors:", ir_values)
            print("Error:", error)
            print("Correction:", correction)
            print(f"Left Speed: {left_speed}, Right Speed: {right_speed}")
            print("Encoder 1 Count:", position1)
            print("Encoder 2 Count:", position2)
            print("Distance: {:.2f} cm".format(dist) if dist != -1 else "Ultrasonic: Timeout")
            print("Button Pressed:", button_pressed)
            print("Yaw angle (deg): {:.2f}".format(yaw_angle))
            
            sleep_ms(100)

    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        stop_motors()
        print("Motors stopped.")

# Start the line following loop
run_line_follower()