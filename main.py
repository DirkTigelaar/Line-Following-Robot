# THIS CODE READS ALL THE SENSOR VALUES
# LINE FOLLOWING BEHAVIOUR WITH NODE DETECTION

from machine import Pin, I2C, PWM
from time import sleep_us, sleep_ms, ticks_us, ticks_diff, sleep, ticks_ms
from math import atan2, sqrt, cos, pi

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

BASE_SPEED = 350
MAX_SPEED_PWM = 1023 # Max PWM duty cycle

def set_motor_speed(motor_pwm_obj, motor_in2_pin_obj, speed):
    # Ensure speed is within PWM range [0, 1023]
    speed = max(0, min(int(speed), MAX_SPEED_PWM))
    if speed >= 0: # Forward
        motor_in2_pin_obj.value(0)
        motor_pwm_obj.duty(speed)
    else: # Backward (though not typically used in line following forward-only)
        motor_in2_pin_obj.value(1)
        motor_pwm_obj.duty(speed) # abs(speed) should be handled by max(0, min(int(speed), MAX_SPEED_PWM))

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
            'x': (gx - ox) / 131.0, # Gyro sensitivity for +/- 2000 deg/s
            'y': (gy - oy) / 131.0,
            'z': (gz - oz) / 131.0
        }

    def calibrate_gyro(self, samples=100):
        # print("Calibrating gyro... hold still")
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
        # print("Gyro calibration complete.")

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

# Global variables for yaw calculation (from MPU6050)
yaw_angle = 0.0 # Radians
last_mpu_time = ticks_ms()

# Ultrasonic sensor
trig = Pin(19, Pin.OUT)
echo = Pin(23, Pin.IN)

# Button
button = Pin(18, Pin.IN, Pin.PULL_UP)

# Electromagnet
electromagnet = Pin(5, Pin.OUT)
electromagnet.off()

# IR line sensors (assuming 0 = line detected, 1 = no line)
# Ordered from left to right, matching weights [-2, -1, 0, 1, 2]
ir_pins = [
    Pin(39, Pin.IN), # Far Left (-2 weight)
    Pin(4, Pin.IN),  # Left (-1 weight)
    Pin(35, Pin.IN), # Center (0 weight)
    Pin(34, Pin.IN), # Right (1 weight)
    Pin(36, Pin.IN)  # Far Right (2 weight)
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
        if b_state != state: # Clockwise
            position1 += 1
        else: # Counter-clockwise
            position1 -= 1
    last_state1 = state

def update_position2(pin):
    global position2, last_state2
    state = pin_a2.value()
    b_state = pin_b2.value()
    if state != last_state2:
        if b_state != state: # Clockwise
            position2 += 1
        else: # Counter-clockwise
            position2 -= 1
    last_state2 = state

pin_a1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_position1)
pin_a2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_position2)

# ----- Line Following and Node Detection Parameters ---
# PID gains for line following
KP = 100 # Proportional gain for line following
LINE_THRESHOLD = 0.5 # IR sensor threshold for line detection (assuming normalized 0-1 values if used)
                     # For digital sensors (0 or 1), this is less relevant, but could be for analog.
                     # Your current IRs are digital, so 0 is line, 1 is no line.

# State machine variables
current_state = 'line_following'
intersection_counter = 0
no_intersection_counter = 0
intersection_processed = False
turn_target_yaw = 0.0 # Target yaw angle in radians for turns
line_search_counter = 0

# --- Helper functions for state machine ---

def normalize_angle_rad(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle

def detect_intersection(ir_values):
    """
    Detect intersection using IR line sensors.
    Assuming ir_values are ordered [FarLeft, Left, Center, Right, FarRight]
    A true intersection means multiple (e.g., three or more) central sensors
    are on the line (value 0).
    """
    # For a 5-sensor array, a common robust intersection detection:
    # All three central sensors (Left, Center, Right) detect the line.
    # ir_values[1] (Left), ir_values[2] (Center), ir_values[3] (Right)
    # Or, for very wide intersections, check if more than 3 sensors are on the line.
    
    # Let's use the stricter approach from thonny_explained_dirk for "multiple active sensors"
    # Assuming 0 means line detected, 1 means no line.
    line_center = ir_values[2] == 0
    line_left = ir_values[1] == 0
    line_right = ir_values[3] == 0
    line_far_left = ir_values[0] == 0
    line_far_right = ir_values[4] == 0

    # A good intersection detection for a 5-sensor array might be:
    # If the center sensor AND at least one sensor on each side (left/right) detect the line.
    # This pattern indicates a crossroad or T-junction.
    # This is more robust than just checking all 5, which might be too strict for some turns.
    
    # Option 1: Center and both immediate neighbors detect line
    if line_center and line_left and line_right:
        return True
    
    # Option 2: All 5 sensors detect line (very clear intersection)
    if line_far_left and line_left and line_center and line_right and line_far_right:
        return True

    return False

def calculate_line_error(ir_values):
    """Calculate line following error based on 5 IR sensors."""
    error = 0
    weights = [-2, -1, 0, 1, 2] # Corresponds to ir_pins order [FarLeft, Left, Center, Right, FarRight]

    num_active_sensors = 0
    weighted_sum = 0

    for i, sensor_value in enumerate(ir_values):
        if sensor_value == 0: # Assuming 0 means line detected (dark line on light background)
            weighted_sum += weights[i]
            num_active_sensors += 1

    if num_active_sensors > 0:
        error = weighted_sum / num_active_sensors
    else:
        # If no line detected, error can be handled by the state machine
        return None
    return error

def execute_turn(current_yaw, target_yaw, base_speed, max_speed_val):
    """
    Execute a turn to reach the target yaw angle using proportional control.
    Returns left_speed, right_speed, turn_complete flag.
    """
    current_yaw = normalize_angle_rad(current_yaw)
    target_yaw = normalize_angle_rad(target_yaw)

    heading_error = target_yaw - current_yaw
    heading_error = normalize_angle_rad(heading_error) # Normalize error to [-pi, pi]

    turn_tolerance = 0.05 # Radians (approx 2.8 degrees)

    if abs(heading_error) < turn_tolerance:
        return 0, 0, True # Turn complete

    turn_kp = 1.0 # Proportional gain for turning speed
    
    # Calculate base turn speed proportional to the error
    raw_turn_speed = turn_kp * heading_error

    # Max turn speed
    max_abs_turn_speed = base_speed * 1.5 # Can turn faster than base speed
    # Min turn speed to ensure it actually moves when error is small
    min_abs_turn_speed = base_speed * 0.2

    # Apply limits
    # Cap the absolute raw_turn_speed to max_abs_turn_speed, then ensure it's at least min_abs_turn_speed if non-zero
    turn_speed = max(min_abs_turn_speed, min(max_abs_turn_speed, abs(raw_turn_speed)))

    # Determine direction
    if heading_error > 0:  # Turn left (increase yaw)
        left_speed_val = -turn_speed
        right_speed_val = turn_speed
    else:  # Turn right (decrease yaw)
        left_speed_val = turn_speed
        right_speed_val = -turn_speed
        
    # Translate speeds to PWM values. Assuming set_motor_speed handles positive PWM only.
    # A negative speed here means reverse, which we typically don't want for turning in place.
    # For turning in place, one wheel goes forward, one backward.
    # Let's simplify this for current motor control which uses 0/1 on IN2 for direction.
    # It's better to calculate differential drive based on target speeds.
    
    # For turning, one motor forward, one backward, or differential.
    # Here, we'll assume `set_motor_speed` takes a signed speed (0-1023 for forward, -(0-1023) for backward)
    # The current `set_motor_speed` function supports this.
    
    return left_speed_val, right_speed_val, False # Turn not complete

# ----- Main Control Loop -----
def run_robot_control():
    global yaw_angle, last_mpu_time
    global current_state, intersection_counter, no_intersection_counter, intersection_processed, turn_target_yaw, line_search_counter

    try:
        # Initial path (placeholder, as there's no path planning logic here yet)
        # In a full system, this would be computed by Dijkstra's.
        # For this example, we assume it's just line following with node detection.
        # If you were to add path planning, this is where 'self.path' would be initialized.

        while True:
            # --- Sensor Readings ---
            current_mpu_time = ticks_ms()
            dt_mpu = (current_mpu_time - last_mpu_time) / 1000.0 # Seconds
            last_mpu_time = current_mpu_time

            gyro = mpu.get_gyro()
            yaw_angle += gyro['z'] * dt_mpu * (pi / 180.0) # Integrate gyro Z to estimate yaw angle (convert deg/s to rad/s)
            yaw_angle = normalize_angle_rad(yaw_angle) # Keep yaw within [-pi, pi]

            dist = read_distance(trig, echo)
            button_pressed = button.value() == 1
            ir_values = [pin.value() for pin in ir_pins]
            
            # Activate electromagnet based on button state
            if button.value() == 0: # Button not pressed (pulled up, so 0 when pressed)
                electromagnet.off()
            else: # Button pressed (or floating/noise, assuming pull-up means 0 is pressed)
                electromagnet.on()

            # --- State Machine Logic ---
            left_speed = 0
            right_speed = 0
            
            # State: Line Following
            if current_state == 'line_following':
                if detect_intersection(ir_values): # If an intersection is detected
                    intersection_counter += 1
                    no_intersection_counter = 0 # Reset no-intersection counter
                    
                    if intersection_counter >= 3 and not intersection_processed: # Debounce: must detect for 3 cycles
                        print("Node detected! Transitioning to at_intersection.")
                        current_state = 'at_intersection'
                        intersection_processed = True
                        # Set a drive-through duration (e.g., for X number of cycles)
                        # This ensures the robot moves fully into the intersection.
                        intersection_counter = 15 # Drive through for 15 cycles (approx 1.5 seconds at 100ms sleep)
                else: # No intersection detected
                    no_intersection_counter += 1
                    if no_intersection_counter >= 2: # Reset if no intersection for 2 cycles
                        intersection_counter = 0
                        intersection_processed = False # Allow next intersection to be processed

                error = calculate_line_error(ir_values)
                if error is None: # Line lost
                    # print("Line lost! Searching...")
                    # Transition to a line search state or slow down significantly
                    left_speed = BASE_SPEED * 0.3
                    right_speed = BASE_SPEED * 0.3
                    # Or, more robustly, transition to 'post_turn_search' logic from thonny_explained_dirk.
                    # For simplicity, keeping it moving slowly for now.
                else:
                    correction = int(error * KP)
                    left_speed = BASE_SPEED - correction
                    right_speed = BASE_SPEED + correction
                    
                    # Clamp speeds to valid PWM range
                    left_speed = max(0, min(left_speed, MAX_SPEED_PWM))
                    right_speed = max(0, min(right_speed, MAX_SPEED_PWM))

            # State: At Intersection (driving through)
            elif current_state == 'at_intersection':
                # Drive slowly forward into the intersection
                left_speed = BASE_SPEED * 0.4
                right_speed = BASE_SPEED * 0.4
                
                if intersection_counter > 0:
                    intersection_counter -= 1 # Decrement drive-through counter
                else:
                    # Drive-through complete, now decide next action (e.g., turn or continue straight)
                    # For this simple line follower, let's assume it should attempt to find the line again.
                    # In a path planning scenario, this is where you'd compute the next turn.
                    print("Finished driving through intersection. Returning to line following or turning.")
                    # For now, transition back to line following. In a full system, this would be 'turning'
                    # or 'post_turn_search' if a specific turn angle is required.
                    current_state = 'line_following'
                    # Or, for a specific turn, you'd set turn_target_yaw here and transition to 'turning'
                    # Example: turn_target_yaw = normalize_angle_rad(yaw_angle + pi/2) # 90 degree left
                    # current_state = 'turning'

            # State: Turning (if we had specific turning logic)
            # You would enter this state from 'at_intersection' if a turn is needed
            # elif current_state == 'turning':
            #     ls, rs, complete = execute_turn(yaw_angle, turn_target_yaw, BASE_SPEED, MAX_SPEED_PWM)
            #     left_speed = ls
            #     right_speed = rs
            #     if complete:
            #         # print("Turn complete. Searching for line.")
            #         current_state = 'post_turn_search'
            #         line_search_counter = 0

            # State: Post Turn Search (if turn fails to land on line)
            # elif current_state == 'post_turn_search':
            #     error = calculate_line_error(ir_values)
            #     if error is not None: # Line detected after search
            #         # print("Line found after search. Resuming line following.")
            #         current_state = 'line_following'
            #         left_speed, right_speed = BASE_SPEED, BASE_SPEED # Start strong on line
            #     else:
            #         line_search_counter += 1
            #         if line_search_counter < 30: # Max search duration
            #             # Simple search pattern: oscillate
            #             search_speed = BASE_SPEED * 0.3
            #             if (line_search_counter // 5) % 2 == 0:
            #                 left_speed, right_speed = search_speed * 0.8, search_speed * 1.2 # Turn right
            #             else:
            #                 left_speed, right_speed = search_speed * 1.2, search_speed * 0.8 # Turn left
            #         else:
            #             # print("Line not found after extensive search. Stopping.")
            #             current_state = 'stopping' # Give up and stop

            # State: Stopping
            elif current_state == 'stopping':
                stop_motors()
                print("Robot is in STOPPING state.")
                # Could break here if it's a final stop for mission complete
                # For now, it will just continuously print stopping and keep motors off.
                sleep_ms(500) # Sleep to prevent rapid print/looping for stopped state
                continue # Skip remaining loop if stopped

            set_motor_speed(motor1_pwm, motor1_in2_pin, left_speed)
            set_motor_speed(motor2_pwm, motor2_in2_pin, right_speed)

            # --- Debugging Output ---
            # print("\n--- Robot State: {} ---".format(current_state))
            # print("IR Sensors:", ir_values)
            # print("Intersection Counter:", intersection_counter)
            # print("No Intersection Counter:", no_intersection_counter)
            # print("Intersection Processed:", intersection_processed)
            # print(f"Left Speed: {left_speed}, Right Speed: {right_speed}")
            # print("Encoder 1 Count:", position1)
            # print("Encoder 2 Count:", position2)
            # print("Distance: {:.2f} cm".format(dist) if dist != -1 else "Ultrasonic: Timeout")
            # print("Button Pressed:", button_pressed)
            # print("Yaw angle (deg): {:.2f}".format(yaw_angle * (180.0 / pi)))
            
            sleep_ms(100) # Control loop frequency

    except KeyboardInterrupt:
        # print("Program interrupted by user.")
        pass # To prevent the message if user specifically wants only node detection prints
    finally:
        stop_motors()
        # print("Motors stopped.")

# Start the robot control loop
run_robot_control()
