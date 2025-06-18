# THIS CODE READS ALL THE SENSOR VALUES
# LINE FOLLOWING BEHAVIOUR WITH NODE DETECTION AND DIJKSTRA PATH FOLLOWING

from machine import Pin, I2C, PWM
from time import sleep_us, sleep_ms, ticks_us, ticks_diff, sleep, ticks_ms
from math import atan2, sqrt, cos, pi
import heapq

# ----- Path Finding -----
weighted_grid = {
    # Parking spots (P nodes)
    "P1": {"S": ("A1", 2.0)},
    "P2": {"S": ("A2", 2.0)},
    "P3": {"S": ("A3", 2.0)},
    "P4": {"S": ("A4", 2.0)},
    "P5": {"N": ("E3", 2.0)},
    "P6": {"N": ("E4", 2.0)},
    "P7": {"N": ("E5", 2.0)},
    "P8": {"N": ("E6", 2.0)},

    # A row
    "A1": {"N": ("P1", 2.0), "E": ("A2", 1.0), "S": ("C1", 2.5)},
    "A2": {"N": ("P2", 2.0), "E": ("A3", 1.0), "W": ("A1", 1.0)},
    "A3": {"N": ("P3", 2.0), "E": ("A4", 1.0), "W": ("A2", 1.0)},
    "A4": {"N": ("P4", 2.0), "E": ("A5", 1.0), "W": ("A3", 1.0)},
    "A5": {"N": None, "E": ("A6", 4.0), "S": ("B1", 1.0), "W": ("A4", 1.0)},
    "A6": {"N": None, "E": None, "S": ("B2", 1.5), "W": ("A5", 4.0)},

    # B row
    "B1": {"N": ("A5", 1.5), "E": ("B2", 4), "S": ("C2", 1.0)},
    "B2": {"N": ("A6", 1.5), "E": None, "S": ("C3", 1.0), "W": ("B1", 4.0)},

    # C row
    "C1": {"N": ("A1", 2.5), "E": ("C2", 4.0), "S": ("D1", 1.0)},
    "C2": {"N": ("B1", 1.0), "E": ("C3", 4.0), "S": ("D2", 1.0), "W": ("C1", 4.0)},
    "C3": {"N": ("B2", 1.0), "E": None, "S": ("E6", 2.5), "W": ("C2", 4.0)},

    # D row
    "D1": {"N": ("C1", 1.0), "E": ("D2", 4.0), "S": ("E1", 1.5)},
    "D2": {"N": ("C2", 1.0), "E": None, "S": ("E2", 1.5), "W": ("D1", 4.0)},

    # E row
    "E1": {"N": ("D1", 1.5), "E": ("E2", 4.0), "S": None, "W": None},
    "E2": {"N": ("D2", 1.5), "E": ("E3", 1.0), "S": None, "W": ("E1", 4.0)},
    "E3": {"S": ("P5", 2.0), "E": ("E4", 1.0), "N": None, "W": ("E2", 1.0)},
    "E4": {"S": ("P6", 2.0), "E": ("E5", 1.0), "N": None, "W": ("E3", 1.0)},
    "E5": {"S": ("P7", 2.0), "E": ("E6", 1.0), "N": None, "W": ("E4", 1.0)},
    "E6": {"N": ("C3", 2.5), "E": None, "S": ("P8", 2.0), "W": ("E5", 1.0)},
}

# Node coordinates and initial robot orientation (facing North at C1)
# Assumed grid layout where North is positive Y, East is positive X
node_coords = {
    "P1": (0, 7), "P2": (1, 7), "P3": (2, 7), "P4": (3, 7),
    "A1": (0, 6), "A2": (1, 6), "A3": (2, 6), "A4": (3, 6), "A5": (4, 6), "A6": (8, 6),
    "B1": (4, 5), "B2": (8, 5),
    "C1": (0, 4), "C2": (4, 4), "C3": (8, 4),
    "D1": (0, 3), "D2": (4, 3),
    "E1": (0, 2), "E2": (4, 2), "E3": (5, 2), "E4": (6, 2), "E5": (7, 2), "E6": (8, 2),
    "P5": (5, 0), "P6": (6, 0), "P7": (7, 0), "P8": (8, 0)
}

# Mapping direction strings to yaw changes in radians (assuming initial facing North = 0 radians)
# North (N) = 0 radians
# East (E) = pi/2 radians (90 degrees)
# South (S) = pi radians (180 degrees)
# West (W) = -pi/2 radians (-90 degrees)
direction_to_yaw_change = {
    "N": 0,
    "E": pi / 2,
    "S": pi,
    "W": -pi / 2
}

def find_path_dijkstra(graph, start, goal):
    dist = {node: float('inf') for node in graph}
    dist[start] = 0

    prev = {node: None for node in graph}
    # Stores (distance, node, direction_to_get_here)
    priority_queue = [(0, start, None)]

    while priority_queue:
        current_dist, current_node, dir_to_here = heapq.heappop(priority_queue)

        if current_dist > dist[current_node]:
            continue
        if current_node == goal:
            path = []
            current = goal
            while current is not None:
                path.insert(0, (current, prev[current]['direction'] if prev[current] else None))
                current = prev[current]['node'] if prev[current] else None
            # The first element will have None for direction, remove it or handle
            return path[1:] if path else [] # Return (node, direction_to_next_node)
        
        for direction, neighbor_info in graph[current_node].items():
            if neighbor_info is not None:
                neighbor_node, weight = neighbor_info
                new_dist = current_dist + weight
                if new_dist < dist[neighbor_node]:
                    dist[neighbor_node] = new_dist
                    prev[neighbor_node] = {'node': current_node, 'direction': direction}
                    heapq.heappush(priority_queue, (new_dist, neighbor_node, direction))
    return None

def get_bearing_to_next_node(current_node_id, next_node_id, current_orientation_rad):
    """
    Calculates the target yaw angle (in radians) the robot needs to face to go
    from current_node_id to next_node_id.

    Args:
        current_node_id (str): The ID of the current node.
        next_node_id (str): The ID of the next node in the path.
        current_orientation_rad (float): The robot's current yaw angle in radians.

    Returns:
        float: The target yaw angle in radians.
    """
    if current_node_id not in node_coords or next_node_id not in node_coords:
        print(f"Error: Node coordinates not found for {current_node_id} or {next_node_id}")
        return current_orientation_rad # Remain at current orientation

    x1, y1 = node_coords[current_node_id]
    x2, y2 = node_coords[next_node_id]

    dx = x2 - x1
    dy = y2 - y1

    # Debugging print for dx, dy
    print(f"  get_bearing_to_next_node: From {current_node_id} ({x1},{y1}) to {next_node_id} ({x2},{y2})")
    print(f"  Calculated dx: {dx}, dy: {dy}")

    # Calculate target angle using atan2(dy, dx) - standard mathematical angle
    # Where 0 is positive x-axis (East), pi/2 is positive y-axis (North)
    raw_target_angle = atan2(dy, dx)
    
    # Convert from (East=0, North=pi/2) to your convention (North=0, East=pi/2)
    # Target yaw will be pi/2 - raw_target_angle
    target_yaw_rad = pi/2 - raw_target_angle
    
    return normalize_angle_rad(target_yaw_rad)


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
    # Ensure speed is within PWM range [-1023, 1023]
    # Positive speed for forward, negative for backward
    
    if speed >= 0: # Forward
        motor_in2_pin_obj.value(0) # IN2 low for forward
        motor_pwm_obj.duty(min(int(speed), MAX_SPEED_PWM))
    else: # Backward
        motor_in2_pin_obj.value(1) # IN2 high for backward
        motor_pwm_obj.duty(min(int(abs(speed)), MAX_SPEED_PWM))

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

# Global variables for yaw calculation (from MPU6050)
yaw_angle = pi / 2 # Radians. Robot starts facing East (90 degrees) based on your convention (North=0, East=pi/2)
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
path_index = 0 # To track current position in the planned path
planned_path = [] # Stores the sequence of (node, direction_to_get_here) from Dijkstra

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
    line_center = ir_values[2] == 0
    line_left = ir_values[1] == 0
    line_right = ir_values[3] == 0
    line_far_left = ir_values[0] == 0
    line_far_right = ir_values[4] == 0

    # A robust intersection detection: if the center sensor and at least
    # one sensor on each side (left/right) detect the line.
    if line_center and (line_left or line_far_left) and (line_right or line_far_right):
        return True
    
    # Also consider if all 5 sensors are on the line (very clear intersection)
    if all(s == 0 for s in ir_values):
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

    turn_tolerance = 0.05 # Radians (approx 3 degrees) - Made stricter for better precision

    # Check for turn completion first
    if abs(heading_error) < turn_tolerance:
        return 0, 0, True # Turn complete

    turn_kp = 1500.0 # Proportional gain for turning speed - Adjusted to be more aggressive for turning
    
    raw_turn_speed = turn_kp * abs(heading_error) # Calculate speed based on absolute error

    max_abs_turn_speed = max_speed_val 
    min_abs_turn_speed_threshold = base_speed * 0.5 

    turn_speed = raw_turn_speed
    if turn_speed < min_abs_turn_speed_threshold:
        turn_speed = min_abs_turn_speed_threshold
    
    turn_speed = min(turn_speed, max_abs_turn_speed)

    # Determine direction of turn and set motor speeds
    # If heading_error is positive, target_yaw is "ahead" (counter-clockwise)
    # If heading_error is negative, target_yaw is "behind" (clockwise)
    if heading_error > 0:  # Robot needs to turn counter-clockwise (increase yaw)
        # Left wheel backward, Right wheel forward (differential turn)
        left_speed_val = -turn_speed 
        right_speed_val = turn_speed
    else:  # Robot needs to turn clockwise (decrease yaw)
        # Left wheel forward, Right wheel backward (differential turn)
        left_speed_val = turn_speed
        right_speed_val = -turn_speed
            
    return left_speed_val, right_speed_val, False # Turn not complete


# ----- Main Control Loop -----
def run_robot_control(start_node, goal_node):
    global yaw_angle, last_mpu_time
    global current_state, intersection_counter, no_intersection_counter, intersection_processed, turn_target_yaw, line_search_counter
    global planned_path, path_index

    # Plan the path using Dijkstra
    path_with_directions = find_path_dijkstra(weighted_grid, start_node, goal_node)
    if path_with_directions:
        planned_path = path_with_directions
        print(f"Planned path: {planned_path}")
    else:
        print(f"No path found from {start_node} to {goal_node}. Stopping.")
        current_state = 'stopping' # Can't proceed without a path

    try:
        while True:
            # --- Sensor Readings ---
            current_mpu_time = ticks_ms()
            dt_mpu = (current_mpu_time - last_mpu_time) / 1000.0 # Seconds
            last_mpu_time = current_mpu_time

            gyro = mpu.get_gyro()
            # Integrate gyro Z to estimate yaw angle (convert deg/s to rad/s)
            yaw_angle += gyro['z'] * dt_mpu * (pi / 180.0) 
            yaw_angle = normalize_angle_rad(yaw_angle) # Keep yaw within [-pi, pi]

            dist = read_distance(trig, echo)
            button_pressed = button.value() == 0 # Button is pulled up, so 0 when pressed
            ir_values = [pin.value() for pin in ir_pins]
            
            # Activate electromagnet based on button state
            if button_pressed:
                electromagnet.on()
            else:
                electromagnet.off()

            # --- State Machine Logic ---
            left_speed = 0
            right_speed = 0
            
            # State: Line Following
            if current_state == 'line_following':
                if path_index >= len(planned_path): # Check if path is completed (or nearing completion)
                    print("Path completed! Transitioning to stopping.")
                    current_state = 'stopping'
                    continue

                if detect_intersection(ir_values): # If an intersection is detected
                    intersection_counter += 1
                    no_intersection_counter = 0 # Reset no-intersection counter
                    
                    # Debounce: must detect for 3 cycles and not already processed
                    if intersection_counter >= 3 and not intersection_processed: 
                        # Get the node ID from the current planned_path entry. This is the node the robot just arrived at.
                        # The path_index points to the *segment* from current_node to next_node.
                        # So, planned_path[path_index] is (next_node, direction_to_get_there_from_previous_node).
                        # The node we just arrived at is implicitly the 'previous' node of the current segment.
                        # A better way to track current node might be to have a separate 'current_robot_node' variable.
                        # For now, let's assume planned_path[path_index][0] is the *next* node we are heading towards.
                        # This means when we detect an intersection, we have arrived at the node that *was* the next_node.
                        
                        # Let's re-think this: when path_index is 0, planned_path[0] is (first_node_after_start, direction).
                        # When we hit the first intersection, we are at `start_node` and need to go to `planned_path[0][0]`.
                        # So, if we are at an intersection, the node we *just arrived at* is the one that was `planned_path[path_index-1][0]`
                        # if path_index > 0.
                        # Or, even simpler: if we are at an intersection, and we are about to make a decision,
                        # the node we are AT is the `source` for the next path segment.
                        # So, when entering `at_intersection` state, the node we are *at* is the one *before* planned_path[path_index][0].

                        # This logic needs refinement. A simple approach:
                        # planned_path[path_index] = (node_we_are_GOING_TO, direction_to_get_there)
                        # So, when we hit an intersection, we have ARRIVED at planned_path[path_index][0].
                        
                        # Let's use the explicit `current_node_id_in_path` logic used in the `at_intersection` state.
                        # At this point, if path_index is `X`, it means we are driving along the segment
                        # which will lead us *to* the node that is `planned_path[X][0]`.
                        # So, the node we are about to process (turn at/pass through) is `planned_path[path_index][0]`.
                        
                        # For clearer node tracking, let's say the current node the robot *thinks* it is at.
                        # This variable will be updated AFTER a turn or decision at an intersection.
                        # We will make this explicit.
                        
                        print(f"Intersection detected! Preparing to process next segment.")
                        current_state = 'at_intersection'
                        intersection_processed = True
                        intersection_counter = 10 # Drive through for X cycles (adjust as needed)
                else: # No intersection detected
                    no_intersection_counter += 1
                    if no_intersection_counter >= 2: # Reset if no intersection for 2 cycles
                        intersection_counter = 0
                        intersection_processed = False # Allow next intersection to be processed

                error = calculate_line_error(ir_values)
                if error is None: # Line lost
                    print("Line lost! Searching...")
                    # Slow down and try to find the line
                    left_speed = BASE_SPEED * 0.3
                    right_speed = BASE_SPEED * 0.3
                    # Potentially transition to a more aggressive line search state
                    # For now, it just slows down. Could add a dedicated 'line_lost_search' state.
                else:
                    correction = int(error * KP)
                    left_speed = BASE_SPEED - correction
                    right_speed = BASE_SPEED + correction
                    
                    # Clamp speeds to valid PWM range
                    left_speed = max(0, min(left_speed, MAX_SPEED_PWM))
                    right_speed = max(0, min(right_speed, MAX_SPEED_PWM))

            # State: At Intersection (driving through)
            elif current_state == 'at_intersection':
                # Continue driving slowly into the intersection
                left_speed = BASE_SPEED * 0.4
                right_speed = BASE_SPEED * 0.4
                
                if intersection_counter > 0:
                    intersection_counter -= 1 # Decrement drive-through counter
                else:
                    # Drive-through complete, now determine next action based on path
                    
                    # The current_node_id represents the node the robot has just arrived at.
                    # If path_index is 0, we are at start_node, and planning to go to planned_path[0][0].
                    # If path_index > 0, we have just completed the segment that led to planned_path[path_index-1][0].
                    # So, the current node we are "at" is the node from the *previous* path segment,
                    # or the start_node if it's the very beginning.

                    # Let's define the current_node_for_logic clearly:
                    current_node_for_logic = start_node if path_index == 0 else planned_path[path_index-1][0]
                    
                    # If this is the last step in the planned_path, then current_node_for_logic is our goal.
                    # Check if the *next* segment would take us beyond the planned path.
                    if path_index >= len(planned_path):
                        print(f"Reached final destination: {current_node_for_logic}. Stopping.")
                        current_state = 'stopping'
                        continue

                    # The next node in the sequence is `planned_path[path_index][0]`
                    next_node_id_in_path, _ = planned_path[path_index]

                    # Now, find the direction from current_node_for_logic to next_node_id_in_path
                    direction_to_take = None
                    for direction, neighbor_info in weighted_grid[current_node_for_logic].items():
                        if neighbor_info and neighbor_info[0] == next_node_id_in_path:
                            direction_to_take = direction
                            break
                    
                    if direction_to_take is None:
                        print(f"Error: Could not determine direction from {current_node_for_logic} to {next_node_id_in_path}. Stopping.")
                        current_state = 'stopping'
                        continue

                    print(f"Finished driving through node {current_node_for_logic}. Next node in path: {next_node_id_in_path}. Direction to take: {direction_to_take}")
                    
                    # Calculate target yaw for the next segment using coordinates
                    turn_target_yaw = get_bearing_to_next_node(current_node_for_logic, next_node_id_in_path, yaw_angle)
                    
                    print(f"Current Yaw: {yaw_angle * 180/pi:.2f} deg, Target Yaw: {turn_target_yaw * 180/pi:.2f} deg")
                    current_state = 'turning'
                    
                    # IMPORTANT: Increment path_index *after* we've used the current segment's info
                    # for the turn. This path_index now points to the segment *after* the turn.
                    path_index += 1


            # State: Turning
            elif current_state == 'turning':
                ls, rs, complete = execute_turn(yaw_angle, turn_target_yaw, BASE_SPEED, MAX_SPEED_PWM)
                left_speed = ls
                right_speed = rs

                # Debugging print for turning state
                error_deg = normalize_angle_rad(turn_target_yaw - yaw_angle) * (180.0 / pi)
                print(f"  Turning... Heading Error (deg): {error_deg:.2f}")

                if complete:
                    print(f"Turn complete. Current Yaw: {yaw_angle * 180/pi:.2f} deg. Searching for line.")
                    current_state = 'post_turn_search'
                    line_search_counter = 0 # Reset search counter

            # State: Post Turn Search (if turn fails to land on line)
            elif current_state == 'post_turn_search':
                error = calculate_line_error(ir_values)
                if error is not None: # Line detected after search
                    print("Line found after search. Resuming line following.")
                    current_state = 'line_following'
                    left_speed, right_speed = BASE_SPEED, BASE_SPEED # Start strong on line
                else:
                    line_search_counter += 1
                    if line_search_counter < 50: # Max search duration (adjust based on robot speed and environment)
                        # Simple search pattern: oscillate
                        search_speed = BASE_SPEED * 0.3
                        # Alternate turning left and right to find the line
                        if (line_search_counter % 20) < 10: # Turn one way for 10 cycles
                             left_speed, right_speed = search_speed * 0.5, search_speed * 1.5 # Turn right
                        else: # Turn other way for 10 cycles
                            left_speed, right_speed = search_speed * 1.5, search_speed * 0.5 # Turn left
                    else:
                        print("Line not found after extensive search. Stopping.")
                        current_state = 'stopping' # Give up and stop

            # State: Stopping
            elif current_state == 'stopping':
                stop_motors()
                print("Robot is in STOPPING state.")
                break # Exit the while loop
            
            set_motor_speed(motor1_pwm, motor1_in2_pin, left_speed)
            set_motor_speed(motor2_pwm, motor2_in2_pin, right_speed)

            # --- Debugging Output ---
            print(f"\n--- Robot State: {current_state} ---")
            print("IR Sensors:", ir_values)
            print(f"Left Speed: {left_speed}, Right Speed: {right_speed}")
            print(f"Yaw angle (deg): {yaw_angle * (180.0 / pi):.2f}")
            if current_state == 'turning':
                print(f"Target Yaw (deg): {turn_target_yaw * (180.0 / pi):.2f}")
            # print("Encoder 1 Count:", position1)
            # print("Encoder 2 Count:", position2)
            # print("Distance: {:.2f} cm".format(dist) if dist != -1 else "Ultrasonic: Timeout")
            # print("Button Pressed:", button_pressed)
            
            sleep_ms(50) # Control loop frequency

    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        stop_motors()
        print("Motors stopped.")

# ----- Path finding example and robot run -----
start_node = "C1"
goal_node = "B2" # Example goal node

print(f"Calculating shortest path from {start_node} to {goal_node}...")
# The path is now calculated inside run_robot_control
# and stored in the global planned_path variable.

# Start the robot control loop
run_robot_control(start_node, goal_node)


