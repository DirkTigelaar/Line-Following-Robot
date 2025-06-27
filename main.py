# THIS CODE READS ALL THE SENSOR VALUES
# LINE FOLLOWING
# PATH FOLLOWING
# OBSTACLE AVOIDANCE
# BOX PICKUP AND DELIVERY
# ©GROUP 3

from machine import Pin, I2C, PWM
from time import sleep_us, sleep_ms, ticks_us, ticks_diff, sleep, ticks_ms
from math import atan2, sqrt, cos, pi # Import pi for radian calculations
import heapq # Import for Dijkstra's algorithm

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
REVERSE_SPEED = 400 # Adjusted speed for backward line following

def set_motor_speed(motor_pwm_obj, motor_in2_pin_obj, speed):
    """
    Sets the speed and direction of a motor.
    Args:
        motor_pwm_obj: PWM object for the motor's IN1 pin.
        motor_in2_pin_obj: Pin object for the motor's IN2 pin.
        speed: Integer speed value (-1023 to 1023). Positive for forward, negative for backward.
    """
    if speed >= 0: # Forward
        motor_in2_pin_obj.value(0)
        motor_pwm_obj.duty(min(speed, 1023))
    else: # Backward
        motor_in2_pin_obj.value(1)
        motor_pwm_obj.duty(min(abs(speed), 1023))

def stop_motors():
    """Stops both motors."""
    motor1_pwm.duty(0)
    motor2_pwm.duty(0)
    motor1_in2_pin.value(0)
    motor2_in2_pin.value(0)

# ----- MPU6050 Setup -----
class MPU6050:
    """Class for interacting with the MPU6050 IMU sensor."""
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')  # Wake up the MPU6050
        self.gyro_offset = {'x': 0, 'y': 0, 'z': 0}

    def _read_raw_data(self, reg):
        """Reads raw 6-byte data from a specified register."""
        data = self.i2c.readfrom_mem(self.addr, reg, 6)
        return (
            self._to_int16(data[0], data[1]),
            self._to_int16(data[2], data[3]),
            self._to_int16(data[4], data[5])
        )

    def _to_int16(self, high, low):
        """Converts two 8-bit bytes to a signed 16-bit integer."""
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value

    def get_accel(self):
        """Reads and returns accelerometer values (g's)."""
        ax, ay, az = self._read_raw_data(0x3B)
        return {
            'x': ax / 16384.0,
            'y': ay / 16384.0,
            'z': az / 16384.0
        }

    def get_gyro(self):
        """Reads and returns gyroscope values (degrees/second) with offset applied."""
        gx, gy, gz = self._read_raw_data(0x43)
        ox = self.gyro_offset['x']
        oy = self.gyro_offset['y']
        oz = self.gyro_offset['z']
        return {
            'x': (gx - ox) / 131.0, # Gyro output is degrees/second
            'y': (gy - oy) / 131.0,
            'z': (gz - oz) / 131.0
        }

    def calibrate_gyro(self, samples=100):
        """Calibrates the gyroscope by averaging sensor readings while stationary."""
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
    """
    Reads distance from the HC-SR04 ultrasonic sensor.
    Args:
        trig: Pin object for the trigger pin.
        echo: Pin object for the echo pin.
    Returns:
        Distance in cm, or -1 if timeout occurs.
    """
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

# Ultrasonic sensor
trig = Pin(19, Pin.OUT)
echo = Pin(23, Pin.IN)

# Button (Switch)
button = Pin(18, Pin.IN, Pin.PULL_UP)

# Electromagnet
electromagnet = Pin(5, Pin.OUT)
electromagnet.off()

# IR line sensors (assuming 5 sensors: left_outer, left_inner, center, right_inner, right_outer)
ir_pins = [
    Pin(39, Pin.IN), # Left-most
    Pin(4, Pin.IN),
    Pin(35, Pin.IN), # Center
    Pin(34, Pin.IN),
    Pin(36, Pin.IN)  # Right-most
]

# ----- Encoder Pins and Setup -----
pin_a1 = Pin(25, Pin.IN, Pin.PULL_UP)
pin_b1 = Pin(26, Pin.IN, Pin.PULL_UP)

position1 = 0
last_state1 = pin_a1.value()

pin_a2 = Pin(17, Pin.IN, Pin.PULL_UP)
pin_b2 = Pin(16, Pin.IN, Pin.PULL_UP)

position2 = 0
last_state2 = pin_a2.value()

def update_position1(pin):
    """Interrupt handler for encoder 1 (left wheel)."""
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
    """Interrupt handler for encoder 2 (right wheel)."""
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
# PID constants for line following (tuned for smoother performance)
KP_LF = 90   # Proportional gain (adjusted from 80)
KI_LF = 0.5  # Integral gain (small value to reduce steady-state error)
KD_LF = 250  # Derivative gain (adjusted from 200, higher value for dampening oscillations)
MAX_CORRECTION = 200 # Max correction applied to speed

# ----- Node Detection Parameters ---
# Threshold for number of active sensors to consider it a node
NODE_SENSOR_THRESHOLD = 3
# Time to stop at a node (in ms)
NODE_STOP_TIME_MS = 750
# Cooldown period for node detection (in ms)
NODE_DETECTION_COOLDOWN_MS = 2000 # 2 seconds

# ----- Obstacle Detection Parameters ---
OBSTACLE_DISTANCE_CM = 10 # Distance threshold for obstacle detection in cm

# ----- Path Planning Grid (Corrected and Weighted) -----
corrected_weighted_grid = {
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

# --- Dijkstra's Algorithm ---
def find_path_dijkstra(start_node, goal_node, grid_map_weighted, blocked_nodes=None):
    """
    Find shortest path between nodes using Dijkstra's algorithm with weighted edges.
    Args:
        start_node (str): The starting node.
        goal_node (str): The target goal node.
        grid_map_weighted (dict): A dictionary representing the weighted graph.
        blocked_nodes (set, optional): A set of nodes that are blocked/unreachable. Defaults to None.
    Returns:
        list: A list of nodes representing the shortest path from start_node to goal_node,
              or an empty list if no path is found or input nodes are invalid.
    """
    if blocked_nodes is None:
        blocked_nodes = set()

    if start_node not in grid_map_weighted or goal_node not in grid_map_weighted:
        print("Error: Invalid nodes - Start: {}, Goal: {}".format(start_node, goal_node))
        return []

    distances = {node: float('infinity') for node in grid_map_weighted}
    distances[start_node] = 0

    # Priority queue: (distance, current_node, path_to_current_node)
    priority_queue = [(0, start_node, [start_node])]

    while priority_queue:
        current_distance, current_node_pq, path_pq = heapq.heappop(priority_queue)

        # If we've found a shorter path to this node already, skip
        if current_distance > distances[current_node_pq]:
            continue

        # If goal reached, return the path
        if current_node_pq == goal_node:
            return path_pq

        # Explore neighbors
        for direction, neighbor_info in grid_map_weighted[current_node_pq].items():
            if neighbor_info: # Check if there's a valid connection
                neighbor, weight = neighbor_info
                if neighbor not in blocked_nodes:
                    distance = current_distance + weight
                    # If a shorter path to the neighbor is found
                    if distance < distances[neighbor]:
                        distances[neighbor] = distance
                        new_path = path_pq + [neighbor]
                        heapq.heappush(priority_queue, (distance, neighbor, new_path))

    print(f"No path found from {start_node} to {goal_node} with blocked nodes: {blocked_nodes}")
    return []

# --- Helper functions for orientation (now using radians) ---
def normalize_angle_rad(angle_rad):
    """Normalizes an angle to be within the -pi to pi radian range."""
    normalized = angle_rad % (2 * pi)
    if normalized > pi:
        normalized -= (2 * pi)
    elif normalized < -pi:
        normalized += (2 * pi)
    return normalized

def get_shortest_angle_difference_rad(angle1_rad, angle2_rad):
    """
    Calculates the shortest angular difference between two angles (in radians, -pi to pi).
    Returns a value in [-pi, pi]. Positive means angle2 is clockwise from angle1.
    """
    diff = normalize_angle_rad(angle2_rad) - normalize_angle_rad(angle1_rad)
    if diff > pi:
        diff -= (2 * pi)
    elif diff < -pi:
        diff += (2 * pi)
    return diff

def get_direction_between_nodes(node1, node2, grid):
    """
    Determines the cardinal direction from node1 to node2 based on the grid.
    Returns 'N', 'S', 'E', 'W' or None.
    """
    if node1 not in grid or node2 not in grid:
        return None

    for direction, info in grid[node1].items():
        if info and info[0] == node2:
            return direction
    return None

def block_path_segment_by_weight(node1, node2, grid, new_weight=1000.0):
    """
    Increases the weight of the path segment between node1 and node2 in the grid
    to effectively block it for future path calculations. It also attempts to block
    the reverse path for symmetry, assuming obstacles block both directions.
    Args:
        node1 (str): The starting node of the segment.
        node2 (str): The ending node of the segment.
        grid (dict): The weighted grid map (will be modified).
        new_weight (float): The high weight to assign to the blocked path.
    Returns:
        bool: True if at least one segment was found and its weight increased, False otherwise.
    """
    found_and_blocked = False
    
    # Block path from node1 to node2
    for direction, info in grid[node1].items():
        if info and info[0] == node2:
            grid[node1][direction] = (node2, new_weight)
            print(f"Path from {node1} to {node2} ({direction}) weight increased to {new_weight}")
            found_and_blocked = True
            break
    
    # Block path from node2 to node1 (for bidirectional segments)
    for direction, info in grid[node2].items():
        if info and info[0] == node1:
            grid[node2][direction] = (node1, new_weight)
            print(f"Path from {node2} to {node1} ({direction}) weight increased to {new_weight}")
            found_and_blocked = True
            break
    return found_and_blocked

# --- Robust Node Detection Logic ---
def is_node_detected_robust(ir_values, num_active_sensors):
    """
    Checks if a node is detected based on IR sensor patterns.
    Args:
        ir_values (list): List of 0s and 1s from IR sensors (0 = line, 1 = no line).
                          Assumes 5 sensors: [L_outer, L_inner, Center, R_inner, R_outer]
        num_active_sensors (int): Count of sensors detecting the line.
    Returns:
        bool: True if a node is detected, False otherwise.
    """
    # Pattern 1: A significant number of sensors are active, indicating a wide line or intersection.
    # This is useful for '+' intersections or very wide 'T' junctions.
    if num_active_sensors >= NODE_SENSOR_THRESHOLD: # NODE_SENSOR_THRESHOLD is 3
        return True

    # Pattern 2: Detects a clear T-junction or 90-degree turn to the left.
    # Left-outer, left-inner, and center sensors are on the line.
    # This implies the robot is approaching a left turn from the main line.
    if ir_values[0] == 0 and ir_values[1] == 0 and ir_values[2] == 0:
        return True
    
    # Pattern 3: Detects a clear T-junction or 90-degree turn to the right.
    # Right-outer, right-inner, and center sensors are on the line.
    # This implies the robot is approaching a right turn from the main line.
    if ir_values[2] == 0 and ir_values[3] == 0 and ir_values[4] == 0:
        return True
    
    # Pattern 4: All sensors are on the line. This is a strong indicator of a major intersection.
    if ir_values == [0, 0, 0, 0, 0]:
        return True

    # Pattern for a distinct side road to the left (leftmost and center on line, rightmost off)
    if ir_values[0] == 0 and ir_values[2] == 0 and ir_values[4] == 1:
        return True
    
    # Pattern for a distinct side road to the right (rightmost and center on line, leftmost off)
    if ir_values[4] == 0 and ir_values[2] == 0 and ir_values[0] == 1:
        return True
    
    # Another pattern for wider lines/junctions where inner sensors are active along with center,
    # suggesting widening of the path or a junction forming.
    if (ir_values[1] == 0 and ir_values[2] == 0 and ir_values[3] == 0): # Inner three sensors active
        return True

    return False

# --- Mission Control Parameters ---
# Define the pickup and delivery sequence
# Each tuple: (pickup_node, delivery_node)
MISSION_PLAN = [
    ("P1", "P5"),
    ("P2", "P6"),
    ("P3", "P7"),
    ("P4", "P8"),
]

# Global state variables for the mission
MISSION_STATE_PICKUP = 0
MISSION_STATE_DELIVER = 1
MISSION_STATE_COMPLETE = 2

current_mission_idx = 0
current_mission_state = MISSION_STATE_PICKUP # Start by picking up the first box
current_pickup_node = ""
current_delivery_node = ""

# Global variables for yaw calculation and node detection cooldown
yaw_angle = 0.0 # Yaw angle in radians
last_time = ticks_ms() # This 'last_time' is now primarily for orient_robot and overall system time.
last_node_detection_time = 0 # Initialize for cooldown
obstacle_detected_flag = False # New global flag for obstacle detection

# Global variables to manage path
calculated_path = [] # Will be populated initially and recalculated
current_path_idx = 0
current_target_node = "" # The node the robot is currently heading towards in its path

# Cardinal directions and their target yaw angles (in radians)
# Assumes 'E' (East) is 0 radians, 'N' (North) is pi/2, 'S' (South) is -pi/2, 'W' (West) is pi.
TARGET_YAW_ANGLES = {
    'N': pi / 2,     # 90 degrees
    'E': 0.0,        # 0 degrees
    'S': -pi / 2,    # -90 degrees or 270 degrees
    'W': pi          # 180 degrees
}

# PD control constants for orientation turns
KP_TURN = 250 # Proportional gain
KD_TURN = 600 # Derivative gain (adjusted from 500, tune carefully to avoid jittering)
TURN_SPEED = 600 # Maximum speed for turning (used for clamping)

# Yaw tolerance: 1.5 degrees on each side
YAW_TOLERANCE = 1.5 * (pi / 180.0) # Converted from 1.5 degrees to radians for precision

# Global variables for PID line following
last_error_lf = 0
integral_lf = 0

def orient_robot(target_yaw_radians, spin_in_place=True):
    """
    Orients the robot to a target yaw angle using proportional-derivative (PD) control.
    Args:
        target_yaw_radians (float): The desired yaw angle in radians (-pi to pi).
        spin_in_place (bool): If True, spins in place (one wheel forward, one backward).
                              For this modification, we will force spin_in_place to be True.
    """
    global yaw_angle, last_time, last_angle_diff_turn # Need to track last angle diff for derivative

    print(f"Orienting robot to {target_yaw_radians:.2f} rad ({target_yaw_radians * 180/pi:.2f} deg) (Spin in Place)...")
    
    target_yaw_radians = normalize_angle_rad(target_yaw_radians)

    turn_count = 0
    max_turn_attempts = 750 # Increased attempts for safety if turn is slow


    # Re-initialize last_time and last_angle_diff_turn for this specific turn operation for accurate dt and derivative
    last_time_turn_loop = ticks_ms()
    last_angle_diff_turn = get_shortest_angle_difference_rad(normalize_angle_rad(yaw_angle), target_yaw_radians)


    while abs(get_shortest_angle_difference_rad(yaw_angle, target_yaw_radians)) > YAW_TOLERANCE and turn_count < max_turn_attempts:
        current_yaw_normalized = normalize_angle_rad(yaw_angle)
        angle_diff = get_shortest_angle_difference_rad(current_yaw_normalized, target_yaw_radians)

        current_time_loop = ticks_ms()
        dt_loop = (current_time_loop - last_time_turn_loop) / 1000.0 # Time in seconds
        
        # Calculate derivative term for turn (rate of change of error)
        derivative_turn = 0
        if dt_loop > 0:
            derivative_turn = (angle_diff - last_angle_diff_turn) / dt_loop
        last_angle_diff_turn = angle_diff # Update last angle diff for next iteration
        last_time_turn_loop = current_time_loop # Update local last_time

        # Proportional-Derivative control for turning
        # The further away from target, the higher the proportional power
        # The derivative term helps to dampen oscillations (negative sign)
        turn_power = int((angle_diff * KP_TURN) - (derivative_turn * KD_TURN))

        # Clamp turn_power to ensure it's within motor limits and has a minimum to move
        # We take absolute value for clamping, then apply direction.
        abs_turn_power = abs(turn_power)
        abs_turn_power = max(50, min(abs_turn_power, TURN_SPEED)) 

        if angle_diff > 0: # Need to turn counter-clockwise (Left) - Left wheel back, Right wheel forward
            set_motor_speed(motor1_pwm, motor1_in2_pin, -abs_turn_power)
            set_motor_speed(motor2_pwm, motor2_in2_pin, abs_turn_power)
        else: # Need to turn clockwise (Right) - Left wheel forward, Right wheel back
            set_motor_speed(motor1_pwm, motor1_in2_pin, abs_turn_power)
            set_motor_speed(motor2_pwm, motor2_in2_pin, -abs_turn_power)

        # Update yaw angle
        gyro = mpu.get_gyro()
        # Note: dt_loop for gyro integration is already updated above.
        yaw_angle += (gyro['z'] * (pi / 180.0)) * dt_loop
        yaw_angle = normalize_angle_rad(yaw_angle) # Keep yaw within -pi to pi

        sleep_ms(1) # Smaller sleep time for faster control loop within turn
        turn_count += 1

    stop_motors()
    print(f"Robot oriented. Final Yaw: {normalize_angle_rad(yaw_angle):.2f} rad ({normalize_angle_rad(yaw_angle) * 180/pi:.2f} deg)")
    if turn_count >= max_turn_attempts:
        print("Warning: Orientation reached max turn attempts. Might not be perfectly aligned.")

    # Add a short sleep at the end to allow for physical settling
    sleep(0.1) 

    # After orientation, ensure global last_time is updated to prevent large dt jump in main loop
    # This `last_time` is less critical now as `dt_loop_lf` handles PID timing.
    # However, keeping it updated is good practice for any other global time-dependent operations.
    last_time = ticks_ms() 

def perform_180_turn():
    """Performs a 180-degree spin in place."""
    global yaw_angle
    print("Performing 180-degree turn...")
    target_yaw = normalize_angle_rad(yaw_angle + pi) # Add pi (180 degrees) to current yaw
    orient_robot(target_yaw)
    print("180-degree turn complete.")

# ----- Line Following Control Loop -----
def run_line_follower():
    """Main loop for line following and node detection and path navigation, now incorporating mission logic."""
    global yaw_angle, current_path_idx, last_node_detection_time, obstacle_detected_flag, \
           calculated_path, current_mission_idx, current_mission_state, \
           current_pickup_node, current_delivery_node, current_target_node, \
           last_error_lf, integral_lf # Declare global for PID

    # Define weights locally within the function to ensure scope
    weights = [-2, -1, 0, 1, 2] # Weights for error calculation

    # Initialize the first mission target
    if MISSION_PLAN:
        current_pickup_node, current_delivery_node = MISSION_PLAN[current_mission_idx]
        print(f"\nStarting mission: Pick up from {current_pickup_node}")
        
        # Initial path calculation: From A1 to P1. The bot starts at A1 facing P1.
        # This implies the first segment is A1 -> P1.
        calculated_path = find_path_dijkstra("A1", current_pickup_node, corrected_weighted_grid) 
        print(f"Initial path to pickup {current_pickup_node}: {calculated_path}")
        if calculated_path:
            # Robot is at calculated_path[0]. Its first target is calculated_path[1].
            if len(calculated_path) > 1:
                current_path_idx = 1 # Index of the first actual target node in the path
                current_target_node = calculated_path[current_path_idx] 
                # Orient from the starting node (calculated_path[0]) towards the first target node (calculated_path[1])
                current_node_for_orient = calculated_path[0]
                direction_to_next = get_direction_between_nodes(current_node_for_orient, current_target_node, corrected_weighted_grid)
                if direction_to_next and direction_to_next in TARGET_YAW_ANGLES:
                    target_yaw = TARGET_YAW_ANGLES[direction_to_next]
                    print(f"Initial orientation from {current_node_for_orient} towards {current_target_node} ({direction_to_next}). Target Yaw: {target_yaw * 180/pi:.2f} deg")
                    orient_robot(target_yaw)
                else:
                    print(f"Warning: Initial orientation failed from {current_node_for_orient} to {current_target_node}.")
            else: # Path is just one node (e.g., if A1 is pickup node and robot is already there)
                current_path_idx = 0 # No movement needed, already at target
                current_target_node = calculated_path[0]
            print(f"Initial Next Node to Target: {current_target_node}")
        else:
            print("ERROR: No path to initial pickup node. Stopping.")
            stop_motors()
            return

    else:
        print("No mission plan defined. Exiting.")
        stop_motors()
        return

    # Initialize local last_time for precise dt calculation within line follower loop
    last_loop_time = ticks_ms() # New local variable for dt calculation within this loop

    try:
        while True:
            # Initialize speeds, error, and correction for printing, default to 0
            left_speed = 0
            right_speed = 0
            error = 'N/A'
            correction = 'N/A'

            # Calculate dt for the current loop iteration
            current_loop_time = ticks_ms()
            dt_loop_lf = (current_loop_time - last_loop_time) / 1000.0 # Time in seconds for PID
            last_loop_time = current_loop_time # Update for the next iteration

            # Update global yaw_angle using a consistent dt
            # This is critical. The dt should reflect the time elapsed since the last gyro reading.
            # Here, we'll use dt_loop_lf for gyro integration too, ensuring consistency.
            gyro = mpu.get_gyro()
            yaw_angle += (gyro['z'] * (pi / 180.0)) * dt_loop_lf
            yaw_angle = normalize_angle_rad(yaw_angle) # Keep yaw within -pi to pi
            
            # Check if all missions are complete
            if current_mission_state == MISSION_STATE_COMPLETE:
                print("\n*** ALL MISSIONS COMPLETE! Robot is idle. ***")
                stop_motors()
                # Optional: return to a specific "home" node here, or just stop.
                while True:
                    sleep(1) # Keep robot stopped
            
            # Ultrasonic distance
            dist = read_distance(trig, echo)
            
            # --- Obstacle Detection Logic ---
            is_obstacle_currently_present = (dist != -1 and dist < OBSTACLE_DISTANCE_CM)

            if is_obstacle_currently_present:
                if not obstacle_detected_flag: # Obstacle just appeared
                    obstacle_detected_flag = True
                    stop_motors() # Ensure motors are stopped before turning
                    print("\n!!! OBSTACLE DETECTED! Performing 180-degree spin in place and recalculating path. !!!")

                    # Determine the segment that is blocked by the obstacle
                    if current_path_idx < len(calculated_path):
                        # The robot is currently trying to reach calculated_path[current_path_idx].
                        # The segment before it is the one just traversed or the current start.
                        if current_path_idx == 0:
                            blocked_segment_start_node = calculated_path[0] # Assuming this is the 'from' node for the first segment
                            if len(calculated_path) > 1:
                                blocked_segment_end_node = calculated_path[1]
                            else: # If path is just one node (start and goal are the same or only goal is left)
                                print("Warning: Path is too short to determine a blocked segment.")
                                blocked_segment_start_node = None
                                blocked_segment_end_node = None
                        else:
                            blocked_segment_start_node = calculated_path[current_path_idx - 1]
                            blocked_segment_end_node = calculated_path[current_path_idx]
                    else: # Robot is at or past the goal
                        print("Warning: Obstacle detected but robot is at or beyond goal node's index. Cannot block a specific segment.")
                        stop_motors()
                        sleep(5)
                        return # Exit to avoid errors or endless loop

                    if blocked_segment_start_node and blocked_segment_end_node:
                        # Increase the weight of the path segment to effectively block it
                        if not block_path_segment_by_weight(blocked_segment_start_node, blocked_segment_end_node, corrected_weighted_grid, new_weight=1000.0):
                             print(f"Warning: Could not find or block segment from {blocked_segment_start_node} to {blocked_segment_end_node}.")
                    else:
                        print("Warning: Could not determine valid segment to block due to path length or index issues. Proceeding with recalculation anyway.")
                        
                    perform_180_turn()
                    
                    # Recalculate path from the last successfully reached node to the goal.
                    # This needs to be the node we were *approaching* or *at* when the obstacle was detected.
                    current_robot_node = calculated_path[current_path_idx] if current_path_idx < len(calculated_path) else calculated_path[-1]
                    
                    # Recalculate the path to the current mission target
                    if current_mission_state == MISSION_STATE_PICKUP:
                        target_goal_node = current_pickup_node
                    else: # MISSION_STATE_DELIVER
                        target_goal_node = current_delivery_node

                    new_calculated_path = find_path_dijkstra(current_robot_node, target_goal_node, corrected_weighted_grid)
                    
                    if not new_calculated_path:
                        print("CRITICAL ERROR: No alternative path found after obstacle. Stopping.")
                        stop_motors()
                        return # Exit the function, robot is stuck
                    else:
                        print(f"New path calculated: {new_calculated_path}")
                        calculated_path = new_calculated_path
                        # After 180 turn, robot is at calculated_path[0] (which is the old current_robot_node).
                        # Its first target is now calculated_path[1].
                        if len(calculated_path) > 1:
                            current_path_idx = 1 # Index of the first actual target node in the new path
                            current_target_node = calculated_path[current_path_idx]
                            
                            current_node_for_orient = calculated_path[0] # The node robot is currently at
                            direction_to_next = get_direction_between_nodes(current_node_for_orient, current_target_node, corrected_weighted_grid)
                            if direction_to_next and direction_to_next in TARGET_YAW_ANGLES:
                                target_yaw = TARGET_YAW_ANGLES[direction_to_next]
                                print(f"Orienting from {current_node_for_orient} towards {current_target_node} ({direction_to_next}). Target Yaw: {target_yaw * 180/pi:.2f} deg")
                                orient_robot(target_yaw)
                            print(f"Obstacle path recalculation: Next Node to Target set to {current_target_node}") # Debug print
                        else: # Path is just one node (robot is already at the target)
                            current_path_idx = 0
                            current_target_node = calculated_path[0]
                            print(f"Obstacle path recalculation: Next Node to Target set to {current_target_node} (single node path)") # Debug print
                        
                        obstacle_detected_flag = False # Reset flag to allow normal operation
            
            # Handle obstacle clearance *after* any potential re-calculation and re-orientation
            if not is_obstacle_currently_present and obstacle_detected_flag:
                print("--- Obstacle cleared. Resuming. ---")
                obstacle_detected_flag = False
            
            # This main block only executes if no obstacle is currently preventing movement
            if not obstacle_detected_flag:
                button_pressed = button.value() == 0
                
                # --- Button Pressed Logic for Pickup ---
                # This applies when the robot is going to a pickup node AND the button is pressed.
                # The P nodes are dead ends, so the bot should drive against the box, activate the switch,
                # pick up, turn, and then re-plan.
                if button_pressed and current_mission_state == MISSION_STATE_PICKUP:
                    # We assume if the button is pressed while in PICKUP state, and we are on a P-node path,
                    # we have reached the box.
                    print(f"Button pressed at a pickup point! Current mission pickup node: {current_pickup_node}")
                    # If the button is pressed while we are trying to reach the current_pickup_node
                    # (which will be the final node in the current path segment for pickup).
                    # This checks if the *last* node we were heading towards in the current path was the pickup node.
                    if calculated_path and calculated_path[current_path_idx] == current_pickup_node: # Check if current target is pickup node
                        print(f"Confirmed at pickup node {current_pickup_node}. Activating electromagnet, picking up box.")
                        electromagnet.on()  # Turn electromagnet ON
                        stop_motors()       # Stop the robot
                        sleep(3)            # Wait for 3 seconds to ensure pickup
                        
                        # After pickup, transition to deliver state
                        current_mission_state = MISSION_STATE_DELIVER
                        print(f"Box picked up. Now moving to deliver to {current_delivery_node}.")
                        
                        # Drive backward, line following, until a node is detected
                        print("Driving backward, line following to detect connecting node from P-node...")
                        start_reverse_time = ticks_ms()
                        reversed_to_node = False
                        
                        # Reset PID for backward line following
                        last_error_lf = 0
                        integral_lf = 0
                        last_loop_time_reverse = ticks_ms() # New local variable for dt during reverse

                        while ticks_diff(ticks_ms(), start_reverse_time) < 3000: # Max 3 seconds reverse
                            current_time_loop_rev = ticks_ms()
                            dt_loop_rev = (current_time_loop_rev - last_loop_time_reverse) / 1000.0
                            last_loop_time_reverse = current_time_loop_rev # Update local last_time

                            ir_values_reverse = [pin.value() for pin in ir_pins]
                            num_active_sensors_reverse = sum(1 for val in ir_values_reverse if val == 0)

                            # Update yaw angle during backward movement as well
                            gyro_reverse = mpu.get_gyro()
                            yaw_angle += (gyro_reverse['z'] * (pi / 180.0)) * dt_loop_rev
                            yaw_angle = normalize_angle_rad(yaw_angle)

                            if num_active_sensors_reverse > 0: # Line detected, apply line following
                                weighted_sum_reverse = sum(weights[i] for i, val in enumerate(ir_values_reverse) if val == 0)
                                current_error_reverse = weighted_sum_reverse # Changed to direct sum
                                
                                # PID calculation for backward line following
                                integral_lf += current_error_reverse * dt_loop_rev
                                derivative_lf = (current_error_reverse - last_error_lf) / dt_loop_rev if dt_loop_rev > 0 else 0
                                last_error_lf = current_error_reverse

                                correction_reverse = int((KP_LF * current_error_reverse) + (KI_LF * integral_lf) + (KD_LF * derivative_lf))
                                # Clamp correction
                                correction_reverse = max(-MAX_CORRECTION, min(correction_reverse, MAX_CORRECTION))

                                # Corrected logic for backward line following:
                                left_speed_rev = -REVERSE_SPEED - correction_reverse
                                right_speed_rev = -REVERSE_SPEED + correction_reverse

                                # Ensure speeds are within valid range (0-1023 magnitude)
                                left_speed_rev = max(-1023, min(left_speed_rev, 0))
                                right_speed_rev = max(-1023, min(right_speed_rev, 0))

                                set_motor_speed(motor1_pwm, motor1_in2_pin, left_speed_rev)
                                set_motor_speed(motor2_pwm, motor2_in2_pin, right_speed_rev)
                            else: # Line lost while reversing, attempt to find by just backing up straight
                                print("Line lost while reversing! Attempting straight backward movement.")
                                set_motor_speed(motor1_pwm, motor1_in2_pin, -REVERSE_SPEED)
                                set_motor_speed(motor2_pwm, motor2_in2_pin, -REVERSE_SPEED)
                                # Reset integral and derivative if line is lost
                                integral_lf = 0
                                last_error_lf = 0
                            
                            if is_node_detected_robust(ir_values_reverse, num_active_sensors_reverse):
                                print("Backed up onto a node. Stopping reverse line following.")
                                reversed_to_node = True
                                break
                            sleep_ms(5) # Smaller sleep for more responsive backward line following
                        stop_motors()
                        if not reversed_to_node:
                            print("Warning: Did not detect a node while reversing from pickup. Might be off track.")
                        
                        # Drive forward for a short duration to compensate for sensor placement
                        print("Driving forward for 1.5 seconds to center on intersection...")
                        set_motor_speed(motor1_pwm, motor1_in2_pin, BASE_SPEED)
                        set_motor_speed(motor2_pwm, motor2_in2_pin, BASE_SPEED)
                        sleep(1.5) # Adjusted duration to 1.5 seconds
                        stop_motors()
                        print("Centered on intersection.")

                        # Reset PID for forward line following after stopping at node
                        last_error_lf = 0
                        integral_lf = 0
                        last_loop_time = ticks_ms() # Reset loop time after this pause

                        # The robot is now logically at the node from which it entered the P-node dead-end.
                        # For P1, it's A1; for P2, it's A2, etc.
                        # Get the "entry" node to the P-node from the grid
                        entry_node_to_pickup = None
                        for direction, info in corrected_weighted_grid[current_pickup_node].items():
                            if info and info[0] in corrected_weighted_grid: # Check if the connected node exists in the main grid
                                entry_node_to_pickup = info[0]
                                break
                        
                        if entry_node_to_pickup:
                            # Recalculate path from this entry node to the delivery node
                            calculated_path = find_path_dijkstra(entry_node_to_pickup, current_delivery_node, corrected_weighted_grid)
                            if calculated_path:
                                # After reversing, the robot is at calculated_path[0] (which is entry_node_to_pickup).
                                # Its first target for this new path is calculated_path[1].
                                if len(calculated_path) > 1:
                                    current_path_idx = 1 # Index of the first actual target node in the new path
                                    current_target_node = calculated_path[current_path_idx]
                                    
                                    current_node_for_orient = calculated_path[0] # The node robot is currently at
                                    direction_to_next = get_direction_between_nodes(current_node_for_orient, current_target_node, corrected_weighted_grid)
                                    if direction_to_next and direction_to_next in TARGET_YAW_ANGLES:
                                        target_yaw = TARGET_YAW_ANGLES[direction_to_next]
                                        print(f"Orienting from {current_node_for_orient} towards {current_target_node} ({direction_to_next}). Target Yaw: {target_yaw * 180/pi:.2f} deg")
                                        orient_robot(target_yaw)
                                    print(f"Pickup Path Recalculation: Next Node to Target set to {current_target_node}") # Debug print
                                else:
                                    current_path_idx = 0 # Single node path (e.g., if delivery node is same as entry node)
                                    current_target_node = calculated_path[0]
                                    print(f"Path to {current_delivery_node} is just one node. No immediate turn needed.")
                                    print(f"Pickup Path Recalculation: Next Node to Target set to {current_target_node}") # Debug print


                                print(f"Path to delivery {current_delivery_node}: {calculated_path}")

                            else:
                                print(f"ERROR: No path found from {entry_node_to_pickup} to {current_delivery_node}. Stopping.")
                                stop_motors()
                                return # Critical error, cannot complete mission
                        else:
                            print(f"ERROR: Could not determine entry node for {current_pickup_node}. Stopping.")
                            stop_motors()
                            return

                    else:
                        print(f"Button pressed, but not at the designated pickup node ({current_pickup_node}). Ignoring.")

                ir_values = [pin.value() for pin in ir_pins] # Read all IR sensor values
                
                num_active_sensors = 0
                weighted_sum = 0

                # Calculate error based on active IR sensors
                for i, sensor_value in enumerate(ir_values):
                    if sensor_value == 0: # Assuming 0 means line detected (dark line on light background)
                        weighted_sum += weights[i]
                        num_active_sensors += 1

                # --- Node Detection Logic with Cooldown ---
                if is_node_detected_robust(ir_values, num_active_sensors):
                    if (current_loop_time - last_node_detection_time) >= NODE_DETECTION_COOLDOWN_MS:
                        # The robot just arrived at calculated_path[current_path_idx]
                        arrived_node_name = calculated_path[current_path_idx]
                        
                        stop_motors()
                        print(f"\n*** NODE DETECTED! Arrived at {arrived_node_name}. Stopping briefly. ***")
                        sleep_ms(NODE_STOP_TIME_MS) # Uses the updated NODE_STOP_TIME_MS
                        
                        # Drive forward for 1 second to clear the node (important for small bots)
                        print("Driving forward for 1 second to clear node with wheels...")
                        set_motor_speed(motor1_pwm, motor1_in2_pin, BASE_SPEED)
                        set_motor_speed(motor2_pwm, motor2_in2_pin, BASE_SPEED)
                        sleep(1) # Drive forward for 1 second
                        stop_motors() # Stop after driving forward

                        last_node_detection_time = current_loop_time # Update last detection time
                        last_loop_time = ticks_ms() # Reset loop time after this pause

                        # Reset PID for forward line following after stopping at node
                        last_error_lf = 0
                        integral_lf = 0

                        # Check if the node we *just arrived at* was the mission stage goal.
                        target_goal_for_mission_stage = ""
                        if current_mission_state == MISSION_STATE_PICKUP:
                            target_goal_for_mission_stage = current_pickup_node
                        elif current_mission_state == MISSION_STATE_DELIVER:
                            target_goal_for_mission_stage = current_delivery_node
                        
                        if arrived_node_name == target_goal_for_mission_stage:
                            print(f"*** Arrived at mission stage goal: {arrived_node_name} ***")
                            stop_motors()
                            
                            if current_mission_state == MISSION_STATE_DELIVER:
                                # Logic for delivery completion
                                print(f"Arrived at delivery node {current_delivery_node}. Deactivating electromagnet.")
                                electromagnet.off() # Turn electromagnet OFF
                                sleep(2) # Give some time for box to settle
                                
                                # Drive backward, line following, until a node is detected
                                print("Driving backward, line following to detect connecting node from P-node (delivery)...")
                                start_reverse_time_delivery = ticks_ms()
                                reversed_to_node_delivery = False
                                
                                # Reset PID for backward line following
                                last_error_lf = 0
                                integral_lf = 0
                                last_loop_time_reverse_delivery = ticks_ms() # New local variable for dt during reverse

                                while ticks_diff(ticks_ms(), start_reverse_time_delivery) < 3000: # Max 3 seconds reverse
                                    current_time_loop_rev_del = ticks_ms()
                                    dt_loop_rev_del = (current_time_loop_rev_del - last_loop_time_reverse_delivery) / 1000.0
                                    last_loop_time_reverse_delivery = current_time_loop_rev_del # Update local last_time

                                    ir_values_reverse_delivery = [pin.value() for pin in ir_pins]
                                    num_active_sensors_reverse_delivery = sum(1 for val in ir_values_reverse_delivery if val == 0)

                                    # Update yaw angle during backward movement as well
                                    gyro_reverse_delivery = mpu.get_gyro()
                                    yaw_angle += (gyro_reverse_delivery['z'] * (pi / 180.0)) * dt_loop_rev_del
                                    yaw_angle = normalize_angle_rad(yaw_angle)

                                    if num_active_sensors_reverse_delivery > 0:
                                        weighted_sum_reverse_delivery = sum(weights[i] for i, val in enumerate(ir_values_reverse_delivery) if val == 0)
                                        current_error_reverse_delivery = weighted_sum_reverse_delivery # Changed to direct sum
                                        
                                        # PID calculation for backward line following
                                        integral_lf += current_error_reverse_delivery * dt_loop_rev_del
                                        derivative_lf = (current_error_reverse_delivery - last_error_lf) / dt_loop_rev_del if dt_loop_rev_del > 0 else 0
                                        last_error_lf = current_error_reverse_delivery

                                        correction_reverse_delivery = int((KP_LF * current_error_reverse_delivery) + (KI_LF * integral_lf) + (KD_LF * derivative_lf))
                                        # Clamp correction
                                        correction_reverse_delivery = max(-MAX_CORRECTION, min(correction_reverse_delivery, MAX_CORRECTION))

                                        # Corrected logic for backward line following:
                                        left_speed_rev_del = -REVERSE_SPEED - correction_reverse_delivery
                                        right_speed_rev_del = -REVERSE_SPEED + correction_reverse_delivery

                                        left_speed_rev_del = max(-1023, min(left_speed_rev_del, 0))
                                        right_speed_rev_del = max(-1023, min(right_speed_rev_del, 0))

                                        set_motor_speed(motor1_pwm, motor1_in2_pin, left_speed_rev_del)
                                        set_motor_speed(motor2_pwm, motor2_in2_pin, right_speed_rev_del)
                                    else:
                                        print("Line lost while reversing from delivery! Attempting straight backward movement.")
                                        set_motor_speed(motor1_pwm, motor1_in2_pin, -REVERSE_SPEED)
                                        set_motor_speed(motor2_pwm, motor2_in2_pin, -REVERSE_SPEED)
                                        # Reset integral and derivative if line is lost
                                        integral_lf = 0
                                        last_error_lf = 0

                                    if is_node_detected_robust(ir_values_reverse_delivery, num_active_sensors_reverse_delivery):
                                        print("Backed up onto a node from delivery. Stopping reverse line following.")
                                        reversed_to_node_delivery = True
                                        break
                                    sleep_ms(5) # Smaller sleep for more responsive backward line following
                                stop_motors()
                                if not reversed_to_node_delivery:
                                    print("Warning: Did not detect a node while reversing from delivery. Might be off track.")
                                
                                # Drive forward for a short duration to compensate for sensor placement
                                print("Driving forward for 1.5 seconds to center on intersection (delivery)...")
                                set_motor_speed(motor1_pwm, motor1_in2_pin, BASE_SPEED)
                                set_motor_speed(motor2_pwm, motor2_in2_pin, BASE_SPEED)
                                sleep(1.5) # Adjusted duration to 1.5 second
                                stop_motors()
                                print("Centered on intersection (delivery).")

                                # Reset PID for forward line following after stopping at node
                                last_error_lf = 0
                                integral_lf = 0
                                last_loop_time = ticks_ms() # Reset loop time after this pause

                                # Move to next mission or complete
                                current_mission_idx += 1
                                if current_mission_idx < len(MISSION_PLAN):
                                    current_pickup_node, current_delivery_node = MISSION_PLAN[current_mission_idx]
                                    current_mission_state = MISSION_STATE_PICKUP # Go to next pickup
                                    print(f"Delivery complete. Next mission: Pick up from {current_pickup_node}.")
                                    
                                    # Recalculate path to the next pickup node from the node we just reversed from
                                    # (e.g., if we delivered to P5, we are now at E3 after reversing).
                                    entry_node_from_delivery = None
                                    for direction, info in corrected_weighted_grid[target_goal_for_mission_stage].items():
                                        if info and info[0] in corrected_weighted_grid:
                                            entry_node_from_delivery = info[0]
                                            break

                                    if entry_node_from_delivery:
                                        calculated_path = find_path_dijkstra(entry_node_from_delivery, current_pickup_node, corrected_weighted_grid)
                                        if calculated_path:
                                            # New path starts from `entry_node_from_delivery`.
                                            # Its first target is calculated_path[1].
                                            if len(calculated_path) > 1:
                                                current_path_idx = 1 # New path, so first target is index 1
                                                current_target_node = calculated_path[current_path_idx]
                                                current_node_for_orient = calculated_path[0] # Node just left
                                                direction_to_next = get_direction_between_nodes(current_node_for_orient, current_target_node, corrected_weighted_grid)
                                                if direction_to_next and direction_to_next in TARGET_YAW_ANGLES:
                                                    target_yaw = TARGET_YAW_ANGLES[direction_to_next]
                                                    print(f"Orienting from {current_node_for_orient} towards {current_target_node} ({direction_to_next}). Target Yaw: {target_yaw * 180/pi:.2f} deg")
                                                    orient_robot(target_yaw)
                                                print(f"Delivery Next Mission Path Recalculation: Next Node to Target set to {current_target_node}") # Debug print
                                            else:
                                                current_path_idx = 0
                                                current_target_node = calculated_path[0]
                                                print(f"Delivery Next Mission Path Recalculation: Next Node to Target set to {current_target_node} (single node path)") # Debug print
                                            print(f"Path to next pickup {current_pickup_node}: {calculated_path}")
                                        else:
                                            print(f"ERROR: No path found from {entry_node_from_delivery} to {current_pickup_node}. Stopping.")
                                            current_mission_state = MISSION_STATE_COMPLETE # Cannot continue
                                            stop_motors()
                                            return
                                    else:
                                        print(f"ERROR: Could not determine entry node for {target_goal_for_mission_stage}. Stopping.")
                                        current_mission_state = MISSION_STATE_COMPLETE
                                        stop_motors()
                                        return

                                else:
                                    print("All boxes delivered! Mission complete.")
                                    current_mission_state = MISSION_STATE_COMPLETE # All missions done
                                    stop_motors()
                                    return # Exit function, robot is done
                            # The pickup logic is now triggered by the button, not by reaching the node alone
                            # So, if we arrive at a pickup node (but didn't press button yet), we just continue line following.
                            # The button handler will then manage the transition.

                        # If the arrived node was NOT a mission stage goal, just an intermediate node
                        # Only increment current_path_idx if it's not the end of the path
                        elif current_path_idx + 1 < len(calculated_path): 
                            current_path_idx += 1 # Advance to the next target node
                            current_target_node = calculated_path[current_path_idx]
                            
                            # For orientation, we need the node we just came from and the node we are going to
                            current_node_for_orient = calculated_path[current_path_idx - 1] # Node just arrived at
                            next_node_for_orient = calculated_path[current_path_idx] # New target node
                            
                            direction_to_next = get_direction_between_nodes(current_node_for_orient, next_node_for_orient, corrected_weighted_grid)
                            
                            if direction_to_next and direction_to_next in TARGET_YAW_ANGLES:
                                target_yaw = TARGET_YAW_ANGLES[direction_to_next]
                                print(f"Current node: {current_node_for_orient}, Next path segment target: {next_node_for_orient}")
                                print(f"Required turn direction: {direction_to_next}, Target Yaw: {target_yaw * 180/pi:.2f} deg")
                                orient_robot(target_yaw)
                            print(f"Node Detection & Orientation: Next Node to Target set to {current_target_node}")
                        else:
                            # This means current_path_idx was already the last index, and we just arrived at the final node.
                            # This should be handled by the 'arrived_node_name == target_goal_for_mission_stage' block.
                            # If not, it means the path ended without reaching a mission goal, which is an error state.
                            print("End of current calculated path reached. Waiting for next mission step or full mission complete (unhandled case).")
                            stop_motors()
                        
                        # After node handling, line following will resume naturally
                        # or the robot will stay stopped if at the goal.

                # Continue with line following if not at goal, not handling obstacle, and not waiting for button
                if not button_pressed or not (current_mission_state == MISSION_STATE_PICKUP and calculated_path[current_path_idx] == current_pickup_node):
                    if num_active_sensors > 0:
                        error = weighted_sum # Changed to direct sum

                        # PID calculation for forward line following
                        integral_lf += error * dt_loop_lf # Use local dt
                        derivative_lf = (error - last_error_lf) / dt_loop_lf if dt_loop_lf > 0 else 0
                        last_error_lf = error # This ensures error tracking is consistent

                        correction = int((KP_LF * error) + (KI_LF * integral_lf) + (KD_LF * derivative_lf))
                        # Clamp correction
                        correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
                    else: # If line is lost, try to continue moving forward
                        print("Line lost! Continuing forwards, attempting to find line.")
                        # Reset integral and derivative if line is lost
                        integral_lf = 0
                        last_error_lf = 0
                        # Removed the 'continue' here to allow printing of state
                        # The sleep_ms(5) at the end of the loop will provide a small delay.

                    # Corrected logic for forward line following (only apply if not line lost and actively following)
                    if num_active_sensors > 0:
                        left_speed = BASE_SPEED + correction
                        right_speed = BASE_SPEED - correction
                    else: # If line lost, keep previous BASE_SPEED as set above
                        left_speed = BASE_SPEED
                        right_speed = BASE_SPEED

                    # Ensure speeds are within valid range (0-1023)
                    left_speed = max(0, min(left_speed, 1023))
                    right_speed = max(0, min(right_speed, 1023))

                    set_motor_speed(motor1_pwm, motor1_in2_pin, left_speed)
                    set_motor_speed(motor2_pwm, motor2_in2_pin, right_speed)

            else:
                # If obstacle_detected_flag is True, stop motors
                stop_motors()
                sleep_ms(50) # Small delay to prevent busy-waiting while waiting for obstacle to clear


            # Print current robot state for debugging
            print("\n--- Current Robot State ---")
            print("IR Sensors:", ir_values)
            print("Error:", error)
            print("Correction:", correction)
            # Now `left_speed` and `right_speed` will always be defined
            print(f"Left Speed: {left_speed}, Right Speed: {right_speed}")
            print("Encoder 1 Count:", position1)
            print("Encoder 2 Count:", position2)
            print("Distance: {:.2f} cm".format(dist) if dist != -1 else "Ultrasonic: Timeout")
            print("Button Pressed:", button_pressed)
            print("Yaw angle (deg): {:.2f}".format(yaw_angle * 180/pi))
            print("Obstacle Detected Flag:", obstacle_detected_flag)
            print("Current Mission Index:", current_mission_idx)
            print("Current Mission State:", "PICKUP" if current_mission_state == MISSION_STATE_PICKUP else "DELIVER" if current_mission_state == MISSION_STATE_DELIVER else "COMPLETE")
            print("Current Pickup Node:", current_pickup_node)
            print("Current Delivery Node:", current_delivery_node)
            print("Current Node Index in Path:", current_path_idx) # Changed label
            print("Next Node to Target in Path:", current_target_node) # Changed label
            if calculated_path:
                print("Current Path:", calculated_path)
            
            sleep_ms(5) # Reduced sleep for more responsive main loop
            # This sleep is global for the main loop, so it affects sensor reading frequency
            # and motor updates for both forward line following and obstacle checking.

    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        stop_motors()
        print("Motors stopped.")

# --- Main execution starts here ---
# 1. Stop motors immediately upon script start
stop_motors()

# 2. Initialize MPU6050 and calibrate gyro (while motors are stopped)
mpu = MPU6050(i2c)
mpu.calibrate_gyro()

# Set the initial yaw angle to North (pi/2 radians)
# This assumes the robot is always placed at A1 facing P1 (North).
yaw_angle = pi / 2

# Start the line following loop (robot will start moving after path is calculated and printed)
run_line_follower()

