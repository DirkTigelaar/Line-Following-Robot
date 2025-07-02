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
# Updated based on user's correction
ir_pins = [
    Pin(39, Pin.IN), # Left-most
    Pin(36, Pin.IN),
    Pin(34, Pin.IN), # Center
    Pin(35, Pin.IN),
    Pin(4, Pin.IN)   # Right-most
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
KP_LF = 120   # Proportional gain (adjusted from 80)
KI_LF = 0.5  # Integral gain (small value to reduce steady-state error)
KD_LF = 250  # Derivative gain (adjusted from 200, higher value for dampening oscillations)
MAX_CORRECTION = 300 # Max correction applied to speed

# PID constants for orientation turns
KP_TURN = 280 # Proportional gain for turning (slightly increased for responsiveness)
KI_TURN = 8   # Integral gain for turning (increased for better drift correction)
KD_TURN = 650 # Derivative gain for turning (increased for better damping)

# New PID constants for reverse line following
KP_LF_REV = 100 # Proportional gain for reverse line following (can be different from forward)
KI_LF_REV = 0.8 # Integral gain for reverse line following (slightly increased)
KD_LF_REV = 300 # Derivative gain for reverse line following (slightly increased)


# ----- Node Detection Parameters ---
# Threshold for number of active sensors to consider it a node
NODE_SENSOR_THRESHOLD = 3
# Time to stop at a node (in ms)
NODE_STOP_TIME_MS = 500
# Cooldown period for node detection (in ms)
NODE_DETECTION_COOLDOWN_MS = 1500 # 2 seconds

# ----- Obstacle Detection Parameters ---
OBSTACLE_DISTANCE_CM = 10 # Distance threshold for obstacle detection in cm
OBSTACLE_DEBOUNCE_COUNT = 3 # Number of consecutive readings to confirm an obstacle

# Printing interval for debug information
PRINT_INTERVAL_MS = 200 # Print debug info every 200 milliseconds

# --- Global Timings for Driving After Nodes/Maneuvers ---
# Duration to drive forward after detecting a general node to clear it with wheels
NODE_CLEAR_DRIVE_TIME_SEC = 1.0
# Duration to drive forward after reversing from a pickup P-node to center on intersection
PICKUP_REVERSE_CENTER_DRIVE_TIME_SEC = 1.5
# Duration to drive forward after reversing from an obstacle to center on intersection
OBSTACLE_REVERSE_CENTER_DRIVE_TIME_SEC = 1.5
# Fixed duration to drive into a delivery P-node
DELIVERY_DRIVE_INTO_PNODE_TIME_SEC = 1.0


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

# Removed get_shortest_angle_difference_rad as it's for absolute yaw
# The new orient_robot will directly measure the relative turn.

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

# New helper: get the opposite direction
def get_opposite_direction(direction):
    """Returns the opposite cardinal direction."""
    if direction == 'N': return 'S'
    if direction == 'S': return 'N'
    if direction == 'E': return 'W'
    if direction == 'W': return 'E'
    return None

# Map cardinal directions to their radian values (for turn calculations)
# 'E' is 0, 'N' is pi/2, 'W' is pi or -pi, 'S' is -pi/2
CARDINAL_RADIAN_MAP = {
    'N': pi / 2,
    'E': 0.0,
    'S': -pi / 2,
    'W': pi # Or -pi, mathematically equivalent
}

def calculate_relative_turn_angle(current_orientation_cardinal, desired_orientation_cardinal):
    """
    Calculates the relative angle to turn from current_orientation to desired_orientation.
    Returns angle in radians (e.g., pi/2 for a 90-degree left turn, -pi/2 for right).
    """
    current_rad = CARDINAL_RADIAN_MAP.get(current_orientation_cardinal)
    desired_rad = CARDINAL_RADIAN_MAP.get(desired_orientation_cardinal)

    if current_rad is None or desired_rad is None:
        print(f"Error: Invalid cardinal orientation provided: Current={current_orientation_cardinal}, Desired={desired_orientation_cardinal}")
        return 0.0
    # Calculate difference, then normalize to -pi to pi
    diff = desired_rad - current_rad
    return normalize_angle_rad(diff)


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
    # Corrected: iterate over grid[node2] not grid[2]
    if node2 in grid: 
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

# Global variable for obstacle detection
obstacle_detected_flag = False
obstacle_readings_count = 0 # Counter for consecutive obstacle readings

# Global variables to manage path
calculated_path = [] # Will be populated initially and recalculated
current_path_idx = 0
current_target_node = "" # The node the robot is currently heading towards in its path

# NEW GLOBAL: Robot's current cardinal orientation (e.g., 'N', 'E', 'S', 'W')
current_robot_orientation = 'N' # Initial assumption: Robot starts facing North at A1.

# Global variables for PID line following
last_error_lf = 0
integral_lf = 0

# Global variables for PID orientation
last_error_turn = 0
integral_turn = 0

# Last time a debug print was made
last_print_time = 0

# Global for node detection cooldown
last_node_detection_time = 0


# --- NEW FUNCTION FOR STRAIGHT DRIVE ---
def drive_straight_for_time(speed, duration_seconds):
    """
    Drives both motors straight forward at a constant speed for a given duration.
    This bypasses PID control to ensure straight movement for centering.
    Args:
        speed (int): The forward speed (0-1023).
        duration_seconds (float): The time in seconds to drive.
    """
    print(f"Driving straight at {speed} for {duration_seconds} seconds...")
    set_motor_speed(motor1_pwm, motor1_in2_pin, speed)
    set_motor_speed(motor2_pwm, motor2_in2_pin, speed)
    sleep(duration_seconds)
    stop_motors()
    print("Straight drive complete.")

# --- MODIFIED orient_robot FUNCTION ---
def orient_robot(relative_angle_radians):
    """
    Orients the robot by a specified relative angle using proportional-integral-derivative (PID) control.
    Args:
        relative_angle_radians (float): The desired relative angle to turn in radians.
                                        Positive for counter-clockwise (left), negative for clockwise (right).
    """
    global last_error_turn, integral_turn, current_robot_orientation

    print(f"Turning by {relative_angle_radians:.2f} rad ({relative_angle_radians * 180/pi:.2f} deg)...")
    
    # Initialize terms for this specific turn operation
    last_time_turn_loop = ticks_ms()
    integral_turn = 0
    total_turned_angle = 0.0 # Accumulate the actual angle turned

    turn_count = 0
    max_turn_attempts = 3000 # Increased attempts for robustness

    # PID calculation for turning
    # The error is now the difference between the remaining angle to turn and the current angular velocity.
    # For relative turn, we simply want to make total_turned_angle reach relative_angle_radians
    
    # Define a smaller tolerance for stopping the turn, as we are accumulating error
    TURN_TOLERANCE_RAD = 5.0 * (pi / 180.0) # 1 degree tolerance

    while abs(relative_angle_radians - total_turned_angle) > TURN_TOLERANCE_RAD and turn_count < max_turn_attempts:
        current_time_loop = ticks_ms()
        dt_loop = (current_time_loop - last_time_turn_loop) / 1000.0 # Time in seconds
        last_time_turn_loop = current_time_loop

        if dt_loop <= 0:
            sleep_ms(1)
            continue

        gyro = mpu.get_gyro()
        # Integrate only the Z-axis gyroscope reading to get angle turned
        # Gyro provides deg/s, convert to rad/s for integration
        current_angular_velocity_rad_s = gyro['z'] * (pi / 180.0)
        total_turned_angle += current_angular_velocity_rad_s * dt_loop

        # Error is the remaining angle to turn
        current_error_turn = relative_angle_radians - total_turned_angle
        
        integral_turn += current_error_turn * dt_loop
        derivative_turn = (current_error_turn - last_error_turn) / dt_loop
        last_error_turn = current_error_turn

        turn_power = int((current_error_turn * KP_TURN) + (integral_turn * KI_TURN) + (derivative_turn * KD_TURN))

        abs_turn_power = abs(turn_power)
        TURN_SPEED = 600
        abs_turn_power = max(70, min(abs_turn_power, TURN_SPEED))

        # Determine direction based on `current_error_turn` (remaining angle)
        if current_error_turn > 0: # Need to turn counter-clockwise (Left)
            set_motor_speed(motor1_pwm, motor1_in2_pin, -abs_turn_power) # Left wheel backward
            set_motor_speed(motor2_pwm, motor2_in2_pin, abs_turn_power)  # Right wheel forward
        else: # Need to turn clockwise (Right)
            set_motor_speed(motor1_pwm, motor1_in1_pin, abs_turn_power)   # Left wheel forward
            set_motor_speed(motor2_pwm, motor2_in2_pin, -abs_turn_power) # Right wheel backward

        sleep_ms(1)
        turn_count += 1

    stop_motors()
    print(f"Turn complete. Actual turned angle: {total_turned_angle:.2f} rad ({total_turned_angle * 180/pi:.2f} deg)")
    if turn_count >= max_turn_attempts:
        print("Warning: Turn reached max attempts. Might not be perfectly aligned due to timeout.")

    sleep(0.1) # Add a short sleep at the end to allow for physical settling

    # Update current_robot_orientation after successful turn
    # This requires converting the current_robot_orientation (cardinal string)
    # to an angle, adding the total_turned_angle, and converting back to cardinal.
    current_rad_val = CARDINAL_RADIAN_MAP[current_robot_orientation]
    new_rad_val = normalize_angle_rad(current_rad_val + total_turned_angle)
    
    # Find the closest cardinal direction
    min_diff = float('inf')
    new_orientation = current_robot_orientation # Default to current if no significant change
    for cardinal, angle in CARDINAL_RADIAN_MAP.items():
        diff = abs(normalize_angle_rad(new_rad_val - angle))
        if diff < min_diff:
            min_diff = diff
            new_orientation = cardinal
    current_robot_orientation = new_orientation
    print(f"Robot's new cardinal orientation: {current_robot_orientation}")


def perform_180_turn():
    """Performs a 180-degree spin in place using the new orient_robot function."""
    print("Performing 180-degree turn...")
    orient_robot(pi) # Turn by 180 degrees (pi radians)
    print("180-degree turn complete.")

# ----- Line Following Control Loop -----
def run_line_follower():
    """Main loop for line following and node detection and path navigation, now incorporating mission logic."""
    global current_path_idx, last_node_detection_time, obstacle_detected_flag, \
           calculated_path, current_mission_idx, current_mission_state, \
           current_pickup_node, current_delivery_node, current_target_node, \
           last_error_lf, integral_lf, last_print_time, current_robot_orientation, \
           obstacle_readings_count # Add new global for debouncing

    # Define weights locally within the function to ensure scope
    weights = [-2, -1, 0, 1, 2] # Weights for error calculation. Assumes sensors are arranged Left-most to Right-most physically.

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
                
                # Determine initial orientation change
                initial_start_node = calculated_path[0]
                initial_next_node = calculated_path[1]
                direction_to_next = get_direction_between_nodes(initial_start_node, initial_next_node, corrected_weighted_grid)
                
                # The robot *starts* at A1 and we assume it's initially facing North ('N') towards P1.
                # If its first path segment is A1 to C1 (South), it needs to turn 180.
                # If A1 to A2 (East), it needs to turn -90.
                # So the initial orientation needs to align with the first segment.
                
                # Set initial cardinal orientation based on first movement
                if direction_to_next:
                    # Calculate the relative turn needed from our assumed 'North' start
                    # to align with the first segment.
                    relative_turn = calculate_relative_turn_angle(current_robot_orientation, direction_to_next)
                    print(f"Initial turn to align with path from {initial_start_node} to {initial_next_node} ({direction_to_next}). Relative Turn: {relative_turn * 180/pi:.2f} deg")
                    if abs(relative_turn) > 0.01: # Only turn if significantly different
                        orient_robot(relative_turn) # This updates current_robot_orientation
                else:
                    print(f"Warning: Could not determine initial direction from {initial_start_node} to {initial_next_node}.")

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
    last_loop_time_lf_pid = ticks_ms() 
    last_print_time = ticks_ms() # Initialize print timer

    try:
        while True:
            # --- START: Define current_time at the beginning of the loop ---
            current_time = ticks_ms()
            # --- END: Define current_time ---

            # Initialize speeds, error, and correction for printing, default to 0
            left_speed = 0
            right_speed = 0
            error = 'N/A'
            correction = 'N/A'

            # Calculate dt for the current line follower PID iteration
            current_loop_time_lf_pid = ticks_ms()
            dt_loop_lf = (current_loop_time_lf_pid - last_loop_time_lf_pid) / 1000.0 # Time in seconds for PID
            last_loop_time_lf_pid = current_loop_time_lf_pid # Update for the next iteration


            # Check if all missions are complete
            if current_mission_state == MISSION_STATE_COMPLETE:
                print("\n*** ALL MISSIONS COMPLETE! Robot is idle. ***")
                stop_motors()
                while True:
                    sleep(1) # Keep robot stopped
            
            # Ultrasonic distance
            dist = read_distance(trig, echo)
            
            # --- Obstacle Detection Logic with Debouncing ---
            is_obstacle_currently_reading = (dist != -1 and dist < OBSTACLE_DISTANCE_CM)

            if is_obstacle_currently_reading:
                obstacle_readings_count += 1
                if obstacle_readings_count >= OBSTACLE_DEBOUNCE_COUNT:
                    if not obstacle_detected_flag: # Obstacle just confirmed
                        obstacle_detected_flag = True
                        stop_motors() # Ensure motors are stopped before turning
                        print("\n!!! OBSTACLE DETECTED! Reversing to last node and recalculating path. !!!")

                        # Determine the segment that is blocked by the obstacle
                        if current_path_idx < len(calculated_path):
                            # The robot is currently trying to reach calculated_path[current_path_idx].
                            # The segment before it is the one just traversed or the current start.
                            if current_path_idx == 0:
                                blocked_segment_start_node = calculated_path[0] # Assuming this is the 'from' node for the first segment
                                if len(calculated_path) > 1:
                                    blocked_segment_end_node = calculated_path[1]
                                else: 
                                    print("Warning: Path is too short to determine a blocked segment for obstacle.")
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
                            
                        # --- NEW: Reverse to the last node instead of 180-degree turn ---
                        print("Driving backward, line following to the last traversed node...")
                        start_reverse_time_obstacle = ticks_ms()
                        reversed_to_node_obstacle = False
                        
                        temp_last_error_lf_obs = 0
                        temp_integral_lf_obs = 0
                        temp_last_loop_time_lf_obs = ticks_ms() 

                        while ticks_diff(ticks_ms(), start_reverse_time_obstacle) < 3000: # Max 3 seconds reverse
                            current_time_loop_rev_obs = ticks_ms()
                            dt_loop_rev_obs = (current_time_loop_rev_obs - temp_last_loop_time_lf_obs) / 1000.0 
                            temp_last_loop_time_lf_obs = current_time_loop_rev_obs 

                            if dt_loop_rev_obs <= 0: 
                                sleep_ms(1)
                                continue

                            ir_values_reverse_obs = [pin.value() for pin in ir_pins]
                            num_active_sensors_reverse_obs = sum(1 for val in ir_values_reverse_obs if val == 0)

                            if num_active_sensors_reverse_obs > 0:
                                weighted_sum_reverse_obs = sum(weights[i] for i, val in enumerate(ir_values_reverse_obs) if val == 0)
                                current_error_reverse_obs = weighted_sum_reverse_obs 
                                
                                temp_integral_lf_obs += current_error_reverse_obs * dt_loop_rev_obs
                                derivative_lf_obs = (current_error_reverse_obs - temp_last_error_lf_obs) / dt_loop_rev_obs 
                                temp_last_error_lf_obs = current_error_reverse_obs

                                correction_reverse_obs = int((KP_LF_REV * current_error_reverse_obs) + (KI_LF_REV * temp_integral_lf_obs) + (KD_LF_REV * derivative_lf_obs))
                                correction_reverse_obs = max(-MAX_CORRECTION, min(correction_reverse_obs, MAX_CORRECTION))

                                left_speed_rev_obs = -REVERSE_SPEED + correction_reverse_obs
                                right_speed_rev_obs = -REVERSE_SPEED - correction_reverse_obs

                                left_speed_rev_obs = max(-1023, min(left_speed_rev_obs, 0))
                                right_speed_rev_obs = max(-1023, min(right_speed_rev_obs, 0))

                                set_motor_speed(motor1_pwm, motor1_in2_pin, left_speed_rev_obs)
                                set_motor_speed(motor2_pwm, motor2_in2_pin, right_speed_rev_obs)
                            else:
                                print("Line lost while reversing from obstacle! Attempting straight backward movement.")
                                set_motor_speed(motor1_pwm, motor1_in2_pin, -REVERSE_SPEED)
                                set_motor_speed(motor2_pwm, motor2_in2_pin, -REVERSE_SPEED)
                                temp_integral_lf_obs = 0
                                temp_last_error_lf_obs = 0
                            
                            if is_node_detected_robust(ir_values_reverse_obs, num_active_sensors_reverse_obs):
                                print("Backed up onto a node. Stopping reverse line following from obstacle.")
                                reversed_to_node_obstacle = True
                                break
                            sleep_ms(5)
                        stop_motors()
                        if not reversed_to_node_obstacle:
                            print("Warning: Did not detect a node while reversing from obstacle. Might be off track.")
                        
                        print(f"Driving forward for {OBSTACLE_REVERSE_CENTER_DRIVE_TIME_SEC} seconds to center on intersection...")
                        drive_straight_for_time(BASE_SPEED, OBSTACLE_REVERSE_CENTER_DRIVE_TIME_SEC)
                        print("Centered on intersection.")

                        # Reset global PID state after temporary segment
                        last_error_lf = 0
                        integral_lf = 0
                        last_loop_time_lf_pid = ticks_ms() 

                        # Update robot's orientation after backing out.
                        # The robot reversed from `blocked_segment_end_node` to `blocked_segment_start_node`.
                        # Its new orientation is the direction from `blocked_segment_end_node` to `blocked_segment_start_node`.
                        # IMPORTANT FIX: When reversing, the robot's physical orientation (the direction its front is pointing)
                        # does NOT change. It simply moves backward along the line.
                        # So, we should NOT update current_robot_orientation here based on the reverse path.
                        # It should retain its orientation from before the reverse.
                        # The subsequent orient_robot call will handle the turn to the new path segment.
                        # if blocked_segment_start_node and blocked_segment_end_node:
                        #     new_orientation_after_reverse = get_direction_between_nodes(blocked_segment_end_node, blocked_segment_start_node, corrected_weighted_grid)
                        #     if new_orientation_after_reverse:
                        #         current_robot_orientation = new_orientation_after_reverse
                        #         print(f"After reversing from obstacle, robot is at {blocked_segment_start_node}, facing {current_robot_orientation}.")
                        #     else:
                        #         print(f"Warning: Could not determine orientation after reversing from obstacle.")
                        # else:
                        #     print("Warning: Blocked segment nodes not defined, cannot determine orientation after reversal.")
                        print(f"After reversing from obstacle, robot is at {blocked_segment_start_node}, maintaining previous orientation {current_robot_orientation}.")


                        # Recalculate path from the last successfully reached node to the goal.
                        # This is now `blocked_segment_start_node`.
                        current_robot_node_for_recalc = blocked_segment_start_node
                        
                        # Recalculate the path to the current mission target
                        if current_mission_state == MISSION_STATE_PICKUP:
                            target_goal_node = current_pickup_node
                        else: # MISSION_STATE_DELIVER
                            target_goal_node = current_delivery_node

                        new_calculated_path = find_path_dijkstra(current_robot_node_for_recalc, target_goal_node, corrected_weighted_grid)
                        
                        if not new_calculated_path:
                            print("CRITICAL ERROR: No alternative path found after obstacle. Stopping.")
                            stop_motors()
                            return # Exit the function, robot is stuck
                        else:
                            print(f"New path calculated: {new_calculated_path}")
                            calculated_path = new_calculated_path
                            
                            if len(calculated_path) > 1:
                                current_path_idx = 1 # Index of the first actual target node in the new path
                                current_target_node = calculated_path[current_path_idx]
                                
                                # Orient robot towards the next target in the new path
                                node_at_recalc = calculated_path[0] # This is where the robot is after reversing
                                next_node_in_path = calculated_path[1]

                                # Determine the desired cardinal orientation for the next segment
                                desired_orientation_to_next = get_direction_between_nodes(node_at_recalc, next_node_in_path, corrected_weighted_grid)
                                
                                if desired_orientation_to_next:
                                    relative_turn = calculate_relative_turn_angle(current_robot_orientation, desired_orientation_to_next)
                                    print(f"Orienting from {node_at_recalc} towards {next_node_in_path} ({desired_orientation_to_next}). Relative Turn: {relative_turn * 180/pi:.2f} deg")
                                    if abs(relative_turn) > 0.01: # Only turn if significantly different
                                        orient_robot(relative_turn) # This updates current_robot_orientation
                                else:
                                    print(f"Warning: Could not determine direction for orientation from {node_at_recalc} to {next_node_in_path} after obstacle.")
                                
                                print(f"Obstacle path recalculation: Next Node to Target set to {current_target_node}") 
                            else: # Path is just one node (robot is already at the target)
                                current_path_idx = 0
                                current_target_node = calculated_path[0]
                                print(f"Obstacle path recalculation: Next Node to Target set to {current_target_node} (single node path)") 
                            
                            # obstacle_detected_flag is already True, no need to reset here
            else: # Current reading does NOT indicate an obstacle
                obstacle_readings_count = 0 # Reset counter
                if obstacle_detected_flag: # Obstacle was previously detected but now cleared
                    print("--- Obstacle cleared. Resuming. ---")
                    obstacle_detected_flag = False
            
            # This main block only executes if no obstacle is currently preventing movement
            if not obstacle_detected_flag:
                button_pressed = button.value() == 0
                
                # --- Button Pressed Logic for Pickup ---
                if button_pressed and current_mission_state == MISSION_STATE_PICKUP:
                    if calculated_path and current_path_idx < len(calculated_path) and calculated_path[current_path_idx] == current_pickup_node:
                        print(f"Confirmed at pickup node {current_pickup_node}. Activating electromagnet, picking up box.")
                        electromagnet.on()  # Turn electromagnet ON
                        stop_motors()       # Stop the robot
                        sleep(3)            # Wait for 3 seconds to ensure pickup
                        
                        current_mission_state = MISSION_STATE_DELIVER
                        print(f"Box picked up. Now moving to deliver to {current_delivery_node}.")
                        
                        print("Driving backward, line following to detect connecting node from P-node...")
                        start_reverse_time = ticks_ms()
                        reversed_to_node = False
                        
                        # Localize PID state for this temporary reverse segment
                        temp_last_error_lf = 0
                        temp_integral_lf = 0
                        temp_last_loop_time_lf_pid = ticks_ms() 

                        while ticks_diff(ticks_ms(), start_reverse_time) < 3000: # Max 3 seconds reverse
                            current_time_loop_rev = ticks_ms()
                            dt_loop_rev = (current_time_loop_rev - temp_last_loop_time_lf_pid) / 1000.0 
                            temp_last_loop_time_lf_pid = current_time_loop_rev 

                            # Ensure dt is not zero
                            if dt_loop_rev <= 0: 
                                sleep_ms(1)
                                continue

                            ir_values_reverse = [pin.value() for pin in ir_pins]
                            num_active_sensors_reverse = sum(1 for val in ir_values_reverse if val == 0)

                            if num_active_sensors_reverse > 0:
                                weighted_sum_reverse = sum(weights[i] for i, val in enumerate(ir_values_reverse) if val == 0)
                                current_error_reverse = weighted_sum_reverse 
                                
                                temp_integral_lf += current_error_reverse * dt_loop_rev
                                derivative_lf = (current_error_reverse - temp_last_error_lf) / dt_loop_rev 
                                temp_last_error_lf = current_error_reverse

                                correction_reverse = int((KP_LF_REV * current_error_reverse) + (KI_LF_REV * temp_integral_lf) + (KD_LF_REV * derivative_lf))
                                correction_reverse = max(-MAX_CORRECTION, min(correction_reverse, MAX_CORRECTION))

                                left_speed_rev = -REVERSE_SPEED + correction_reverse
                                right_speed_rev = -REVERSE_SPEED - correction_reverse

                                left_speed_rev = max(-1023, min(left_speed_rev, 0))
                                right_speed_rev = max(-1023, min(right_speed_rev, 0))

                                set_motor_speed(motor1_pwm, motor1_in2_pin, left_speed_rev)
                                set_motor_speed(motor2_pwm, motor2_in2_pin, right_speed_rev)
                            else:
                                print("Line lost while reversing! Attempting straight backward movement.")
                                set_motor_speed(motor1_pwm, motor1_in2_pin, -REVERSE_SPEED)
                                set_motor_speed(motor2_pwm, motor2_in2_pin, -REVERSE_SPEED)
                                temp_integral_lf = 0
                                temp_last_error_lf = 0
                            
                            if is_node_detected_robust(ir_values_reverse, num_active_sensors_reverse):
                                print("Backed up onto a node. Stopping reverse line following.")
                                reversed_to_node = True
                                break
                            sleep_ms(5)
                        stop_motors()
                        if not reversed_to_node:
                            print("Warning: Did not detect a node while reversing from pickup. Might be off track.")
                        
                        print(f"Driving forward for {PICKUP_REVERSE_CENTER_DRIVE_TIME_SEC} seconds to center on intersection...")
                        drive_straight_for_time(BASE_SPEED, PICKUP_REVERSE_CENTER_DRIVE_TIME_SEC)
                        print("Centered on intersection.")

                        # Reset global PID state after temporary segment
                        last_error_lf = 0
                        integral_lf = 0
                        last_loop_time_lf_pid = ticks_ms() 

                        # The robot is now logically at the node from which it entered the P-node dead-end.
                        # We need to determine this entry node and its orientation.
                        # The robot will have reversed, so its current orientation is the *opposite* of
                        # the direction it took to enter the P-node.
                        node_robot_is_at_after_reverse = ""
                        direction_entered_p_node_from = None
                        
                        # Find which node connects to current_pickup_node from the main grid
                        for direction, info in corrected_weighted_grid[current_pickup_node].items():
                            if info and info[0] in corrected_weighted_grid:
                                node_robot_is_at_after_reverse = info[0]
                                direction_entered_p_node_from = direction # e.g., if P1 has 'S' to A1, means robot entered from South
                                break

                        if node_robot_is_at_after_reverse:
                            # IMPORTANT FIX: When reversing from a P-node, the robot's physical orientation
                            # does NOT change. It simply moves backward.
                            # So, we should NOT update current_robot_orientation to the opposite direction.
                            # current_robot_orientation = get_opposite_direction(direction_entered_p_node_from)
                            print(f"After reversing from {current_pickup_node}, robot is at {node_robot_is_at_after_reverse}, maintaining previous orientation {current_robot_orientation}.")

                            # Recalculate path from this entry node to the delivery node
                            calculated_path = find_path_dijkstra(node_robot_is_at_after_reverse, current_delivery_node, corrected_weighted_grid)
                            if calculated_path:
                                if len(calculated_path) > 1:
                                    current_path_idx = 1 
                                    current_target_node = calculated_path[current_path_idx]
                                    
                                    node_for_orient = calculated_path[0] # This is `node_robot_is_at_after_reverse`
                                    next_node_for_orient = calculated_path[1]
                                    
                                    desired_orientation_for_segment = get_direction_between_nodes(node_for_orient, next_node_for_orient, corrected_weighted_grid)
                                    
                                    if desired_orientation_for_segment:
                                        relative_turn = calculate_relative_turn_angle(current_robot_orientation, desired_orientation_for_segment)
                                        print(f"Orienting from {node_for_orient} towards {next_node_for_orient} ({desired_orientation_for_segment}). Relative Turn: {relative_turn * 180/pi:.2f} deg")
                                        if abs(relative_turn) > 0.01: # Only turn if significantly different
                                            orient_robot(relative_turn) # This updates current_robot_orientation
                                    print(f"Pickup Path Recalculation: Next Node to Target set to {current_target_node}") 
                                else:
                                    current_path_idx = 0 
                                    current_target_node = calculated_path[0]
                                    print(f"Path to {current_delivery_node} is just one node. No immediate turn needed.")
                                    print(f"Pickup Path Recalculation: Next Node to Target set to {current_target_node}") 

                                print(f"Path to delivery {current_delivery_node}: {calculated_path}")

                            else:
                                print(f"ERROR: No path found from {node_robot_is_at_after_reverse} to {current_delivery_node}. Stopping.")
                                stop_motors()
                                return 
                        else:
                            print(f"ERROR: Could not determine entry node for {current_pickup_node}. Stopping.")
                            stop_motors()
                            return

                    else:
                        print(f"Button pressed, but not at the designated pickup node ({current_pickup_node}). Ignoring.")

                ir_values = [pin.value() for pin in ir_pins] 
                
                num_active_sensors = 0
                weighted_sum = 0

                # Calculate error based on active IR sensors
                for i, sensor_value in enumerate(ir_values):
                    if sensor_value == 0:
                        weighted_sum += weights[i]
                        num_active_sensors += 1

                # --- Node Detection Logic with Cooldown ---
                if is_node_detected_robust(ir_values, num_active_sensors):
                    if (current_time - last_node_detection_time) >= NODE_DETECTION_COOLDOWN_MS:
                        # The robot has just arrived at a node.
                        # node_just_arrived_at will be the node the robot is currently physically at.
                        # This node is the one *before* current_target_node in the path.
                        node_just_arrived_at = calculated_path[current_path_idx - 1] if current_path_idx > 0 else calculated_path[0]
                        
                        stop_motors()
                        print(f"\n*** NODE DETECTED! Arrived at {node_just_arrived_at}. Stopping briefly. ***")
                        sleep_ms(NODE_STOP_TIME_MS)
                        
                        print(f"Driving forward for {NODE_CLEAR_DRIVE_TIME_SEC} second to clear node with wheels...")
                        drive_straight_for_time(BASE_SPEED, NODE_CLEAR_DRIVE_TIME_SEC) # Robot is now past Node_X, centered.

                        last_node_detection_time = current_time
                        last_loop_time_lf_pid = ticks_ms() 
                        last_error_lf = 0
                        integral_lf = 0

                        # Check if the next node in the path is the final pickup node
                        if current_mission_state == MISSION_STATE_PICKUP and \
                           current_path_idx < len(calculated_path) and \
                           calculated_path[current_path_idx] == current_pickup_node:
                            
                            print(f"Arrived at pickup node {current_pickup_node}. Waiting for button press to pick up.")
                            # The current_path_idx already points to the pickup node, so no increment.
                            stop_motors() # Ensure it stays stopped
                            continue # Go back to main loop, waiting for button

                        # Check if the *next* node in the path is the delivery P-node.
                        # This means we just arrived at the node *before* the delivery P-node.
                        # `node_just_arrived_at` is the node we just arrived at (e.g., E2).
                        # `calculated_path[current_path_idx]` is the junction node (e.g., E3).
                        # `current_delivery_node` is the final P-node (e.g., P5).
                        elif current_mission_state == MISSION_STATE_DELIVER and \
                             (current_path_idx + 1) < len(calculated_path) and \
                             calculated_path[current_path_idx + 1] == current_delivery_node:
                            
                            # First, drive from `node_just_arrived_at` (e.g., E2) to the junction node (e.g., E3).
                            junction_node = calculated_path[current_path_idx]
                            print(f"Driving from {node_just_arrived_at} to junction node {junction_node} before final delivery to {current_delivery_node}...")
                            
                            # Determine direction and duration to drive to the junction node
                            direction_to_junction = get_direction_between_nodes(node_just_arrived_at, junction_node, corrected_weighted_grid)
                            drive_to_junction_duration = 0.0
                            if direction_to_junction:
                                drive_to_junction_duration = corrected_weighted_grid[node_just_arrived_at][direction_to_junction][1]
                                # Orient towards the junction if necessary
                                relative_turn_to_junction = calculate_relative_turn_angle(current_robot_orientation, direction_to_junction)
                                if abs(relative_turn_to_junction) > 0.01:
                                    orient_robot(relative_turn_to_junction)
                            else:
                                print(f"Warning: Could not determine direction from {node_just_arrived_at} to junction {junction_node}. Using default drive time.")
                                drive_to_junction_duration = 1.0 # Fallback

                            drive_straight_for_time(BASE_SPEED, drive_to_junction_duration)
                            
                            # Now the robot is physically at the `junction_node` (e.g., E3).
                            # Update `current_path_idx` to point to the delivery P-node for the next step.
                            current_path_idx += 1
                            current_target_node = calculated_path[current_path_idx] # This is the P-node (e.g., P5)

                            print(f"*** At junction {junction_node}. Now orienting and driving into delivery node {current_target_node}. ***")
                            
                            # Determine the desired orientation to enter the P-node from the junction node
                            desired_orientation_to_p_node = get_direction_between_nodes(junction_node, current_target_node, corrected_weighted_grid)
                            
                            delivery_drive_duration = DELIVERY_DRIVE_INTO_PNODE_TIME_SEC # User requested this to be fixed at 1.0 second.
                            if desired_orientation_to_p_node:
                                relative_turn_to_p_node = calculate_relative_turn_angle(current_robot_orientation, desired_orientation_to_p_node)
                                print(f"Orienting from {junction_node} towards {current_target_node} ({desired_orientation_to_p_node}). Relative Turn: {relative_turn_to_p_node * 180/pi:.2f} deg")
                                if abs(relative_turn_to_p_node) > 0.01: # Only turn if significantly different
                                    orient_robot(relative_turn_to_p_node) # This updates current_robot_orientation
                                
                                print(f"Using fixed delivery drive duration: {delivery_drive_duration} seconds.")
                            else:
                                print(f"CRITICAL WARNING: Could not determine direction for orientation from {junction_node} to {current_target_node}. Using default duration. This indicates a map error or logic flaw.")
                                # delivery_drive_duration is already set to DELIVERY_DRIVE_INTO_PNODE_TIME_SEC

                            print(f"Driving forward for {delivery_drive_duration} seconds to position for delivery...")
                            drive_straight_for_time(BASE_SPEED, delivery_drive_duration)
                            
                            print("Deactivating electromagnet to drop box.")
                            electromagnet.off()
                            sleep(2)
                            
                            # Step 3: Drive backward from P-node to the junction node.
                            print(f"Driving backward to return to junction node {junction_node}...")
                            start_reverse_time_delivery = ticks_ms()
                            reversed_to_node_delivery = False
                            
                            temp_last_error_lf = 0
                            temp_integral_lf = 0
                            temp_last_loop_time_lf_pid = ticks_ms()
                            delivery_p_node = current_target_node # Store for clarity

                            while ticks_diff(ticks_ms(), start_reverse_time_delivery) < 3000:
                                current_time_loop_rev_del = ticks_ms()
                                dt_loop_rev_del = (current_time_loop_rev_del - temp_last_loop_time_lf_pid) / 1000.0
                                temp_last_loop_time_lf_pid = current_time_loop_rev_del

                                if dt_loop_rev_del <= 0:
                                    sleep_ms(1)
                                    continue

                                ir_values_reverse_delivery = [pin.value() for pin in ir_pins]
                                num_active_sensors_reverse_delivery = sum(1 for val in ir_values_reverse_delivery if val == 0)

                                if is_node_detected_robust(ir_values_reverse_delivery, num_active_sensors_reverse_delivery):
                                    print(f"Backed up onto node {junction_node}. Stopping reverse movement.")
                                    reversed_to_node_delivery = True
                                    break
                                
                                if num_active_sensors_reverse_delivery > 0:
                                    weighted_sum_reverse_delivery = sum(weights[i] for i, val in enumerate(ir_values_reverse_delivery) if val == 0)
                                    current_error_reverse_delivery = weighted_sum_reverse_delivery
                                    
                                    temp_integral_lf += current_error_reverse_delivery * dt_loop_rev_del
                                    derivative_lf = (current_error_reverse_delivery - temp_last_error_lf) / dt_loop_rev_del 
                                    temp_last_error_lf = current_error_reverse_delivery

                                    correction_reverse_delivery = int((KP_LF_REV * current_error_reverse_delivery) + (KI_LF_REV * temp_integral_lf) + (KD_LF_REV * derivative_lf))
                                    correction_reverse_delivery = max(-MAX_CORRECTION, min(correction_reverse_delivery, MAX_CORRECTION))

                                    left_speed_rev_del = -REVERSE_SPEED + correction_reverse_delivery
                                    right_speed_rev_del = -REVERSE_SPEED - correction_reverse_delivery

                                    left_speed_rev_del = max(-1023, min(left_speed_rev_del, 0))
                                    right_speed_rev_del = max(-1023, min(right_speed_rev_del, 0))

                                    set_motor_speed(motor1_pwm, motor1_in2_pin, left_speed_rev_del)
                                    set_motor_speed(motor2_pwm, motor2_in2_pin, right_speed_rev_del)
                                else:
                                    print("Line lost while reversing from delivery! Attempting straight backward movement.")
                                    set_motor_speed(motor1_pwm, motor1_in2_pin, -REVERSE_SPEED)
                                    set_motor_speed(motor2_pwm, motor2_in2_pin, -REVERSE_SPEED)
                                    temp_integral_lf = 0
                                    temp_last_error_lf = 0
                                sleep_ms(5)
                            stop_motors()
                            if not reversed_to_node_delivery:
                                print("Warning: Did not detect a node while reversing from delivery. Might be off track.")
                            
                            # Reset global PID state after temporary segment
                            last_error_lf = 0
                            integral_lf = 0
                            last_loop_time_lf_pid = ticks_ms() 

                            print(f"After reversing from {delivery_p_node}, robot is at {junction_node}, maintaining previous orientation {current_robot_orientation}.")


                            current_mission_idx += 1
                            if current_mission_idx < len(MISSION_PLAN):
                                current_pickup_node, current_delivery_node = MISSION_PLAN[current_mission_idx]
                                current_mission_state = MISSION_STATE_PICKUP 
                                print(f"Delivery complete. Next mission: Pick up from {current_pickup_node}.")
                                
                                # Recalculate path to the next pickup node from the junction node
                                calculated_path = find_path_dijkstra(junction_node, current_pickup_node, corrected_weighted_grid)
                                if calculated_path:
                                    if len(calculated_path) > 1:
                                        current_path_idx = 1 # Start from the first actual target node in the new path
                                        current_target_node = calculated_path[current_path_idx]
                                        node_for_orient = calculated_path[0] # This is `junction_node`
                                        next_node_for_orient = calculated_path[1]
                                        
                                        desired_orientation_for_segment = get_direction_between_nodes(node_for_orient, next_node_for_orient, corrected_weighted_grid)
                                        if desired_orientation_for_segment:
                                            relative_turn = calculate_relative_turn_angle(current_robot_orientation, desired_orientation_for_segment)
                                            print(f"Orienting from {node_for_orient} towards {next_node_for_orient} ({desired_orientation_for_segment}). Relative Turn: {relative_turn * 180/pi:.2f} deg")
                                            if abs(relative_turn) > 0.01: # Only turn if significantly different
                                                orient_robot(relative_turn) # This updates current_robot_orientation
                                        print(f"Delivery Next Mission Path Recalculation: Next Node to Target set to {current_target_node}")
                                    else:
                                        current_path_idx = 0
                                        current_target_node = calculated_path[0]
                                        print(f"Delivery Next Mission Path Recalculation: Next Node to Target set to {current_target_node} (single node path)")
                                    print(f"Path to next pickup {current_pickup_node}: {calculated_path}")
                                else:
                                    print(f"ERROR: No path found from {junction_node} to {current_pickup_node}. Stopping.")
                                    current_mission_state = MISSION_STATE_COMPLETE
                                    stop_motors()
                                    return
                            else: # All missions complete
                                print("All boxes delivered! Mission complete.")
                                current_mission_state = MISSION_STATE_COMPLETE
                                stop_motors()
                                return
                            
                            continue # Restart the main while loop
                        
                        # If neither of the above special cases (final pickup or delivery P-node approach)
                        # then it's a regular intermediate node, so increment path_idx for traversal.
                        elif current_path_idx + 1 < len(calculated_path): 
                            current_path_idx += 1
                            current_target_node = calculated_path[current_path_idx]
                            
                            # Determine the direction from the node we just arrived at
                            # to the next target node in the path.
                            node_robot_was_at = calculated_path[current_path_idx - 1]
                            node_robot_is_going_to = calculated_path[current_path_idx]

                            desired_orientation_for_segment = get_direction_between_nodes(node_robot_was_at, node_robot_is_going_to, corrected_weighted_grid)
                            
                            if desired_orientation_for_segment:
                                relative_turn = calculate_relative_turn_angle(current_robot_orientation, desired_orientation_for_segment)
                                print(f"Current node: {node_robot_was_at}, Next path segment target: {node_robot_is_going_to}")
                                print(f"Required turn direction: {desired_orientation_for_segment}, Relative Turn: {relative_turn * 180/pi:.2f} deg")
                                if abs(relative_turn) > 0.01: # Only turn if significantly different
                                    orient_robot(relative_turn) # This updates current_robot_orientation
                            else:
                                print(f"Warning: Could not determine direction for orientation from {node_robot_was_at} to {node_robot_is_going_to}.")
                                
                            print(f"Node Detection & Orientation: Next Node to Target set to {current_target_node}")
                        else:
                            # This means we arrived at the very last node in the path, and it's not a P-node for delivery.
                            # This would typically be the final pickup node (handled above) or the very end of the mission path.
                            print(f"Arrived at final node {node_just_arrived_at} in path. Waiting for next action (e.g., button press for pickup or mission completion).")
                            stop_motors() # Stay stopped
                            continue # Restart main loop to await external trigger (button) or mission end.

                # Continue with line following if not at goal, not handling obstacle, and not waiting for button
                if not button_pressed or not (current_mission_state == MISSION_STATE_PICKUP and calculated_path[current_path_idx] == current_pickup_node):
                    if num_active_sensors > 0:
                        error = weighted_sum 

                        integral_lf += error * dt_loop_lf 
                        derivative_lf = (error - last_error_lf) / dt_loop_lf if dt_loop_lf > 0 else 0
                        last_error_lf = error 

                        correction = int((KP_LF * error) + (KI_LF * integral_lf) + (KD_LF * derivative_lf))
                        correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
                    else: 
                        print("Line lost! Continuing forwards, attempting to find line.")
                        integral_lf = 0
                        last_error_lf = 0

                    if num_active_sensors > 0:
                        left_speed = BASE_SPEED + correction
                        right_speed = BASE_SPEED - correction
                    else: 
                        left_speed = BASE_SPEED
                        right_speed = BASE_SPEED

                    left_speed = max(0, min(left_speed, 1023))
                    right_speed = max(0, min(right_speed, 1023))

                    set_motor_speed(motor1_pwm, motor1_in2_pin, left_speed)
                    set_motor_speed(motor2_pwm, motor2_in2_pin, right_speed)

            else:
                stop_motors()
                sleep_ms(50) 


            # --- Conditional Debug Printing ---
            current_time_for_print = ticks_ms()
            if (current_time_for_print - last_print_time) >= PRINT_INTERVAL_MS:
                print("\n--- Current Robot State ---")
                print("IR Sensors:", ir_values)
                print("Error:", error)
                print("Correction:", correction)
                print(f"Left Speed: {left_speed}, Right Speed: {right_speed}")
                print("Encoder 1 Count:", position1)
                print("Encoder 2 Count:", position2)
                print("Distance: {:.2f} cm".format(dist) if dist != -1 else "Ultrasonic: Timeout")
                print("Button Pressed:", button_pressed)
                print("Robot Cardinal Orientation:", current_robot_orientation)
                print("Obstacle Detected Flag:", obstacle_detected_flag)
                print("Obstacle Readings Count:", obstacle_readings_count) # Added for debug
                print("Current Mission Index:", current_mission_idx)
                print("Current Mission State:", "PICKUP" if current_mission_state == MISSION_STATE_PICKUP else "DELIVER" if current_mission_state == MISSION_STATE_DELIVER else "COMPLETE")
                print("Current Pickup Node:", current_pickup_node)
                print("Current Delivery Node:", current_delivery_node)
                print("Current Node Index in Path:", current_path_idx) 
                print("Next Node to Target in Path:", current_target_node) 
                if calculated_path:
                    print("Current Path:", calculated_path)
                last_print_time = current_time_for_print 

            sleep_ms(5)

    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        stop_motors()
        electromagnet.off()
        print("Motors stopped, electromagnet off.")

# --- Main execution starts here ---
# 1. Stop motors immediately upon script start
stop_motors()
electromagnet.off()

# 2. Initialize MPU6050 and calibrate gyro (while motors are stopped)
mpu = MPU6050(i2c)
mpu.calibrate_gyro()

# Set the initial cardinal orientation to North
current_robot_orientation = 'N' 

# Start the line following loop (robot will start moving after path is calculated and printed)
run_line_follower()

