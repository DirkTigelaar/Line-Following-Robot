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
KP = 100
MAX_CORRECTION = 200

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
last_time = ticks_ms()
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

TURN_SPEED = 600 # Speed for turning (adjust as needed for controlled turns, increased from 200)
# Yaw tolerance: now 10 degrees on each side
YAW_TOLERANCE = 2.0 * (pi / 180.0) # Converted from 10 degrees to radians for precision

def orient_robot(target_yaw_radians, spin_in_place=True):
    """
    Orients the robot to a target yaw angle.
    Args:
        target_yaw_radians (float): The desired yaw angle in radians (-pi to pi).
        spin_in_place (bool): If True, spins in place (one wheel forward, one backward).
                              If False, pivots using one stationary wheel.
                              For this modification, we will force spin_in_place to be True.
    """
    global yaw_angle, last_time

    print(f"Orienting robot to {target_yaw_radians:.2f} rad ({target_yaw_radians * 180/pi:.2f} deg) (Spin in Place)...")
    
    # Normalize target angle for consistency
    target_yaw_radians = normalize_angle_rad(target_yaw_radians)

    turn_count = 0
    max_turn_attempts = 300 # Safety limit to prevent endless loop during turning

    while abs(get_shortest_angle_difference_rad(yaw_angle, target_yaw_radians)) > YAW_TOLERANCE and turn_count < max_turn_attempts:
        current_yaw_normalized = normalize_angle_rad(yaw_angle)
        angle_diff = get_shortest_angle_difference_rad(current_yaw_normalized, target_yaw_radians)

        # Always perform a spin in place
        if angle_diff > 0: # Need to turn counter-clockwise (Left)
            set_motor_speed(motor1_pwm, motor1_in2_pin, -TURN_SPEED) # Left wheel backward
            set_motor_speed(motor2_pwm, motor2_in2_pin, TURN_SPEED)  # Right wheel forward
            # turn_direction_str = "LEFT (CCW) - Spin"
        else: # Need to turn clockwise (Right)
            set_motor_speed(motor1_pwm, motor1_in2_pin, TURN_SPEED)  # Left wheel forward
            set_motor_speed(motor2_pwm, motor2_in2_pin, -TURN_SPEED) # Right wheel backward
            # turn_direction_str = "RIGHT (CW) - Spin"

        # Update yaw angle
        gyro = mpu.get_gyro()
        current_time_loop = ticks_ms()
        dt_loop = (current_time_loop - last_time) / 1000.0 # Time in seconds
        last_time = current_time_loop
        
        # Integrate gyro Z (degrees/s) into yaw_angle (radians)
        yaw_angle += (gyro['z'] * (pi / 180.0)) * dt_loop
        yaw_angle = normalize_angle_rad(yaw_angle) # Keep yaw within -pi to pi

        # print(f"  Current Yaw: {current_yaw_normalized:.2f} rad ({current_yaw_normalized * 180/pi:.2f} deg), Target Yaw: {target_yaw_radians:.2f} rad, Diff: {angle_diff:.2f} rad, Turning: {turn_direction_str}")
        sleep_ms(50) # Small delay for MPU reading and motor response
        turn_count += 1

    stop_motors()
    print(f"Robot oriented. Final Yaw: {normalize_angle_rad(yaw_angle):.2f} rad ({normalize_angle_rad(yaw_angle) * 180/pi:.2f} deg)")
    if turn_count >= max_turn_attempts:
        print("Warning: Orientation reached max turn attempts. Might not be perfectly aligned.")


def perform_180_turn():
    """Performs a 180-degree turn in place."""
    global yaw_angle
    print("Performing 180-degree turn...")
    target_yaw_180 = normalize_angle_rad(yaw_angle + pi)
    orient_robot(target_yaw_180, spin_in_place=True)
    print("180-degree turn complete.")

# ----- Line Following Control Loop -----
def run_line_follower():
    """Main loop for line following and node detection and path navigation, now incorporating mission logic."""
    global yaw_angle, last_time, current_path_idx, last_node_detection_time, obstacle_detected_flag, \
           calculated_path, current_mission_idx, current_mission_state, \
           current_pickup_node, current_delivery_node, current_target_node # Declare global to modify

    # Initialize the first mission target
    if MISSION_PLAN:
        current_pickup_node, current_delivery_node = MISSION_PLAN[current_mission_idx]
        print(f"\nStarting mission: Pick up from {current_pickup_node}")
        # Initial path calculation for pickup
        calculated_path = find_path_dijkstra("C3", current_pickup_node, corrected_weighted_grid) # Assuming C3 as starting point
        print(f"Initial path to pickup {current_pickup_node}: {calculated_path}")
        if calculated_path:
            current_path_idx = 0
            if len(calculated_path) > 1:
                current_target_node = calculated_path[current_path_idx + 1] # Set the immediate next node as target
            else:
                current_target_node = calculated_path[current_path_idx] # If path is just the pickup node
        else:
            print("ERROR: No path to initial pickup node. Stopping.")
            stop_motors()
            return

    else:
        print("No mission plan defined. Exiting.")
        stop_motors()
        return


    try:
        while True:
            # Check if all missions are complete
            if current_mission_state == MISSION_STATE_COMPLETE:
                print("\n*** ALL MISSIONS COMPLETE! Robot is idle. ***")
                stop_motors()
                # Optional: return to a specific "home" node here, or just stop.
                while True:
                    sleep(1) # Keep robot stopped
            
            # MPU6050 - Always update yaw angle in the main loop
            gyro = mpu.get_gyro()
            current_time = ticks_ms()
            dt = (current_time - last_time) / 1000.0 # Time in seconds
            last_time = current_time
            
            # Integrate gyro Z (degrees/s) into yaw_angle (radians)
            yaw_angle += (gyro['z'] * (pi / 180.0)) * dt
            yaw_angle = normalize_angle_rad(yaw_angle) # Keep yaw within -pi to pi
            
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
                        # Reset path index based on the new current_robot_node
                        try:
                            current_path_idx = calculated_path.index(current_robot_node)
                            # If we just turned 180 degrees, we are effectively at the "start" of the segment
                            # that was just blocked, so we need to target the next node in the new path.
                            if current_path_idx < len(calculated_path) - 1:
                                current_target_node = calculated_path[current_path_idx + 1]
                            else: # If new path is just the current node (goal is current node)
                                current_target_node = calculated_path[current_path_idx]
                        except ValueError:
                            print("Warning: Last known node not found in new path. Resetting index to 0.")
                            current_path_idx = 0
                            current_target_node = calculated_path[0] if calculated_path else ""
                        
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
                # The node detection should confirm we are at the pickup node.
                if button_pressed and current_mission_state == MISSION_STATE_PICKUP:
                    # Check if we are at the designated pickup node (current_target_node or last node in path)
                    # This is a simplification; a more robust check would involve confirming our location.
                    print(f"Button pressed at a potential pickup point! Current target node: {current_target_node}")
                    # If the current target node is indeed the pickup node
                    if current_target_node == current_pickup_node:
                        print(f"Confirmed at pickup node {current_pickup_node}. Activating electromagnet, picking up box.")
                        electromagnet.on()  # Turn electromagnet ON
                        stop_motors()       # Stop the robot
                        sleep(3)            # Wait for 3 seconds to ensure pickup
                        
                        # After pickup, transition to deliver state
                        current_mission_state = MISSION_STATE_DELIVER
                        print(f"Box picked up. Now moving to deliver to {current_delivery_node}.")
                        
                        # Perform 180-degree turn
                        perform_180_turn()

                        # Recalculate path from current node (which is current_pickup_node) to delivery node
                        calculated_path = find_path_dijkstra(current_pickup_node, current_delivery_node, corrected_weighted_grid)
                        if calculated_path:
                            current_path_idx = 0
                            if len(calculated_path) > 1:
                                current_target_node = calculated_path[current_path_idx + 1]
                            else:
                                current_target_node = calculated_path[current_path_idx] # If path is just the delivery node itself
                            print(f"Path to delivery {current_delivery_node}: {calculated_path}")
                        else:
                            print(f"ERROR: No path found from {current_pickup_node} to {current_delivery_node}. Stopping.")
                            stop_motors()
                            return # Critical error, cannot complete mission
                    else:
                        print("Button pressed but not at current pickup node. Ignoring for now.")


                ir_values = [pin.value() for pin in ir_pins] # Read all IR sensor values
                error = 0
                weights = [-2, -1, 0, 1, 2] # Weights for error calculation

                num_active_sensors = 0
                weighted_sum = 0

                # Calculate error based on active IR sensors
                for i, sensor_value in enumerate(ir_values):
                    if sensor_value == 0: # Assuming 0 means line detected (dark line on light background)
                        weighted_sum += weights[i]
                        num_active_sensors += 1

                # --- Node Detection Logic with Cooldown ---
                if is_node_detected_robust(ir_values, num_active_sensors):
                    if (current_time - last_node_detection_time) >= NODE_DETECTION_COOLDOWN_MS:
                        stop_motors()
                        print("\n*** NODE DETECTED! Stopping briefly. ***")
                        sleep_ms(NODE_STOP_TIME_MS) # Uses the updated NODE_STOP_TIME_MS
                        
                        # Drive forward for 1 second to clear the node (important for small bots)
                        print("Driving forward for 1 second to clear node with wheels...")
                        set_motor_speed(motor1_pwm, motor1_in2_pin, BASE_SPEED)
                        set_motor_speed(motor2_pwm, motor2_in2_pin, BASE_SPEED)
                        sleep(1) # Drive forward for 1 second
                        stop_motors() # Stop after driving forward

                        last_node_detection_time = current_time # Update last detection time

                        # Check if we have reached the current target node in the path
                        # The robot is considered to have 'reached' a node if a node is detected,
                        # and that detected node is the one immediately following the current_path_idx.
                        # For simplicity, we assume robust detection and that the detected node IS current_target_node.

                        current_node_reached = False
                        if current_path_idx < len(calculated_path):
                            # The node we just passed/are at is calculated_path[current_path_idx]
                            # and our target was calculated_path[current_path_idx]
                            print(f"Node detection event: Robot at or just passed {calculated_path[current_path_idx]}")
                            current_node_reached = True
                        else:
                            # This means we are at the very end of the path or beyond
                            print("Node detected, but path index is at or beyond end of path.")
                            # Still proceed to handle the 'goal' logic if applicable below

                        if current_node_reached:
                            current_path_idx += 1 # Advance to the next segment's start node (or goal)

                            # Check if current_path_idx now points to the goal for the current mission stage
                            target_goal_for_mission_stage = ""
                            if current_mission_state == MISSION_STATE_PICKUP:
                                target_goal_for_mission_stage = current_pickup_node
                            elif current_mission_state == MISSION_STATE_DELIVER:
                                target_goal_for_mission_stage = current_delivery_node
                            
                            # If we just arrived at the mission's immediate goal (pickup or delivery)
                            if (current_path_idx > 0 and calculated_path[current_path_idx - 1] == target_goal_for_mission_stage):
                                print(f"*** Arrived at mission stage goal: {target_goal_for_mission_stage} ***")
                                stop_motors()
                                
                                if current_mission_state == MISSION_STATE_DELIVER:
                                    # Logic for delivery completion
                                    print(f"Arrived at delivery node {current_delivery_node}. Deactivating electromagnet.")
                                    electromagnet.off() # Turn electromagnet OFF
                                    sleep(2) # Give some time for box to settle
                                    
                                    # Move to next mission or complete
                                    current_mission_idx += 1
                                    if current_mission_idx < len(MISSION_PLAN):
                                        current_pickup_node, current_delivery_node = MISSION_PLAN[current_mission_idx]
                                        current_mission_state = MISSION_STATE_PICKUP # Go to next pickup
                                        print(f"Delivery complete. Next mission: Pick up from {current_pickup_node}.")
                                        
                                        # Perform 180-degree turn
                                        perform_180_turn()

                                        # Recalculate path to the next pickup node
                                        # Assuming robot is currently at the *previous* delivery node
                                        calculated_path = find_path_dijkstra(target_goal_for_mission_stage, current_pickup_node, corrected_weighted_grid)
                                        if calculated_path:
                                            current_path_idx = 0
                                            if len(calculated_path) > 1:
                                                current_target_node = calculated_path[current_path_idx + 1]
                                            else:
                                                current_target_node = calculated_path[current_path_idx]
                                            print(f"Path to next pickup {current_pickup_node}: {calculated_path}")
                                        else:
                                            print(f"ERROR: No path found from {target_goal_for_mission_stage} to {current_pickup_node}. Stopping.")
                                            current_mission_state = MISSION_STATE_COMPLETE # Cannot continue
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

                            # If not at the ultimate mission goal, orient to the next segment of the current path
                            if current_path_idx < len(calculated_path): # Check if there's a next node in the path
                                current_node_name = calculated_path[current_path_idx -1] # The node we just came from/are at
                                next_node_name = calculated_path[current_path_idx] # The node we need to go to
                                
                                direction_to_next = get_direction_between_nodes(current_node_name, next_node_name, corrected_weighted_grid)
                                
                                if direction_to_next and direction_to_next in TARGET_YAW_ANGLES:
                                    target_yaw = TARGET_YAW_ANGLES[direction_to_next]
                                    print(f"Current node: {current_node_name}, Next path segment target: {next_node_name}")
                                    print(f"Required turn direction: {direction_to_next}, Target Yaw: {target_yaw * 180/pi:.2f} deg")
                                    orient_robot(target_yaw) # Orient the robot
                                    current_target_node = next_node_name # Update the immediate target node
                                else:
                                    print(f"Warning: Could not determine direction from {current_node_name} to {next_node_name} or direction is not recognized. Attempting to continue.")
                                    # If direction cannot be found, robot might get lost. Keep going straight?
                                    # For now, just advance the index and hope for the best.
                                    # It's crucial that all valid transitions are defined in TARGET_YAW_ANGLES.
                            else:
                                # This means we've processed all nodes in the current calculated_path,
                                # and we are essentially at the end of this leg of the journey.
                                # The mission state logic above should handle what happens next (delivery, new pickup, complete).
                                print("End of current calculated path reached. Waiting for next mission step.")
                                stop_motors() # Stop and wait for the mission logic to re-plan.
                        
                        # After node handling, line following will resume naturally
                        # or the robot will stay stopped if at the goal.

                # Continue with line following if not at goal, not handling obstacle, and not waiting for button
                if not button_pressed and not (current_mission_state == MISSION_STATE_PICKUP and current_target_node == current_pickup_node):
                    if num_active_sensors > 0:
                        error = weighted_sum / num_active_sensors
                    else: # If line is lost, try to continue moving forward
                        print("Line lost! Continuing forwards, attempting to find line.")
                        set_motor_speed(motor1_pwm, motor1_in2_pin, BASE_SPEED)
                        set_motor_speed(motor2_pwm, motor2_in2_pin, BASE_SPEED)
                        sleep_ms(100) # Small delay to allow the robot to move forward a bit
                        continue # Skip the rest of the loop and try to find the line again in the next iteration

                    correction = int(error * KP)

                    left_speed = BASE_SPEED - correction
                    right_speed = BASE_SPEED + correction

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
            # Ensure left_speed and right_speed are defined before printing, in case line is lost initially
            print(f"Left Speed: {locals().get('left_speed', 'N/A')}, Right Speed: {locals().get('right_speed', 'N/A')}")
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
            print("Current Path Index:", current_path_idx)
            print("Current Target Node (in path):", current_target_node)
            if calculated_path:
                print("Current Path:", calculated_path)
            
            sleep_ms(20) # Small delay to stabilize readings and prevent busy-waiting

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

# Start the line following loop (robot will start moving after path is calculated and printed)
run_line_follower()
