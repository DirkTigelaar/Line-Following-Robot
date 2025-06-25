# THIS CODE READS ALL THE SENSOR VALUES
# LINE FOLLOWING
# PATH FOLLOWING
# OBSTACLE AVOIDANCE
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

# Button
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

pin_a2 = Pin(17, Pin.IN, Pin.PULL_UP)
pin_b2 = Pin(16, Pin.IN, Pin.PULL_UP)

position1 = 0
last_state1 = pin_a1.value()

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
NODE_STOP_TIME_MS = 500 
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
                # The 'blocked_nodes' parameter is kept for general use,
                # but for this specific obstacle avoidance, we rely on high edge weights.
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
    # Pattern for T-junctions or intersections (many sensors on line)
    # This captures wide lines.
    if num_active_sensors >= NODE_SENSOR_THRESHOLD:
        return True

    # Pattern for a distinct side road to the left (leftmost and center on line, rightmost off)
    # This helps catch 90-degree turns that might not activate many sensors for long
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

# --- Hardcoded Start and Goal Nodes ---
START_NODE = "E6"
GOAL_NODE = "C1"

# Global variables for yaw calculation and node detection cooldown
yaw_angle = 0.0 # Yaw angle in radians
last_time = ticks_ms()
last_node_detection_time = 0 # Initialize for cooldown
obstacle_detected_flag = False # New global flag for obstacle detection

# Global variables to manage path
calculated_path = [] # Will be populated initially and recalculated
current_path_idx = 0

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
YAW_TOLERANCE = 10.0 * (pi / 180.0) # Converted from 10 degrees to radians for precision

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
            turn_direction_str = "LEFT (CCW) - Spin"
        else: # Need to turn clockwise (Right)
            set_motor_speed(motor1_pwm, motor1_in2_pin, TURN_SPEED)  # Left wheel forward
            set_motor_speed(motor2_pwm, motor2_in2_pin, -TURN_SPEED) # Right wheel backward
            turn_direction_str = "RIGHT (CW) - Spin"

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


# ----- Line Following Control Loop -----
def run_line_follower():
    """Main loop for line following and node detection and path navigation."""
    global yaw_angle, last_time, current_path_idx, last_node_detection_time, obstacle_detected_flag, \
           calculated_path # Declare global to modify

    print(f"\nCalculated path from {START_NODE} to {GOAL_NODE}: {calculated_path}")

    try:
        while True:
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

                    # Determine the segment that is blocked by the obstacle based on the current_path_idx
                    # current_path_idx here indicates the target node the robot is currently moving towards.
                    blocked_segment_start_node = None
                    blocked_segment_end_node = None

                    # If current_path_idx is 0, the robot is trying to move from START_NODE to calculated_path[0].
                    # The obstacle is on this very first segment.
                    if current_path_idx == 0:
                        blocked_segment_start_node = START_NODE
                        if len(calculated_path) > 0: # Ensure there's a target node
                            blocked_segment_end_node = calculated_path[0]
                    else:
                        # If current_path_idx > 0, it means the robot has successfully passed
                        # calculated_path[current_path_idx - 1] and is now trying to move
                        # towards calculated_path[current_path_idx].
                        # The obstacle is detected on the segment from calculated_path[current_path_idx - 1]
                        # to calculated_path[current_path_idx].
                        if current_path_idx < len(calculated_path): # Ensure current_path_idx is a valid target node
                            blocked_segment_start_node = calculated_path[current_path_idx - 1]
                            blocked_segment_end_node = calculated_path[current_path_idx]
                        else:
                            # This scenario means current_path_idx is beyond the end of the calculated_path,
                            # implying the robot is at or past the goal, or path is invalid.
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
                        # Even if no specific segment is blocked, we still try to recalculate the path.
                        
                    # Perform the 180-degree turn
                    target_yaw_180 = normalize_angle_rad(yaw_angle + pi)
                    orient_robot(target_yaw_180, spin_in_place=True) 
                    
                    # Recalculate path from the last successfully reached node to the goal.
                    # This corresponds to blocked_segment_start_node.
                    current_robot_node = blocked_segment_start_node 
                    
                    # Recalculate the path. The find_path_dijkstra now uses the grid with increased weights.
                    new_calculated_path = find_path_dijkstra(current_robot_node, GOAL_NODE, corrected_weighted_grid)
                    
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
                        except ValueError:
                            # This should ideally not happen if current_robot_node is in the new path
                            print("Warning: Last reached node not found in new path. Resetting index to 0.")
                            current_path_idx = 0 
                        
                        # --- REMOVED: Immediate re-orientation after recalculation. Robot will now line-follow. ---
                        # The robot will now naturally proceed to line-follow on the new path.
                        # The re-orientation for the next segment will happen when it detects the next node.
                        
                        obstacle_detected_flag = False # Reset flag to allow normal operation
            
            # Handle obstacle clearance *after* any potential re-calculation and re-orientation
            if not is_obstacle_currently_present and obstacle_detected_flag:
                # This block will likely be hit if the obstacle was only momentarily detected
                # or if the turn effectively moved the robot away from the obstacle's direct line of sight.
                print("--- Obstacle cleared. Resuming. ---")
                obstacle_detected_flag = False
            
            # This main block only executes if no obstacle is currently preventing movement
            if not obstacle_detected_flag:
                # Button state
                button_pressed = button.value() == 0
                
                # Activate electromagnet
                if button.value() == 1:
                    electromagnet.off()
                else:
                    electromagnet.on()
                
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
                # Now using the more robust detection function
                if is_node_detected_robust(ir_values, num_active_sensors):
                    # Check if enough time has passed since the last node detection
                    if (current_time - last_node_detection_time) >= NODE_DETECTION_COOLDOWN_MS:
                        stop_motors()
                        print("\n*** NODE DETECTED! Stopping briefly. ***")
                        sleep_ms(NODE_STOP_TIME_MS)
                        
                        # --- MODIFICATION: Drive forward for 1 second to clear the node ---
                        print("Driving forward for 1 second to clear node with wheels...")
                        set_motor_speed(motor1_pwm, motor1_in2_pin, BASE_SPEED)
                        set_motor_speed(motor2_pwm, motor2_in2_pin, BASE_SPEED)
                        sleep(1) # Drive forward for 1 second
                        stop_motors() # Stop after driving forward
                        # --- END MODIFICATION ---

                        last_node_detection_time = current_time # Update last detection time

                        # Check if we are still on the path
                        if current_path_idx < len(calculated_path) - 1:
                            current_node_name = calculated_path[current_path_idx]
                            next_node_name = calculated_path[current_path_idx + 1]
                            
                            direction_to_next = get_direction_between_nodes(current_node_name, next_node_name, corrected_weighted_grid)
                            
                            if direction_to_next and direction_to_next in TARGET_YAW_ANGLES:
                                target_yaw = TARGET_YAW_ANGLES[direction_to_next]
                                print(f"Current node: {current_node_name}, Next node: {next_node_name}")
                                print(f"Required turn direction: {direction_to_next}, Target Yaw: {target_yaw * 180/pi:.2f} deg")
                                orient_robot(target_yaw) # Orient the robot
                                current_path_idx += 1 # Advance to the next node in the path
                            else:
                                print(f"Warning: Could not determine direction from {current_node_name} to {next_node_name} or direction is not recognized.")
                                current_path_idx += 1 # Still advance, to avoid getting stuck at this node
                        elif current_path_idx == len(calculated_path) - 1:
                            print("*** GOAL NODE REACHED! ***")
                            stop_motors()
                            # Optionally, add a behavior for reaching the goal, e.g., electromagnet action, sound.
                            while True: # Keep robot stopped at goal
                                sleep(1) # Sleep indefinitely or until reset
                        else:
                            # This case should ideally not be reached if logic is sound,
                            # but it covers scenarios where a node is detected beyond the end of the path.
                            print("Node detected, but path already completed or index out of bounds.")
                            stop_motors() # Stop to prevent unexpected movement

                        # After node handling (orientation or goal reached), line following will resume naturally
                        # or the robot will stay stopped if at the goal.

                # Continue with line following if not at goal or if orientation is complete
                if num_active_sensors > 0:
                    error = weighted_sum / num_active_sensors
                else: # If line is lost, continue moving forward
                    print("Line lost! Continuing forwards.")
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
                # If obstacle_detected_flag is still True at this point, it means an obstacle is still present
                # (e.g., if the turn didn't fully clear it, or if it reappeared immediately after turn)
                stop_motors()
                sleep_ms(50) # Small delay to prevent busy-waiting while waiting for obstacle to clear


            # Print current robot state for debugging
            print("\n--- Current Robot State ---")
            print("IR Sensors:", ir_values)
            print("Error:", error)
            print("Correction:", correction)
            print(f"Left Speed: {left_speed}, Right Speed: {right_speed}")
            print("Encoder 1 Count:", position1)
            print("Encoder 2 Count:", position2)
            print("Distance: {:.2f} cm".format(dist) if dist != -1 else "Ultrasonic: Timeout")
            print("Button Pressed:", button_pressed)
            print("Yaw angle (deg): {:.2f}".format(yaw_angle * 180/pi))
            print("Obstacle Detected Flag:", obstacle_detected_flag)
            print("Current Path Index:", current_path_idx)
            if calculated_path:
                # The displayed "Current Logical Node" here refers to the target node of the segment being traversed
                print("Current Logical Node:", calculated_path[current_path_idx] if current_path_idx < len(calculated_path) else "At Goal or Beyond")
            # The 'blocked_path_segments' is no longer used for dynamic blocking in this version,
            # as blocking is now done via increasing edge weights directly in corrected_weighted_grid.
            # print("Blocked Path Segments (Nodes):", blocked_path_segments)
            
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

# 3. Calculate the initial path using Dijkstra's
calculated_path = find_path_dijkstra(START_NODE, GOAL_NODE, corrected_weighted_grid)
print(f"Initial Path from {START_NODE} to {GOAL_NODE}: {calculated_path}")

# 4. Set initial yaw angle based on the first segment of the path
if len(calculated_path) > 1:
    initial_direction = get_direction_between_nodes(calculated_path[0], calculated_path[1], corrected_weighted_grid)
    if initial_direction and initial_direction in TARGET_YAW_ANGLES:
        yaw_angle = TARGET_YAW_ANGLES[initial_direction]
        print(f"Initial yaw set to {yaw_angle:.2f} rad ({yaw_angle * 180/pi:.2f} deg) based on direction '{initial_direction}' from {calculated_path[0]} to {calculated_path[1]}.")
    else:
        print("Warning: Could not set initial yaw based on path direction. Defaulting to 0 rad.")
        yaw_angle = 0.0 # Default if direction cannot be determined
else:
    print("Path has only one node or is empty. Initial yaw set to 0 rad.")
    yaw_angle = 0.0 # Default for single-node paths

# Start the line following loop (robot will start moving after path is calculated and printed)
run_line_follower()
