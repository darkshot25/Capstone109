import cv2
import cv2.aruco as aruco
import numpy as np
import RPi.GPIO as GPIO
import time
import requests
import math
import board
import busio
import adafruit_vl53l0x
from RPLCD.i2c import CharLCD
from datetime import datetime
import threading

# ==========================================
# CONFIGURATION
# ==========================================

# IoT Server Configuration
IOT_SERVER_URL = "http://your-iot-server-ip:port/endpoint"  # REPLACE THIS
DEVICE_ID = "AMR_TEST_RIG_01"

# GPIO Pins (BCM Mode)
BTN_SPEED = 17
BTN_PATH = 27
BTN_EMERGENCY = 22
BTN_BRAKE = 10
SERVO_PIN = 18

# Servo Settings
SERVO_FREQ = 50 # 50Hz pulse
SERVO_OPEN_DC = 2.5  # Duty Cycle for 0 degrees (Adjust for your servo)
SERVO_CLOSE_DC = 7.5 # Duty Cycle for 90 degrees

# Camera Settings
CAM_WIDTH = 640
CAM_HEIGHT = 480
PIXELS_PER_METER = 350.0  # CALIBRATION REQUIRED: How many pixels equal 1 meter?

# AruCo Marker IDs
ID_ROBOT = 1
ID_PATH_REF = 2
ID_S1 = 3
ID_S2 = 4

# Thresholds
SPEED_MIN = 1.0
SPEED_MAX = 2.0
PATH_MAX_DEVIATION = 15.0 # Degrees
EMERGENCY_TIME_LIMIT = 5.0 # Seconds

# ==========================================
# HARDWARE INITIALIZATION
# ==========================================

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup([BTN_SPEED, BTN_PATH, BTN_EMERGENCY, BTN_BRAKE], GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Servo PWM
pwm_servo = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
pwm_servo.start(0)

# I2C Setup (LCD and ToF share the bus)
try:
    # Initialize LCD (Address usually 0x27)
    lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=16, rows=2, dotsize=8)
    lcd.clear()
    
    # Initialize ToF Sensor
    i2c = busio.I2C(board.SCL, board.SDA)
    tof = adafruit_vl53l0x.VL53L0X(i2c)
except Exception as e:
    print(f"I2C Init Error: {e}")

# Camera Setup
cap = cv2.VideoCapture(0)
cap.set(3, CAM_WIDTH)
cap.set(4, CAM_HEIGHT)

# AruCo Setup
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
aruco_params = aruco.DetectorParameters()

# ==========================================
# HELPER FUNCTIONS
# ==========================================

def display_lcd(line1, line2=""):
    """Writes text to LCD."""
    lcd.clear()
    lcd.write_string(line1)
    lcd.crlf()
    lcd.write_string(line2)
    print(f"[LCD] {line1} | {line2}")

def set_servo(angle):
    """0 = Open, 90 = Obstacle."""
    dc = SERVO_OPEN_DC if angle == 0 else SERVO_CLOSE_DC
    pwm_servo.ChangeDutyCycle(dc)
    time.sleep(0.5)
    pwm_servo.ChangeDutyCycle(0) # Stop jitter

def send_data(test_name, result_data, status):
    """Sends JSON data to server."""
    payload = {
        "session_id": f"{test_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
        "timestamp": datetime.now().isoformat(),
        "data": result_data,
        "status": status
    }
    try:
        # response = requests.post(IOT_SERVER_URL, json=payload, timeout=2) # Uncomment for real server
        print(f"Uploading: {payload}")
    except Exception as e:
        print(f"Upload Failed: {e}")

def get_aruco_positions(frame):
    """Returns a dict of found AruCo IDs and their centers (x, y)."""
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
    positions = {}
    if ids is not None:
        ids = ids.flatten()
        for (marker_corner, marker_id) in zip(corners, ids):
            corners = marker_corner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            positions[marker_id] = (cX, cY)
    return positions

# ==========================================
# TEST LOGIC FUNCTIONS
# ==========================================

def test_speed():
    display_lcd("Speed Test", "Ready...")
    time.sleep(2)
    
    positions_prev = None
    time_prev = None
    speeds = []
    
    start_time = time.time()
    
    # Run test for 5 seconds
    while time.time() - start_time < 5:
        ret, frame = cap.read()
        if not ret: continue
        
        positions = get_aruco_positions(frame)
        
        if ID_ROBOT in positions:
            curr_pos = positions[ID_ROBOT]
            curr_time = time.time()
            
            if positions_prev is not None:
                # Calculate Euclidean distance in pixels
                dist_px = math.sqrt((curr_pos[0]-positions_prev[0])**2 + (curr_pos[1]-positions_prev[1])**2)
                dist_m = dist_px / PIXELS_PER_METER
                dt = curr_time - time_prev
                
                if dt > 0:
                    speed = dist_m / dt
                    speeds.append(speed)
                    print(f"Current Speed: {speed:.2f} m/s")
            
            positions_prev = curr_pos
            time_prev = curr_time

    if len(speeds) > 0:
        avg_speed = sum(speeds) / len(speeds)
        status = "Pass" if SPEED_MIN <= avg_speed <= SPEED_MAX else "Fail"
        display_lcd(f"Spd: {avg_speed:.2f} m/s", status)
        send_data("Speed_Test", {"speed": avg_speed}, status)
    else:
        display_lcd("Error", "No Marker Found")

def test_path_accuracy():
    display_lcd("Path Acc Test", "Finding Refs...")
    time.sleep(1)
    
    ret, frame = cap.read()
    positions = get_aruco_positions(frame)
    
    if ID_ROBOT in positions and ID_PATH_REF in positions:
        p1 = positions[ID_ROBOT]
        p2 = positions[ID_PATH_REF] # Middle of track
        
        # Calculate angle of line connecting Robot and Ref relative to vertical
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        # Angle in degrees. 0 degrees is strictly vertical movement
        angle_rad = math.atan2(dx, dy) 
        angle_deg = math.degrees(angle_rad)
        deviation = abs(angle_deg)
        
        status = "Pass" if deviation < PATH_MAX_DEVIATION else "Fail"
        
        display_lcd(f"Dev: {deviation:.1f} deg", status)
        send_data("Path_Test", {"deviation": deviation}, status)
    else:
        display_lcd("Error", "Mkrs Missing")

def test_emergency_brake():
    # Ensure servo is open
    set_servo(0)
    display_lcd("Emg Brake Test", "Running...")
    
    # Random wait 2 to 5 seconds
    time.sleep(np.random.uniform(2, 5))
    
    # Trigger Obstacle
    set_servo(90)
    trigger_time = time.time()
    
    # Monitor ToF for stop
    history = []
    stop_time = 0
    robot_stopped = False
    
    while not robot_stopped and (time.time() - trigger_time < 10):
        try:
            dist = tof.range
            history.append(dist)
            if len(history) > 5: history.pop(0)
            
            # Check if distance is constant (variance is low)
            if len(history) == 5 and max(history) - min(history) < 10: # 10mm tolerance
                robot_stopped = True
                stop_time = time.time() - trigger_time
        except:
            pass
        time.sleep(0.1)

    # Reset Servo
    set_servo(0)
    
    if robot_stopped:
        status = "Pass" if stop_time < EMERGENCY_TIME_LIMIT else "Fail"
        display_lcd(f"Stop: {stop_time:.2f}s", status)
        send_data("Emg_Brake", {"stop_time": stop_time}, status)
    else:
        display_lcd("Fail", "No Stop Detect")
        send_data("Emg_Brake", {"stop_time": -1}, "Fail")

def test_brake_zone():
    display_lcd("Brake Zone Test", "Wait for Stop...")
    
    # 1. Capture Reference positions first
    ret, frame = cap.read()
    positions = get_aruco_positions(frame)
    
    if ID_S1 not in positions or ID_S2 not in positions:
        display_lcd("Error", "Zones Missing")
        return

    # Assume S1 is first line (lower Y pixel value if top-down view?), S2 is second
    # Adjust logic based on your camera orientation.
    # Here assuming Robot moves from Top of image (low Y) to Bottom (High Y)
    y_s1 = positions[ID_S1][1]
    y_s2 = positions[ID_S2][1]
    
    # 2. Wait for robot to stop moving
    last_pos = (0,0)
    stable_count = 0
    
    while stable_count < 10: # wait for 10 frames of no movement
        ret, frame = cap.read()
        positions = get_aruco_positions(frame)
        if ID_ROBOT in positions:
            curr_pos = positions[ID_ROBOT]
            dist = math.sqrt((curr_pos[0]-last_pos[0])**2 + (curr_pos[1]-last_pos[1])**2)
            
            if dist < 2: # barely moved
                stable_count += 1
            else:
                stable_count = 0
            last_pos = curr_pos
        time.sleep(0.1)
        
    # 3. Check position
    robot_y = last_pos[1]
    
    # Logic: Start -> S1 -> S2 -> End
    # Pass if S1 < Robot < S2
    
    status = ""
    res_text = ""
    
    if robot_y < y_s1:
        status = "Fail (Over)" # Stopped too early
        res_text = "Overbrake"
    elif robot_y > y_s2:
        status = "Fail (Under)" # Stopped too late
        res_text = "Underbrake"
    else:
        status = "Pass"
        res_text = "Good Stop"
        
    display_lcd(res_text, status)
    send_data("Brake_Zone", {"position_y": robot_y, "s1": y_s1, "s2": y_s2}, status)

# ==========================================
# MAIN LOOP
# ==========================================

def main():
    display_lcd("System Ready", "Select Mode")
    
    try:
        while True:
            if GPIO.input(BTN_SPEED) == GPIO.LOW:
                test_speed()
                time.sleep(2)
                display_lcd("System Ready", "Select Mode")
                
            elif GPIO.input(BTN_PATH) == GPIO.LOW:
                test_path_accuracy()
                time.sleep(2)
                display_lcd("System Ready", "Select Mode")

            elif GPIO.input(BTN_EMERGENCY) == GPIO.LOW:
                test_emergency_brake()
                time.sleep(2)
                display_lcd("System Ready", "Select Mode")

            elif GPIO.input(BTN_BRAKE) == GPIO.LOW:
                test_brake_zone()
                time.sleep(2)
                display_lcd("System Ready", "Select Mode")
                
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        display_lcd("System Off", "")
        pwm_servo.stop()
        GPIO.cleanup()
        cap.release()

if __name__ == "__main__":
    main()