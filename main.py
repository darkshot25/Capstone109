import cv2
import cv2.aruco as aruco
import numpy as np
import RPi.GPIO as GPIO
import time
import requests
import math
import random
from RPLCD.i2c import CharLCD
from datetime import datetime

# ==========================================
# CONFIGURATION
# ==========================================

# SET THIS TO 'True' TO TEST WITHOUT REAL SENSOR
# SET TO 'False' TO TRY READING THE REAL ULTRASONIC SENSOR
SIMULATION_MODE = True 

# IoT Server Configuration
IOT_SERVER_URL = "http://192.168.0.100:5000/data" 
DEVICE_ID = "AMR_TEST_RIG_01"

# GPIO Pins (BCM Mode)
BTN_SPEED = 17
BTN_PATH = 27
BTN_EMERGENCY = 22
BTN_BRAKE = 10
SERVO_PIN = 18

# Ultrasonic Pins (Reusing old ToF pins)
TRIG_PIN = 4
ECHO_PIN = 5

# Servo Settings
SERVO_FREQ = 50 
SERVO_OPEN_DC = 2.5  
SERVO_CLOSE_DC = 7.5 

# Camera Settings
CAM_WIDTH = 640
CAM_HEIGHT = 480
PIXELS_PER_METER = 642.0  # Calibrated Value

# AruCo Marker IDs
ID_ROBOT = 1
ID_REF_PATH = 2 
ID_S1 = 3       
ID_S2 = 4       

# Thresholds
SPEED_MIN = 1.0
SPEED_MAX = 2.0
PATH_MAX_DEVIATION = 15.0 
EMERGENCY_TIME_LIMIT = 5.0 

# ==========================================
# HARDWARE INITIALIZATION
# ==========================================

GPIO.setmode(GPIO.BCM)
GPIO.setup([BTN_SPEED, BTN_PATH, BTN_EMERGENCY, BTN_BRAKE], GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

GPIO.output(TRIG_PIN, False)

pwm_servo = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
pwm_servo.start(0)

# LCD Setup (Bus 1)
lcd = None
try:
    lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=16, rows=2, dotsize=8)
    lcd.clear()
    print("LCD Init: SUCCESS")
except Exception as e:
    print(f"LCD Init FAILED: {e}")

# Camera Setup
cap = cv2.VideoCapture(0)
cap.set(3, CAM_WIDTH)
cap.set(4, CAM_HEIGHT)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
aruco_params = aruco.DetectorParameters()

# Global variable for simulation
sim_distance = 1500 

# ==========================================
# HELPER FUNCTIONS
# ==========================================

def display_lcd(line1, line2=""):
    print(f"[DISPLAY] {line1} | {line2}")
    if lcd:
        try:
            lcd.clear()
            lcd.write_string(line1)
            lcd.crlf()
            lcd.write_string(line2)
        except:
            pass

def set_servo(angle):
    dc = SERVO_OPEN_DC if angle == 0 else SERVO_CLOSE_DC
    pwm_servo.ChangeDutyCycle(dc)
    time.sleep(0.5)
    pwm_servo.ChangeDutyCycle(0)

def send_data(test_name, result_data, status):
    payload = {
        "session_id": f"{test_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
        "timestamp": datetime.now().isoformat(),
        "device_id": DEVICE_ID,
        "data": result_data,
        "status": status
    }
    try:
        # requests.post(IOT_SERVER_URL, json=payload, timeout=2) 
        print(f"Uploading: {payload}")
    except Exception as e:
        print(f"Upload Failed: {e}")

def get_aruco_positions(frame):
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

def get_distance():
    """Handles both Simulation and Real Ultrasonic Sensor"""
    global sim_distance

    if SIMULATION_MODE:
        # Logic: Robot approaches (dist decreases) then stops
        if sim_distance > 300:
            sim_distance -= 60  # Move 60mm closer per tick
            noise = random.randint(-5, 5)
            return sim_distance + noise
        else:
            # Robot stopped at 300mm
            return 300 + random.randint(-2, 2)
    else:
        # REAL SENSOR LOGIC
        try:
            GPIO.output(TRIG_PIN, True)
            time.sleep(0.00001)
            GPIO.output(TRIG_PIN, False)

            timeout = time.time() + 0.04 # 40ms timeout
            pulse_start = time.time()
            while GPIO.input(ECHO_PIN) == 0:
                pulse_start = time.time()
                if pulse_start > timeout: return -1

            pulse_end = time.time()
            while GPIO.input(ECHO_PIN) == 1:
                pulse_end = time.time()
                if pulse_end > timeout: return -1

            pulse_duration = pulse_end - pulse_start
            distance_mm = (pulse_duration * 343000) / 2
            return int(distance_mm)
        except:
            return -1

# ==========================================
# TEST SCENARIOS
# ==========================================

def test_speed():
    display_lcd("Speed Test", "Running...")
    time.sleep(1)
    
    positions_prev = None
    time_prev = None
    speeds = []
    
    start_time = time.time()
    
    while time.time() - start_time < 5:
        ret, frame = cap.read()
        if not ret: continue
        
        positions = get_aruco_positions(frame)
        
        if ID_ROBOT in positions:
            curr_pos = positions[ID_ROBOT]
            curr_time = time.time()
            
            if positions_prev is not None:
                dist_px = math.sqrt((curr_pos[0]-positions_prev[0])**2 + (curr_pos[1]-positions_prev[1])**2)
                dist_m = dist_px / PIXELS_PER_METER
                dt = curr_time - time_prev
                
                if dt > 0:
                    speed = dist_m / dt
                    speeds.append(speed)
                    print(f"Speed: {speed:.2f} m/s")
            
            positions_prev = curr_pos
            time_prev = curr_time

    if len(speeds) > 0:
        avg_speed = sum(speeds) / len(speeds)
        status = "Pass" if SPEED_MIN <= avg_speed <= SPEED_MAX else "Fail"
        display_lcd(f"Avg: {avg_speed:.2f} m/s", status)
        send_data("Speed_Test", {"speed": avg_speed}, status)
    else:
        display_lcd("Error", "Marker Not Seen")

def test_path_accuracy():
    display_lcd("Path Acc Test", "Scanning...")
    time.sleep(1)
    
    ret, frame = cap.read()
    positions = get_aruco_positions(frame)
    
    if ID_ROBOT in positions and ID_REF_PATH in positions:
        p1 = positions[ID_ROBOT]
        p2 = positions[ID_REF_PATH]
        
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        angle_rad = math.atan2(dy, dx) 
        angle_deg = math.degrees(angle_rad)
        
        deviation = abs(angle_deg)
        if deviation > 90:
            deviation = abs(180 - deviation)
            
        status = "Pass" if deviation < PATH_MAX_DEVIATION else "Fail"
        
        display_lcd(f"Dev: {deviation:.1f} deg", status)
        send_data("Path_Test", {"deviation": deviation}, status)
    else:
        display_lcd("Error", "Missing Markers")

def test_emergency_brake():
    # Reset Simulation Variable
    global sim_distance
    sim_distance = 1500 
    
    set_servo(0)
    display_lcd("Emg Brake Test", "Wait for Trig...")
    
    time.sleep(2) 
    
    # Trigger Obstacle
    set_servo(90)
    trigger_time = time.time()
    display_lcd("OBSTACLE!", "Tracking...")
    
    history = []
    stop_time = 0
    robot_stopped = False
    
    # Monitor Loop
    while not robot_stopped and (time.time() - trigger_time < 10):
        
        dist = get_distance()
        print(f"Dist: {dist}mm") # Debug print
        
        if dist > 0 and dist < 3000:
            history.append(dist)
            if len(history) > 5: history.pop(0)
            
            # Stop Condition (20mm tolerance for Ultrasonic noise)
            if len(history) == 5 and (max(history) - min(history) < 20):
                robot_stopped = True
                stop_time = time.time() - trigger_time
        
        time.sleep(0.1)

    set_servo(0) 
    
    if robot_stopped:
        status = "Pass" if stop_time < EMERGENCY_TIME_LIMIT else "Fail"
        display_lcd(f"Time: {stop_time:.2f}s", status)
        send_data("Emg_Brake", {"stop_time": stop_time}, status)
    else:
        display_lcd("Fail", "Did Not Stop")
        send_data("Emg_Brake", {"stop_time": -1}, "Fail")

def test_brake_zone():
    display_lcd("Brake Zone", "Scanning Refs...")
    
    ret, frame = cap.read()
    positions = get_aruco_positions(frame)
    
    if ID_S1 not in positions or ID_S2 not in positions:
        display_lcd("Error", "Ref 3 or 4 Missing")
        return

    x_coords = sorted([positions[ID_S1][0], positions[ID_S2][0]])
    limit_left = x_coords[0]
    limit_right = x_coords[1]
    
    display_lcd("Move Robot", "Wait for Stop...")
    
    last_pos = (0,0)
    stable_count = 0
    
    while stable_count < 10: 
        ret, frame = cap.read()
        positions = get_aruco_positions(frame)
        if ID_ROBOT in positions:
            curr_pos = positions[ID_ROBOT]
            dist = math.sqrt((curr_pos[0]-last_pos[0])**2 + (curr_pos[1]-last_pos[1])**2)
            
            if dist < 3: 
                stable_count += 1
            else:
                stable_count = 0
            last_pos = curr_pos
        time.sleep(0.1)
        
    robot_x = last_pos[0]
    
    status = "Fail"
    res_text = ""
    
    if limit_left < robot_x < limit_right:
        status = "Pass"
        res_text = "Good Stop"
    elif robot_x < limit_left:
        res_text = "Fail: Too Left"
    else:
        res_text = "Fail: Too Right"
        
    display_lcd(res_text, status)
    send_data("Brake_Zone", {"pos_x": robot_x, "lim_l": limit_left, "lim_r": limit_right}, status)

# ==========================================
# MAIN LOOP
# ==========================================

def main():
    display_lcd("System Ready", "Select Mode")
    print("System Running.")
    print(f"Simulation Mode: {SIMULATION_MODE}")
    
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
        print("\nExiting...")
        display_lcd("System Off", "")
        pwm_servo.stop()
        GPIO.cleanup()
        cap.release()

if __name__ == "__main__":
    main()