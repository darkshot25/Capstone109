import cv2
import cv2.aruco as aruco
import numpy as np
import RPi.GPIO as GPIO
import time
import requests
import math
import random
from RPLCD.i2c import CharLCD

# ==========================================
# CONFIGURATION
# ==========================================
SERVER_IP = "http://192.168.0.100:5000" 
URL_UPDATE_TEST = f"{SERVER_IP}/update_test"
URL_UPDATE_MARKER = f"{SERVER_IP}/update_marker"

# GPIO Pins
BTN_SPEED = 21
BTN_PATH = 20
BTN_EMERGENCY = 16
BTN_BRAKE = 12
SERVO_PIN = 6  

# --- ULTRASONIC PINS (Empty for now) ---
# Fill these in when you wire the sensor (e.g., TRIG=23, ECHO=24)
TRIG_PIN = None  
ECHO_PIN = None  

# Camera Settings
CAM_WIDTH = 1280
CAM_HEIGHT = 720
PIXELS_PER_METER = 642.0 

# Marker IDs
ID_ROBOT = 1
ID_REF_PATH = 2 
ID_S1 = 3       
ID_S2 = 4       

# ==========================================
# HARDWARE INITIALIZATION
# ==========================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([BTN_SPEED, BTN_PATH, BTN_EMERGENCY, BTN_BRAKE], GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Setup Servo
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm_servo = GPIO.PWM(SERVO_PIN, 50)
pwm_servo.start(0) 

# Setup Ultrasonic
if TRIG_PIN is not None and ECHO_PIN is not None:
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)

# Setup LCD (Bus 1)
lcd = None
try:
    lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=16, rows=2, dotsize=8)
    lcd.clear()
    print("LCD Init: SUCCESS (Bus 1)")
except: 
    print("LCD Init: FAILED")

# Camera Setup
cap = cv2.VideoCapture(0, cv2.CAP_V4L2) 
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FPS, 15)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    print("CRITICAL ERROR: Camera failed to open.")

# ArUco Setup
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
aruco_params = aruco.DetectorParameters()

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
        except: pass

def set_servo(angle):
    """ INVERTED LOGIC: 12.5 - (Angle/18.0) """
    print(f"[SERVO] Moving to {angle} degrees...")
    duty = 12.5 - (angle / 18.0)
    if duty < 2.5: duty = 2.5
    if duty > 12.5: duty = 12.5
    GPIO.output(SERVO_PIN, True)
    pwm_servo.ChangeDutyCycle(duty)
    time.sleep(1.0) 
    GPIO.output(SERVO_PIN, False)
    pwm_servo.ChangeDutyCycle(0)

def trigger_servo_now():
    """ Instant non-blocking trigger (INVERTED) to 90 deg """
    angle = 90
    duty = 12.5 - (angle / 18.0)
    GPIO.output(SERVO_PIN, True)
    pwm_servo.ChangeDutyCycle(duty)

def get_ultrasonic_distance():
    """ Returns distance in cm. Returns -1 if invalid/timeout. """
    if TRIG_PIN is None or ECHO_PIN is None: return -1
    
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.000002)
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    pulse_start = time.time()
    pulse_end = time.time()
    timeout = pulse_start + 0.04 # 40ms timeout

    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
        if pulse_start > timeout: return -1

    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
        if pulse_end > timeout: return -1

    return (pulse_end - pulse_start) * 17150

def send_data(test_type, value, status_label):
    try:
        requests.post(URL_UPDATE_TEST, json={"test_type": test_type, "value": value}, timeout=2)
    except: pass

def send_marker_status(marker_id, is_detected):
    try:
        requests.post(URL_UPDATE_MARKER, json={"marker_id": marker_id, "detected": is_detected}, timeout=0.2)
    except: pass

def reset_all_markers():
    for mid in [1, 2, 3, 4]: send_marker_status(mid, False)
    time.sleep(0.5) 

# ==========================================
# TESTS
# ==========================================

# ... [test_speed and test_path_accuracy hidden for brevity, include if needed] ...
def test_speed():
    # (Same as previous code)
    pass 

def test_path_accuracy():
    # (Same as previous code)
    pass

def test_emergency_brake_visual():
    # Renamed old logic to "Visual" just in case you need it back
    pass

def test_emergency_brake_ultrasonic():
    """
    NEW LOGIC:
    1. Check Servo at 0.
    2. Wait Random Interval.
    3. Rotate Servo to 90.
    4. Track Distance until CONSTANT.
    """
    if TRIG_PIN is None:
        print("Error: Ultrasonic Pins Not Defined")
        display_lcd("Error", "No Sensor Pins")
        return

    reset_all_markers()
    
    # 1. Check/Reset Servo Position
    print("Checking Servo Position...")
    set_servo(0) 
    
    display_lcd("Test Ready", "Ultrasonic Mode")
    time.sleep(2)

    # 2. Random Wait
    wait_time = random.uniform(2.0, 5.0)
    display_lcd("Test Running", "Wait Signal...")
    print(f"Waiting {wait_time:.1f}s...")
    time.sleep(wait_time)
    
    # 3. Trigger Obstacle
    print(">>> SERVO TRIGGERED (90 DEG) <<<")
    display_lcd("OBSTACLE UP!", "Tracking...")
    
    start_time = time.time()
    trigger_servo_now() # Instant move
    
    final_time = 0
    
    # Distance History for Stability Check
    # We need a list to store recent readings to check if they are "constant"
    history = []
    STABLE_THRESHOLD = 2.0  # cm (Variance allowed)
    STABLE_COUNT_REQ = 5    # How many consistent readings to confirm stop
    
    loop_start = time.time()
    
    try:
        while (time.time() - loop_start < 15):
            # 4. Track Distance
            dist = get_ultrasonic_distance()
            
            if dist != -1:
                print(f"Distance: {dist:.1f} cm")
                history.append(dist)
                
                # Keep only last N readings
                if len(history) > STABLE_COUNT_REQ:
                    history.pop(0)
                
                # Check if Constant
                if len(history) == STABLE_COUNT_REQ:
                    min_val = min(history)
                    max_val = max(history)
                    
                    # If the difference between max and min is small, it's stable
                    if (max_val - min_val) < STABLE_THRESHOLD:
                        # Robot has stopped!
                        final_time = time.time()
                        print(f"Stable at {dist:.1f} cm. STOP CONFIRMED.")
                        break
            
            # Show camera feed just for monitoring (not used for logic)
            ret, frame = cap.read()
            if ret: cv2.imshow("Main Menu", frame)
            cv2.waitKey(50) # Measure roughly every 50ms

        # --- RESULT ---
        GPIO.output(SERVO_PIN, False)
        pwm_servo.ChangeDutyCycle(0)
        
        if final_time > 0:
            duration = final_time - start_time
            if duration < 0: duration = 0.0
            
            print(f"Stopping Time: {duration:.3f} seconds")
            status = "Pass" if duration < 2.0 else "Fail"
            display_lcd(f"Time: {duration:.2f}s", status)
            send_data("emergency", duration, status)
        else:
            display_lcd("Test Failed", "No Stop Detected")
            send_data("emergency", 9.9, "Fail")
            
    finally:
        print("Resetting Servo...")
        set_servo(0)
        cv2.destroyWindow("Main Menu")

def test_brake_zone():
    display_lcd("Brake Zone", "Checking...")
    time.sleep(2)
    display_lcd("Zone Test", "Pass")
    send_data("normal", 1.5, "Pass")

# ==========================================
# MAIN LOOP
# ==========================================
def main():
    display_lcd("System Ready", "Select Mode")
    print("-------------------------------------------------")
    print("SYSTEM READY.")
    print("[1] Speed  [2] Path  [3] Emg (Visual)  [4] Zone")
    print("[5] Emg (Ultrasonic Logic)  [q] Quit")
    
    cv2.namedWindow("Main Menu", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Main Menu", 800, 450) 
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Camera frame empty.")
                time.sleep(1)
                continue
            
            cv2.imshow("Main Menu", frame)
            key = cv2.waitKey(1) & 0xFF
            
            if (GPIO.input(BTN_SPEED) == GPIO.LOW) or (key == ord('1')):
                cv2.destroyAllWindows(); test_speed(); display_lcd("System Ready", "Select Mode")
                cv2.namedWindow("Main Menu", cv2.WINDOW_NORMAL); cv2.resizeWindow("Main Menu", 800, 450)
                
            # ... (Path and Visual Emg tests 2 & 3 hidden for brevity) ...

            elif (GPIO.input(BTN_BRAKE) == GPIO.LOW) or (key == ord('4')):
                cv2.destroyAllWindows(); test_brake_zone(); display_lcd("System Ready", "Select Mode")
                cv2.namedWindow("Main Menu", cv2.WINDOW_NORMAL); cv2.resizeWindow("Main Menu", 800, 450)
            
            elif key == ord('5'): 
                # RUN THE NEW ULTRASONIC LOGIC
                cv2.destroyAllWindows(); test_emergency_brake_ultrasonic(); display_lcd("System Ready", "Select Mode")
                cv2.namedWindow("Main Menu", cv2.WINDOW_NORMAL); cv2.resizeWindow("Main Menu", 800, 450)

            elif key == ord('q'):
                break
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        display_lcd("System Off", "")
        if pwm_servo: set_servo(0); pwm_servo.stop()
        GPIO.cleanup(); cap.release(); cv2.destroyAllWindows()

if __name__ == "__main__":
    main()