import time
import sys
import random
import statistics

# --- CONFIGURATION ---
# GPIO Pin Mapping (BCM Mode)
PINS = {
    "BTN_SPEED": 17,
    "BTN_PATH": 27,
    "BTN_BRAKE": 22,
    "BTN_EBRAKE": 10
}

#Servo Configuration
SERVO_PIN = 18
OBSTACLE_DOWN_ANGLE = 90
OBSTACLE_UP_ANGLE = 0

#ToF stop threshhold
STOP_VARIANCE_LIMIT = 5.0  # If readings vary less than 5mm, robot is stopped
STOP_CHECK_WINDOW = 5      # How many readings to check for stability


# --- MOCK CLASSES (For PC Simulation) ---
#Comment this block when run on Raspberry Pi

class MockVL53L0X:
    """Simulates a ToF sensor seeing a robot approach and stop."""
    def __init__(self):
        self.current_distance = 1500  # Start at 1.5m
        self.is_moving = False
        self.start_time = 0
        
    def start_simulation(self):
        self.is_moving = True
        self.start_time = time.time()
        self.current_distance = 1500

    def range(self):
        """Returns distance in mm."""
        if not self.is_moving:
            return 1500 + random.randint(-2, 2) # Idle noise
        
        elapsed = time.time() - self.start_time
        
        # Simulate Robot Physics:
        # Move fast for 1.5 seconds, then decelerate, then stop at 2.0s
        if elapsed < 1.0:
            self.current_distance -= 40 # Moving fast
        elif elapsed < 2.0:
            self.current_distance -= 10 # Braking hard
        else:
            # Robot stopped, but sensor has noise (+/- 2mm)
            return 400 + random.randint(-2, 2) 
            
        return max(0, self.current_distance)

# --- HARDWARE ABSTRACTION LAYER ---
# This block detects if we are on a Pi or PC and sets up the environment accordingly.

IS_RPI = False
tof_sensor = None
pwm_servo = None

try:
    import RPi.GPIO as GPIO # type: ignore
    import board            # type: ignore
    import busio            # type: ignore
    import adafruit_vl53l0x # type: ignore
    # If this succeeds, we are on the Pi
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Setup Buttons with Internal Pull-Up Resistors
    # (Default High, goes Low when pressed)
    for name, pin in PINS.items():
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
    IS_RPI = True
    print("[SYSTEM] Hardware detected: Raspberry Pi GPIO Active")

    # Setup ToF (I2C)
    i2c = busio.I2C(board.SCL, board.SDA)
    tof_sensor = adafruit_vl53l0x.VL53L0X(i2c)

    # Setup Servo
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    pwm_servo = GPIO.PWM(SERVO_PIN, 50) # 50Hz
    pwm_servo.start(0)

except ImportError:
    # If this fails, we are on PC
    print("[SYSTEM] Hardware not detected: Running in SIMULATION MODE")
    tof_sensor = MockVL53L0X()
    IS_RPI = False

except Exception as e:
    print(f"[SYSTEM] Hardware Error: {e}")
    tof_sensor = MockVL53L0X()

# --- LCD CLASS DEFINITION ---
class SystemDisplay:
    def __init__(self):
        self.line1 = ""
        self.line2 = ""
        if IS_RPI:
            # Here you would initialize the real I2C LCD library
            # e.g., self.lcd = I2cCharDisplay(...)
            pass 

    def update(self, line1_text, line2_text=""):
        """Updates the display content"""
        self.line1 = line1_text
        self.line2 = line2_text
        
        if IS_RPI:
            # Code to send text to real I2C LCD
            # self.lcd.write_string(line1_text)
            pass
        
        # Always print to console for debugging/simulation
        self.print_simulation()

    def print_simulation(self):
        """Draws a fake LCD on the terminal"""
        print("\n" + "="*20)
        print(f"| {self.line1.center(16)} |")
        print(f"| {self.line2.center(16)} |")
        print("="*20 + "\n")

# --- TEST MODE FUNCTIONS ---
# These are the specific functions the code "goes to" when a button is pressed

def set_servo_angle(angle):
    if IS_RPI:
        # Map 0-180 degrees to 2.5-12.5 duty cycle
        duty = 2.5 + (angle / 18.0)
        pwm_servo.ChangeDutyCycle(duty)
        time.sleep(0.3) # Wait for movement
        pwm_servo.ChangeDutyCycle(0) # Stop jitter
    else:
        print(f"[SERVO] Moving to {angle} degrees")

def get_distance():
    if IS_RPI:
        return tof_sensor.range
    else:
        return tof_sensor.range()

def run_speed_test():
    display.update("Mode Selected:", "SPEED TEST")
    print(">>> [LOGIC] Initializing Camera...")
    print(">>> [LOGIC] Looking for QR Code...")
    time.sleep(1) # Simulate setup time
    print(">>> [LOGIC] Calculating Displacement...")
    # Add your Speed Test Logic here later

def run_path_accuracy_test():
    display.update("Mode Selected:", "PATH ACCURACY")
    print(">>> [LOGIC] Calibrating Center Line...")
    print(">>> [LOGIC] Tracking Robot Deviation...")
    time.sleep(1)
    # Add your Path Logic here later

def run_brake_test():
    display.update("Mode Selected:", "NORMAL BRAKE")
    print(">>> [LOGIC] Waiting for Trigger S1...")
    time.sleep(1)
    # Add your Brake Logic here later

def run_emergency_brake_test(display_handler = None):
    """
    Executes the full Emergency Brake Test cycle.
    Args:
        display_handler: The display object from your main code to show status.
    """
    if display_handler is None:
        class ConsoleDisplay:
            def update(self, l1, l2): print(f"[LCD-SIM] {l1} | {l2}")
        display_handler = ConsoleDisplay()
    display_handler.update("TEST: E-BRAKE", "Status: Idle")
    
    # 1. Reset System
    print("\n--- STARTING EMERGENCY BRAKE TEST ---")
    set_servo_angle(OBSTACLE_UP_ANGLE)
    time.sleep(1)
    
    # 2. Random Interval Wait (Simulate robot moving normally)
    wait_time = random.randint(2, 5) # Wait 2 to 5 seconds
    print(f"Waiting {wait_time}s for robot to reach speed...")
    
    for i in range(wait_time, 0, -1):
        display_handler.update("TEST: E-BRAKE", f"Drop in {i}s...")
        time.sleep(1)
        
    # 3. INTRODUCE OBSTACLE (Trigger)
    print(">>> TRIGGER: OBSTACLE DOWN! <<<")
    if not IS_RPI: tof_sensor.start_simulation() # Start mock physics
    
    start_time = time.time()
    set_servo_angle(OBSTACLE_DOWN_ANGLE)
    
    # 4. Measure Stopping
    display_handler.update("TEST: E-BRAKE", "Tracking...")
    
    distance_history = []
    robot_stopped = False
    final_distance = 0
    
    while not robot_stopped:
        # Read sensor
        dist = get_distance()
        current_time = time.time() - start_time
        
        print(f"Time: {current_time:.2f}s | Dist: {dist}mm")
        
        # Add to history buffer for stability check
        distance_history.append(dist)
        if len(distance_history) > STOP_CHECK_WINDOW:
            distance_history.pop(0)
            
            # CHECK STABILITY: Calculate variance/stdev of last few readings
            # If the readings aren't changing much, the robot has stopped.
            if len(distance_history) == STOP_CHECK_WINDOW:
                variance = statistics.stdev(distance_history)
                if variance < STOP_VARIANCE_LIMIT:
                    robot_stopped = True
                    final_distance = statistics.mean(distance_history)
        
        time.sleep(0.1) # Sampling rate (10Hz)
        
        # Timeout safety
        if current_time > 10.0:
            print("TIMEOUT: Robot did not stop.")
            break

    # 5. Calculate Results
    stop_duration = time.time() - start_time
    
    print("\n--- TEST COMPLETE ---")
    print(f"Stopping Time:    {stop_duration:.2f} seconds")
    print(f"Stopping Dist:    {final_distance:.0f} mm")
    
    display_handler.update("RESULT:", f"{stop_duration:.2f}s / {final_distance:.0f}mm")
    
    # 6. Reset
    time.sleep(3)
    set_servo_angle(OBSTACLE_UP_ANGLE)

# --- MAIN EXECUTION ---

# Initialize Display
display = SystemDisplay()

def main_loop():
    display.update("SYSTEM READY", "SELECT MODE")
    
    if IS_RPI:
        # --- RASPBERRY PI LOOP ---
        try:
            while True:
                # Check Button 1: Speed Test
                if GPIO.input(PINS["BTN_SPEED"]) == GPIO.LOW:
                    run_speed_test()
                    time.sleep(0.5) # Debounce delay

                # Check Button 2: Path Accuracy
                elif GPIO.input(PINS["BTN_PATH"]) == GPIO.LOW:
                    run_path_accuracy_test()
                    time.sleep(0.5)

                # Check Button 3: Brake Test
                elif GPIO.input(PINS["BTN_BRAKE"]) == GPIO.LOW:
                    run_brake_test()
                    time.sleep(0.5)

                # Check Button 4: Emergency Brake
                elif GPIO.input(PINS["BTN_EBRAKE"]) == GPIO.LOW:
                    run_emergency_brake_test()
                    time.sleep(0.5)

                time.sleep(0.1) # Prevent CPU hogging

        except KeyboardInterrupt:
            print("Exiting...")
            GPIO.cleanup()

    else:
        # --- PC SIMULATION LOOP ---
        while True:
            print("Simulate Button Press: [1] Speed [2] Path [3] Brake [4] E-Brake [Q] Quit")
            choice = input("INPUT > ").strip()

            if choice == "1":
                run_speed_test()
            elif choice == "2":
                run_path_accuracy_test()
            elif choice == "3":
                run_brake_test()
            elif choice == "4":
                run_emergency_brake_test()
            elif choice.upper() == "Q":
                break
            else:
                print("Invalid Button")

if __name__ == "__main__":
    main_loop()

'''
# --- SELF-TEST BLOCK ---
if __name__ == "__main__":
    # Mock display class for testing this file alone
    class MockDisplay:
        def update(self, l1, l2): print(f"[LCD] {l1} | {l2}")
            
    # Ensure the function accepts the argument
    run_emergency_brake_test(MockDisplay())
'''