import time
import sys

# --- CONFIGURATION ---
# GPIO Pin Mapping (BCM Mode)
PINS = {
    "BTN_SPEED": 17,
    "BTN_PATH": 27,
    "BTN_BRAKE": 22,
    "BTN_EBRAKE": 10
}

# --- HARDWARE ABSTRACTION LAYER ---
# This block detects if we are on a Pi or PC and sets up the environment accordingly.

IS_RPI = False

try:
    import RPi.GPIO as GPIO
    # If this succeeds, we are on the Pi
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Setup Buttons with Internal Pull-Up Resistors
    # (Default High, goes Low when pressed)
    for name, pin in PINS.items():
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
    IS_RPI = True
    print("[SYSTEM] Hardware detected: Raspberry Pi GPIO Active")

except ImportError:
    # If this fails, we are on PC
    print("[SYSTEM] Hardware not detected: Running in SIMULATION MODE")
    IS_RPI = False

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

def run_emergency_brake_test():
    display.update("Mode Selected:", "EMERGENCY STOP")
    print(">>> [LOGIC] Arming Servo Motor...")
    print(">>> [LOGIC] Preparing ToF Sensor...")
    time.sleep(1)
    # Add your E-Brake Logic here later

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