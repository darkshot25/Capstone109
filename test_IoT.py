import requests
import time
import random

# ==========================================
# CONFIGURATION
# ==========================================

# SERVER CONFIGURATION
# Update this IP to match your server
SERVER_IP = "http://10.161.36.112:5000" 

# Endpoints derived from app.py
URL_UPDATE_TEST = f"{SERVER_IP}/update_test"
URL_UPDATE_MARKER = f"{SERVER_IP}/update_marker"

# ==========================================
# NETWORK FUNCTIONS
# ==========================================

def send_test_result(test_type, value):
    """
    Sends the final test result (Pass/Fail metrics).
    Endpoint: /update_test
    """
    payload = {
        "test_type": test_type,
        "value": value
    }
    
    print(f"[TEST UPLOAD] Sending {test_type} result: {value}...")
    try:
        response = requests.post(URL_UPDATE_TEST, json=payload, timeout=2)
        if response.status_code == 200:
            print(f"   -> Success: {response.json()}")
        else:
            print(f"   -> Error {response.status_code}: {response.text}")
    except Exception as e:
        print(f"   -> Connection Failed: {e}")

def send_marker_status(marker_id, is_detected):
    """
    Sends the ArUco marker detection status.
    Endpoint: /update_marker
    """
    payload = {
        "marker_id": marker_id,
        "detected": is_detected
    }
    
    status_str = "DETECTED" if is_detected else "LOST"
    print(f"[MARKER] ID {marker_id}: {status_str}")
    
    try:
        requests.post(URL_UPDATE_MARKER, json=payload, timeout=1)
    except Exception:
        print(f"   -> Failed to send marker status")

# ==========================================
# SIMULATION LOGIC
# ==========================================

def simulate_speed_test():
    print("\n--- SIMULATING SPEED TEST ---")
    
    # 1. Simulate Tracking the Robot (Marker ID 1)
    print("Step 1: Tracking Robot...")
    for _ in range(3):
        send_marker_status(1, True) # Robot Found
        time.sleep(0.5)
        
    # 2. Generate and Send Result
    print("Step 2: Calculation Complete")
    sim_speed = round(random.uniform(0.8, 2.5), 2)
    send_test_result("speed", sim_speed)

def simulate_path_test():
    print("\n--- SIMULATING PATH ACCURACY TEST ---")
    
    # 1. Simulate Tracking Robot (ID 1) and Reference (ID 2)
    print("Step 1: Scanning Markers...")
    send_marker_status(1, True) # Robot Found
    send_marker_status(2, True) # Path Ref Found
    time.sleep(1.0)
    
    # 2. Send Result
    sim_deviation = round(random.uniform(0.0, 20.0), 1)
    send_test_result("path", sim_deviation)

def simulate_emergency_brake():
    print("\n--- SIMULATING EMERGENCY BRAKE TEST ---")
    
    # 1. Tracking Robot
    send_marker_status(1, True)
    time.sleep(0.5)
    
    # 2. Trigger Obstacle -> Robot Stops
    print("Step 2: Obstacle Triggered!")
    time.sleep(1.0) # Simulate braking time
    
    # 3. Send Result
    stop_time = round(random.uniform(3.0, 7.0), 2)
    send_test_result("emergency", stop_time)

def simulate_brake_zone():
    print("\n--- SIMULATING NORMAL BRAKE ZONE ---")
    
    # 1. Scan for Zone Markers (ID 3 and 4)
    print("Step 1: Finding Zone Limits...")
    send_marker_status(3, True)
    send_marker_status(4, True)
    time.sleep(0.5)
    
    # 2. Track Robot entering zone
    send_marker_status(1, True)
    time.sleep(0.5)
    
    # 3. Send Result
    sim_val = round(random.uniform(0.5, 2.5), 2)
    send_test_result("normal", sim_val)

# ==========================================
# MAIN LOOP
# ==========================================

if __name__ == "__main__":
    print(f"PC IoT Simulator Started.")
    print(f"Target Server: {SERVER_IP}")
    print("-----------------------------------")
    
    try:
        while True:
            print("\nSelect Simulation:")
            print("1. Speed Test")
            print("2. Path Accuracy")
            print("3. Emergency Brake")
            print("4. Normal/Brake Zone")
            print("q. Quit")
            
            choice = input("Enter Choice: ")
            
            if choice == '1':
                simulate_speed_test()
            elif choice == '2':
                simulate_path_test()
            elif choice == '3':
                simulate_emergency_brake()
            elif choice == '4':
                simulate_brake_zone()
            elif choice == 'q':
                break
            else:
                print("Invalid choice.")
            
            time.sleep(0.5)
                
    except KeyboardInterrupt:
        print("\nExiting Simulator.")