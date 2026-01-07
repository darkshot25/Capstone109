import time
import threading
import random
from datetime import datetime
from flask import Flask, render_template_string, jsonify

# ==========================================
# MOCK HARDWARE STATE
# ==========================================
system_state = {
    "status": "IDLE",
    "connection": "ONLINE",
    "markers": [],
    "console_log": [],
    "test_result": "READY",
    "last_value": "--"
}

app = Flask(__name__)

# ==========================================
# HELPER FUNCTIONS
# ==========================================
def log(msg):
    timestamp = datetime.now().strftime("%H:%M:%S")
    entry = f"[{timestamp}] {msg}"
    system_state["console_log"].insert(0, entry)
    if len(system_state["console_log"]) > 50:
        system_state["console_log"].pop()
    print(entry)

def simulate_delay(seconds):
    time.sleep(seconds)

# ==========================================
# SIMULATED TEST LOGIC
# ==========================================
def run_speed_test_logic():
    system_state["status"] = "RUNNING: SPEED TEST"
    system_state["test_result"] = "..."
    system_state["last_value"] = "..."
    log("Started Speed Test. Initializing Camera...")
    simulate_delay(1.0)
    log("Tracking Robot (Marker ID 1)...")
    system_state["markers"] = [1] 
    simulate_delay(2.5)
    speed = round(random.uniform(1.0, 2.2), 2)
    is_pass = 1.0 <= speed <= 2.0
    log(f"Calculation Complete. Speed: {speed} m/s")
    system_state["markers"] = [] 
    system_state["last_value"] = f"{speed} m/s"
    system_state["test_result"] = "PASS" if is_pass else "FAIL"
    system_state["status"] = "IDLE"

def run_path_test_logic():
    system_state["status"] = "RUNNING: PATH ACCURACY"
    system_state["test_result"] = "..."
    log("Started Path Test. Scanning for Reference...")
    simulate_delay(1.0)
    system_state["markers"] = [1, 2] 
    log("Markers Acquired. Measuring deviation...")
    simulate_delay(2.0)
    deviation = round(random.uniform(0.0, 20.0), 1)
    is_pass = deviation < 15.0
    log(f"Deviation Measured: {deviation} degrees")
    system_state["markers"] = []
    system_state["last_value"] = f"{deviation}°"
    system_state["test_result"] = "PASS" if is_pass else "FAIL"
    system_state["status"] = "IDLE"

def run_emergency_test_logic():
    system_state["status"] = "RUNNING: EMG BRAKE"
    system_state["test_result"] = "..."
    log("Emergency Brake Test Initiated.")
    simulate_delay(1.5)
    log("Waiting for robot trigger...")
    system_state["markers"] = [1]
    simulate_delay(1.5)
    log("TRIGGER: Obstacle Activated (Servo 90deg)")
    system_state["markers"] = [] 
    simulate_delay(3.0)
    log("Test Cycle Complete. Resetting Servo.")
    system_state["last_value"] = "Manual"
    system_state["test_result"] = "PASS"
    system_state["status"] = "IDLE"

def run_zone_test_logic():
    system_state["status"] = "RUNNING: BRAKE ZONE"
    system_state["test_result"] = "..."
    log("Zone Test Started. Scanning Zone Limits...")
    system_state["markers"] = [3, 4] 
    simulate_delay(1.0)
    log("Waiting for Robot entry...")
    system_state["markers"] = [1, 3, 4] 
    simulate_delay(2.0)
    log("Robot Stop Detected.")
    zone_result = random.choice(["PASS", "FAIL"])
    system_state["markers"] = []
    system_state["last_value"] = zone_result
    system_state["test_result"] = zone_result
    system_state["status"] = "IDLE"

# ==========================================
# FLASK WEB SERVER (Styled Like Reference Image)
# ==========================================

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AMR Safety Dashboard</title>
    <link href="https://fonts.googleapis.com/css2?family=Roboto:wght@400;500;700;900&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg-color: #f0f2f5;       /* Light Gray Background */
            --card-bg: #ffffff;        /* White Cards */
            --nav-bg: #2d3436;         /* Dark Header */
            --text-dark: #2d3436;      /* Main Value Color */
            --label-gold: #e67e22;     /* The Gold/Orange Labels */
            --accent-green: #27ae60;   /* Success Green */
            --accent-red: #c0392b;     /* Fail Red */
            --shadow: 0 4px 20px rgba(0,0,0,0.08); /* Soft Shadow */
        }

        body {
            font-family: 'Roboto', sans-serif;
            background-color: var(--bg-color);
            margin: 0;
            padding: 0;
            height: 100vh;
            display: flex;
            flex-direction: column;
        }

        /* HEADER / NAVIGATION */
        .navbar {
            background-color: var(--nav-bg);
            padding: 0 40px;
            height: 70px;
            display: flex;
            align-items: center;
            justify-content: space-between;
            color: white;
            box-shadow: 0 2px 10px rgba(0,0,0,0.2);
        }

        .nav-tabs {
            display: flex;
            gap: 10px;
            height: 100%;
            align-items: center;
        }

        .nav-tab {
            padding: 10px 25px;
            font-size: 14px;
            font-weight: 700;
            color: #bdc3c7;
            text-transform: uppercase;
            letter-spacing: 1px;
            cursor: pointer;
            border-radius: 4px;
            transition: all 0.2s;
        }
        
        /* The "White Pill" active state from your image */
        .nav-tab.active {
            background-color: white;
            color: var(--text-dark);
            box-shadow: 0 2px 5px rgba(0,0,0,0.2);
        }

        /* MAIN DASHBOARD GRID */
        .dashboard {
            padding: 30px 40px;
            display: grid;
            grid-template-columns: 1fr 1fr; /* 2 Columns */
            grid-template-rows: auto auto;  /* 2 Rows */
            gap: 25px;
            max-width: 1200px;
            margin: 0 auto;
            width: 100%;
            box-sizing: border-box;
        }

        /* CARD STYLE */
        .card {
            background: var(--card-bg);
            border-radius: 12px;
            padding: 25px;
            box-shadow: var(--shadow);
            display: flex;
            flex-direction: column;
            justify-content: space-between;
            min-height: 180px;
            position: relative;
        }

        /* TYPOGRAPHY MATCHING IMAGE */
        .card-label {
            color: var(--label-gold);
            font-size: 13px;
            font-weight: 900;
            text-transform: uppercase;
            letter-spacing: 0.5px;
            margin-bottom: 10px;
        }

        .card-value {
            color: var(--text-dark);
            font-size: 42px;
            font-weight: 900;
            margin-top: 5px;
        }

        .card-subtext {
            color: #7f8c8d;
            font-size: 14px;
            margin-top: 5px;
            font-weight: 500;
        }

        /* TELEMETRY CIRCLES (Bottom Right Battery style) */
        .telemetry-container {
            display: flex;
            gap: 20px;
            align-items: center;
            margin-top: 15px;
        }
        .marker-circle {
            width: 50px;
            height: 50px;
            border-radius: 50%;
            border: 3px solid #eee;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 10px;
            font-weight: bold;
            color: #ccc;
            flex-direction: column;
            transition: all 0.3s;
        }
        .marker-circle.active {
            border-color: var(--accent-green);
            color: var(--accent-green);
            background: rgba(39, 174, 96, 0.1);
            transform: scale(1.1);
        }
        /* Icon inside circle */
        .marker-circle span { font-size: 18px; display: block; margin-bottom: -2px;}

        /* CONTROL BUTTONS (The "Ver Grafica" style) */
        .control-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
            margin-top: 10px;
        }

        .btn-control {
            background: #2d3436; /* Dark button like image */
            color: white;
            border: none;
            padding: 15px;
            border-radius: 8px;
            font-weight: 700;
            font-size: 13px;
            text-transform: uppercase;
            cursor: pointer;
            display: flex;
            align-items: center;
            justify-content: space-between;
            transition: transform 0.1s, background 0.2s;
        }
        .btn-control:hover { background: #454d4f; }
        .btn-control:active { transform: translateY(2px); }
        .btn-control:disabled { opacity: 0.5; cursor: not-allowed; }
        
        /* Small arrow icon */
        .btn-control::after { content: '→'; font-size: 16px; margin-left: 10px; }

        /* STATUS BADGES */
        .status-badge {
            display: inline-block;
            padding: 5px 12px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: bold;
            text-transform: uppercase;
        }
        .status-pass { color: var(--accent-green); background: rgba(39, 174, 96, 0.15); }
        .status-fail { color: var(--accent-red); background: rgba(192, 57, 43, 0.15); }
        .status-ready { color: #7f8c8d; background: #ecf0f1; }

        /* BOTTOM BAR (Toggles style) */
        .bottom-bar {
            background: white;
            margin: 0 40px 30px 40px;
            padding: 15px 30px;
            border-radius: 50px;
            box-shadow: var(--shadow);
            display: flex;
            justify-content: space-between;
            align-items: center;
            max-width: 1200px;
            width: calc(100% - 80px);
            margin: auto auto 30px auto;
        }
        .log-line { font-family: monospace; font-size: 12px; color: #7f8c8d; }

    </style>
</head>
<body>

    <div class="navbar">
        <div style="font-weight: 900; letter-spacing: 1px;">AMR SAFETY RIG</div>
        <div class="nav-tabs">
            <div class="nav-tab active">DASHBOARD</div>
            <div class="nav-tab">LOGS</div>
            <div class="nav-tab">SETTINGS</div>
            <div class="nav-tab">HELP</div>
        </div>
    </div>

    <div class="dashboard">
        
        <div class="card">
            <div>
                <div class="card-label">CURRENT RESULT</div>
                <div id="ui-result-val" class="card-value">--</div>
            </div>
            <div style="text-align: right;">
                 <div id="ui-result-status" class="status-badge status-ready">READY</div>
            </div>
        </div>

        <div class="card">
            <div>
                <div class="card-label">SYSTEM STATUS</div>
                <div id="ui-sys-status" class="card-value" style="font-size: 32px;">IDLE</div>
            </div>
             <button class="btn-control" style="width: auto; align-self: flex-end; margin-top: auto;" disabled>
                System Online
            </button>
        </div>

        <div class="card">
            <div class="card-label">ACTIVE SENSORS (ARUCO)</div>
            <div class="card-subtext">Real-time detection status</div>
            
            <div class="telemetry-container">
                <div id="m1" class="marker-circle"><span>🤖</span>ID 1</div>
                <div id="m2" class="marker-circle"><span>🛣️</span>ID 2</div>
                <div id="m3" class="marker-circle"><span>🛑</span>ID 3</div>
                <div id="m4" class="marker-circle"><span>🛑</span>ID 4</div>
            </div>
        </div>

        <div class="card">
            <div class="card-label">TEST CONTROLS</div>
            <div class="control-grid">
                <button class="btn-control" onclick="triggerTest('speed')">Speed Test</button>
                <button class="btn-control" onclick="triggerTest('path')">Path Test</button>
                <button class="btn-control" onclick="triggerTest('emergency')">Emg. Brake</button>
                <button class="btn-control" onclick="triggerTest('zone')">Brake Zone</button>
            </div>
        </div>

    </div>

    <div class="bottom-bar">
        <div style="font-weight: bold; font-size: 12px; color: #ccc; text-transform: uppercase;">LATEST LOG EVENT</div>
        <div id="ui-latest-log" class="log-line">System initialized...</div>
        <div style="font-size: 20px;">⚡</div>
    </div>

    <script>
        setInterval(updateDashboard, 500);

        function updateDashboard() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    // Update Values
                    document.getElementById('ui-result-val').innerText = data.last_value;
                    document.getElementById('ui-sys-status').innerText = data.status;

                    // Update Status Badge Color
                    const badge = document.getElementById('ui-result-status');
                    badge.innerText = data.test_result;
                    badge.className = "status-badge"; // Reset
                    if(data.test_result === "PASS") badge.classList.add("status-pass");
                    else if(data.test_result === "FAIL") badge.classList.add("status-fail");
                    else badge.classList.add("status-ready");

                    // Update Markers
                    [1, 2, 3, 4].forEach(id => {
                        const el = document.getElementById('m' + id);
                        if (data.markers.includes(id)) el.classList.add('active');
                        else el.classList.remove('active');
                    });

                    // Update Log
                    if(data.console_log.length > 0) {
                        document.getElementById('ui-latest-log').innerText = data.console_log[0];
                    }

                    // Disable buttons if running
                    let isRunning = data.status.includes("RUNNING");
                    document.querySelectorAll('.btn-control').forEach(b => b.disabled = isRunning);
                });
        }

        function triggerTest(testType) {
            fetch('/trigger/' + testType, { method: 'POST' });
        }
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/status')
def get_status():
    return jsonify(system_state)

@app.route('/trigger/<test_type>', methods=['POST'])
def trigger(test_type):
    if "RUNNING" in system_state["status"]:
        return jsonify({"error": "System is busy"}), 400

    if test_type == "speed": threading.Thread(target=run_speed_test_logic).start()
    elif test_type == "path": threading.Thread(target=run_path_test_logic).start()
    elif test_type == "emergency": threading.Thread(target=run_emergency_test_logic).start()
    elif test_type == "zone": threading.Thread(target=run_zone_test_logic).start()
    
    return jsonify({"status": "started"})

if __name__ == '__main__':
    print("HMI Mockup Started on http://localhost:8080")
    app.run(host='0.0.0.0', port=8080, debug=True)