import socket
import math
import tkinter as tk
from tkinter import ttk

# ================= CONFIGURATION =================
DEFAULT_IP = "192.168.1.100"
ESP_PORT = 3333
SEND_INTERVAL_MS = 50       # Send command every 50ms
JOYSTICK_SIZE = 180         # Size of the gray box
KNOB_RADIUS = 30            # Size of the red ball
CENTER = JOYSTICK_SIZE / 2
THRESHOLD = 10              # Min distance to start moving
MAX_DIST = CENTER - KNOB_RADIUS - 10 # Max joystick travel
DECEL_STEP_SPEED = 10       # Speed change per tick (0-100)
DECEL_STEP_DIR = 15         # Direction change per tick (0-360 degrees)
DECEL_STEP_ANG = 10         # Rotation change per tick (0-100)
# =================================================

# Setup UDP Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Set non-blocking or a short timeout so the GUI doesn't freeze if network is weird
sock.settimeout(0.01) 
target_ip = DEFAULT_IP

# Global Drive State
target_drive_state = { "speed": 0, "dir": 0, "ang": 0 }
current_drive_state = { "speed": 0.0, "dir": 0.0, "ang": 0.0 }

# Helper to normalize angle to 0-360
def normalize_angle(a):
    return a % 360

# Helper to get shortest angle difference
def shortest_angle_diff(target, current):
    diff = (target - current + 180) % 360 - 180
    return diff

# Helper to send UDP
def send_udp():
    global target_ip, target_drive_state, current_drive_state
    
    # 1. Speed Ramping
    diff_speed = target_drive_state["speed"] - current_drive_state["speed"]
    if abs(diff_speed) <= DECEL_STEP_SPEED:
        current_drive_state["speed"] = target_drive_state["speed"]
    else:
        if diff_speed > 0: current_drive_state["speed"] += DECEL_STEP_SPEED
        else: current_drive_state["speed"] -= DECEL_STEP_SPEED

    # 2. Direction Ramping (Smart Angle Interpolation)
    if target_drive_state["speed"] > 0 or current_drive_state["speed"] > 0:
        diff_dir = shortest_angle_diff(target_drive_state["dir"], current_drive_state["dir"])
        
        if abs(diff_dir) <= DECEL_STEP_DIR:
            current_drive_state["dir"] = target_drive_state["dir"]
        else:
            if diff_dir > 0:
                current_drive_state["dir"] += DECEL_STEP_DIR
            else:
                current_drive_state["dir"] -= DECEL_STEP_DIR
            
            # Normalize to 0-360
            current_drive_state["dir"] = normalize_angle(current_drive_state["dir"])

    # 3. Angular Ramping
    diff_ang = target_drive_state["ang"] - current_drive_state["ang"]
    if abs(diff_ang) <= DECEL_STEP_ANG:
        current_drive_state["ang"] = target_drive_state["ang"]
    else:
        if diff_ang > 0: current_drive_state["ang"] += DECEL_STEP_ANG
        else: current_drive_state["ang"] -= DECEL_STEP_ANG

    # Prepare integer values
    s = int(current_drive_state['speed'])
    d = int(current_drive_state['dir'])
    a = int(current_drive_state['ang'])

    # ---------------------------------------------------------
    # MODIFICATION: Print <s,d,a> regardless of connection
    # ---------------------------------------------------------
    print(f"<{s},{d},{a}>")
    # ---------------------------------------------------------

    cmd_str = f"drive:{s},{d},{a}"
    
    try:
        sock.sendto(cmd_str.encode(), (target_ip, ESP_PORT))
    except Exception:
        # Pass silently or print minimal error so we don't spam the console
        # while watching the <s,d,a> values
        pass 
    
    root.after(SEND_INTERVAL_MS, send_udp)

# ==========================================
#      JOYSTICK CLASS (Reusable Widget)
# ==========================================
class VirtualJoystick:
    def __init__(self, parent, label_text, on_change, mode="locomotion"):
        self.on_change = on_change
        self.mode = mode
        self.enabled_strafe = True 

        self.frame = tk.Frame(parent, bg="#222", padx=10, pady=10)
        self.frame.pack(side=tk.LEFT, padx=30)

        self.label = tk.Label(self.frame, text=label_text, fg="#00ff00", bg="#222", font=("Consolas", 14, "bold"))
        self.label.pack()

        self.canvas = tk.Canvas(self.frame, width=JOYSTICK_SIZE, height=JOYSTICK_SIZE, bg="#333", highlightthickness=0)
        self.canvas.pack(pady=10)

        self.canvas.create_line(CENTER, 20, CENTER, JOYSTICK_SIZE-20, fill="#555", width=2)
        self.canvas.create_line(20, CENTER, JOYSTICK_SIZE-20, CENTER, fill="#555", width=2)
        self.canvas.create_oval(30, 30, JOYSTICK_SIZE-30, JOYSTICK_SIZE-30, outline="#555")

        self.knob = self.canvas.create_oval(
            CENTER - KNOB_RADIUS, CENTER - KNOB_RADIUS,
            CENTER + KNOB_RADIUS, CENTER + KNOB_RADIUS,
            fill="#ff3333", outline="black", width=2
        )

        self.canvas.bind("<Button-1>", self.start_drag)
        self.canvas.bind("<B1-Motion>", self.drag)
        self.canvas.bind("<ButtonRelease-1>", self.release)
    
    def set_strafe(self, enabled):
        self.enabled_strafe = enabled
        
    def start_drag(self, event):
        self.drag(event)

    def drag(self, event):
        dx = event.x - CENTER
        dy = event.y - CENTER
        
        if self.mode == "locomotion" and not self.enabled_strafe:
            dx = 0

        dist = math.sqrt(dx*dx + dy*dy)

        if dist > MAX_DIST:
            scale = MAX_DIST / dist
            dx *= scale
            dy *= scale
            dist = MAX_DIST

        self.canvas.coords(self.knob, 
                           CENTER + dx - KNOB_RADIUS, CENTER + dy - KNOB_RADIUS,
                           CENTER + dx + KNOB_RADIUS, CENTER + dy + KNOB_RADIUS)

        intensity = 0
        if dist > THRESHOLD:
            intensity = ((dist - THRESHOLD) / (MAX_DIST - THRESHOLD)) * 100
            if intensity > 100: intensity = 100
            
        rad = math.atan2(dy, dx) 
        deg_math = math.degrees(rad)
        
        robot_deg = deg_math + 90
        if robot_deg < 0: robot_deg += 360
        if robot_deg >= 360: robot_deg -= 360
        
        x_val = 0
        if self.mode == "rotation":
             x_val = (dx / MAX_DIST) * 100
             if abs(x_val) < 10: x_val = 0
             intensity = x_val

        if self.mode == "locomotion":
            self.on_change(intensity, robot_deg)
        else:
            self.on_change(intensity, 0)

    def release(self, event):
        self.canvas.coords(self.knob, 
                           CENTER - KNOB_RADIUS, CENTER - KNOB_RADIUS,
                           CENTER + KNOB_RADIUS, CENTER + KNOB_RADIUS)
        self.on_change(0, 0)

# ==========================================
#      MAIN APPLICATION LOGIC
# ==========================================
root = tk.Tk()
root.title("UNIVERSAL ROBOT TELEOP")
root.configure(bg="#111")
root.geometry("650x550")

def on_move_change(speed, direction):
    global target_drive_state
    target_drive_state["speed"] = speed
    if speed > 0:
        target_drive_state["dir"] = direction

def on_rotate_change(val, _):
    global target_drive_state
    target_drive_state["ang"] = val

def update_ip(*args):
    global target_ip
    target_ip = ip_entry.get()
    status_label.config(text=f"Target: {target_ip}:{ESP_PORT} | Mode: {type_var.get()}")

def on_type_change(event):
    drive_type = type_var.get()
    if drive_type == "Differential":
        joy_move.set_strafe(False)
        joy_move.label.config(text="MOVE (No Strafe)")
    else:
        joy_move.set_strafe(True)
        joy_move.label.config(text="MOVE / STRAFE")
    update_ip()

# --- UI Layout ---
tk.Label(root, text="UNIVERSAL ROBOT TELEOP", fg="white", bg="#111", font=("Impact", 24)).pack(pady=(15, 10))

settings_frame = tk.Frame(root, bg="#222", pady=10)
settings_frame.pack(fill=tk.X, padx=20, pady=10)

tk.Label(settings_frame, text="Robot IP:", fg="#ccc", bg="#222", font=("Arial", 10)).pack(side=tk.LEFT, padx=(10, 5))
ip_entry = tk.Entry(settings_frame, width=15, font=("Arial", 10))
ip_entry.insert(0, DEFAULT_IP)
ip_entry.pack(side=tk.LEFT, padx=5)
ip_entry.bind("<Return>", update_ip)
ip_entry.bind("<FocusOut>", update_ip)

tk.Label(settings_frame, text="Drive Type:", fg="#ccc", bg="#222", font=("Arial", 10)).pack(side=tk.LEFT, padx=(20, 5))
type_var = tk.StringVar()
drive_types = ["Differential", "Omni", "Mecanum"]
type_combo = ttk.Combobox(settings_frame, textvariable=type_var, values=drive_types, state="readonly", width=12)
type_combo.current(2)
type_combo.pack(side=tk.LEFT, padx=5)
type_combo.bind("<<ComboboxSelected>>", on_type_change)

container = tk.Frame(root, bg="#111")
container.pack(pady=10)

joy_move = VirtualJoystick(container, "MOVE / STRAFE", on_move_change, mode="locomotion")
joy_rot = VirtualJoystick(container, "ROTATE", on_rotate_change, mode="rotation")

status_label = tk.Label(root, text=f"Target: {DEFAULT_IP}:{ESP_PORT} | Mode: Mecanum", fg="#666", bg="#111", font=("Consolas", 10))
status_label.pack(side=tk.BOTTOM, pady=10)

send_udp()
root.mainloop()