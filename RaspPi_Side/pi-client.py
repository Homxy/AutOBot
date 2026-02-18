#!/usr/bin/env python3
"""
Raspberry Pi 5 AI Robot Controller (Enhanced)
------------------------------------------------------
- Auto-detects the correct serial port.
- Sends <Speed, Direction, Angular> to Arduino.
- Supports Human/Line Following modes.
- ADDED: Human Box Size Distance Control.
- ADDED: Line Color Selection & Default Move Command.
"""

import sys
import time
import threading
import signal
import socket
import argparse
import subprocess
import os
from abc import ABC, abstractmethod

# Libraries
try:
    import cv2
    import numpy as np
    import gpiod
except ImportError:
    print("[ERR] Missing libraries. Run: pip install opencv-python-headless numpy gpiod")
    sys.exit(1)

# Optional YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("[WARN] Ultralytics not found. Human detection disabled.")

# ================= CONFIGURATION =================
class Config:
    # SERIAL_PORT is now determined dynamically
    BAUD_RATE = 115200
    
    # GPIO
    CHIP_PATH = "/dev/gpiochip0"
    LED_WIFI = 17
    LED_STATUS = 27
    WIFI_POLL_S = 2.0
    
    # Camera
    CAM_WIDTH = 320
    CAM_HEIGHT = 240
    
    # AI Model
    MODEL_PATH = "best.pt"
    
    # Logic Defaults
    BASE_SPEED = 60        
    TURN_GAIN_HUMAN = 0.7  
    TURN_GAIN_LINE = 0.7   
    LINE_THRESHOLD = 100   

    # --- NEW PARAMETERS ---
    # Human Following: Distance Control
    HUMAN_TARGET_SIZE = 30000 # Target area of bounding box
    HUMAN_BOX_DEADZONE = 5000 # +/- allowance
    
    # Line Following: Color & Default Move
    LINE_COLOR = "black"      # black, white, red, blue, green
    # Default Command if line NOT found: (Speed, Direction, Angular)
    LINE_DEFAULT_CMD = (0, 0, 0) 

# ================= HARDWARE WRAPPERS =================
class GPIOHandler:
    def __init__(self):
        self.chip = None
        self.leds = {}
        try:
            self.chip = gpiod.Chip(Config.CHIP_PATH)
            self._add_led("wifi", Config.LED_WIFI)
            self._add_led("status", Config.LED_STATUS)
        except Exception as e:
            print(f"[GPIO] Init Error: {e}")

    def _add_led(self, name, pin):
        l = self.chip.get_line(pin)
        l.request(consumer="bot", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        self.leds[name] = l

    def set(self, name, state):
        if name in self.leds: self.leds[name].set_value(1 if state else 0)
        
    def led_on(self, name): self.set(name, True)
    def led_off(self, name): self.set(name, False)

    def cleanup(self):
        for l in self.leds.values(): l.release()
        if self.chip: self.chip.close()

class LedOutAdapter:
    def __init__(self, gpio_handler, name):
        self.handler = gpio_handler
        self.name = name
    def on(self): self.handler.led_on(self.name)
    def off(self): self.handler.led_off(self.name)

# ================= WIFI LOGIC =================
def is_wifi_connected(interface="wlan0") -> bool:
    try:
        with open(f"/sys/class/net/{interface}/operstate") as f:
            if f.read().strip() == "up": return True
    except: pass
    try:
        out = subprocess.run(["ip", "-o", "-4", "addr", "show", "dev", interface], capture_output=True, text=True).stdout
        if "inet " in out: return True
    except: pass
    return False

def wifi_monitor(led1, stop_event):
    last = None
    while not stop_event.is_set():
        connected = is_wifi_connected()
        if connected != last:
            if connected:
                led1.on(); print("[Wi-Fi] Connected -> LED1 ON")
            else:
                led1.off(); print("[Wi-Fi] Disconnected -> LED1 OFF")
            last = connected
        stop_event.wait(Config.WIFI_POLL_S)

class SerialManager(threading.Thread):
    def __init__(self, callback):
        super().__init__(daemon=True)
        self.callback = callback
        self.running = True
        self.ser = None
        
        # Robust Port Finder
        possible_ports = ['/dev/serial0', '/dev/ttyAMA0', '/dev/ttyS0', '/dev/ttyUSB0', '/dev/ttyUSB1']
        for port in possible_ports:
            if os.path.exists(port):
                try:
                    import serial
                    self.ser = serial.Serial(port, Config.BAUD_RATE, timeout=0.1)
                    print(f"[SERIAL] Successfully connected to {port}")
                    break
                except Exception as e:
                    print(f"[SERIAL] Failed to open {port}: {e}")
        
        if self.ser is None:
            print("[SERIAL] Critical Error: No serial port found/opened.")

    def send_drive_command(self, speed, direction, angular):
        if self.ser and self.ser.is_open:
            s = max(0, min(100, int(speed)))
            d = max(0, min(360, int(direction)))
            a = max(-100, min(100, int(angular)))
            
            msg = f"<{s},{d},{a}>\n"
            try:
                self.ser.write(msg.encode('utf-8'))
            except: pass

    def run(self):
        if not self.ser: return
        while self.running:
            if self.ser.in_waiting > 0:
                try:
                    data = self.ser.read(self.ser.in_waiting).decode(errors='ignore')
                    if '\n' in data:
                        lines = data.split('\n')
                        for line in lines[:-1]:
                            if line.strip(): self.callback(line.strip())
                except: pass
            time.sleep(0.02)
            
    def close(self):
        self.running = False
        if self.ser: self.ser.close()

# ================= VISION LOGIC =================
class VisionProcessors:
    def __init__(self):
        # Counters
        self.human_count = 0
        self.line_count = 0
        # Hold variables: (Found, Error, Area)
        self.last_human_result = (False, 0, 0)
        self.last_line_result = (False, 0)
        
        # Color Ranges (HSV) for Line Mode
        self.colors = {
            "red": ([0, 120, 70], [10, 255, 255]), # Lower Red
            "red2": ([170, 120, 70], [180, 255, 255]), # Upper Red
            "blue": ([100, 150, 0], [140, 255, 255]),
            "green": ([40, 70, 70], [80, 255, 255])
        }

    def detect_human(self, frame, model):
        self.human_count += 1
        
        # 1. Sample and Hold
        if self.human_count % 5 != 0:
            return self.last_human_result

        # 2. Process frame
        if not model: 
            self.last_human_result = (False, 0, 0)
            return False, 0, 0
            
        results = model.track(frame, persist=True, verbose=False, classes=[0], conf=0.5)
        target_box = None
        max_area = 0
        
        for r in results:
            for box in r.boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                area = (xyxy[2]-xyxy[0]) * (xyxy[3]-xyxy[1])
                if area > max_area:
                    max_area = area
                    target_box = xyxy
                    
        if target_box is not None:
            h, w = frame.shape[:2]
            cx = (target_box[0] + target_box[2]) / 2
            error = (cx - (w / 2)) / (w / 2)
            
            # Update Hold: Return Found, Error, Area
            self.last_human_result = (True, error, max_area)
            return True, error, max_area
            
        self.last_human_result = (False, 0, 0)
        return False, 0, 0

    def detect_line(self, frame, threshold=100, color="black"):
        self.line_count += 1
        
        # 1. Sample and Hold
        if self.line_count % 5 != 0:
            return self.last_line_result

        # 2. Process frame
        h, w = frame.shape[:2]
        # Crop to bottom part for line following
        roi = frame[int(h*0.7):h, :] 
        
        mask = None
        
        # Grayscale approach for Black/White
        if color == "black":
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            # Invert binary: Black line becomes white (255)
            _, mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)
        elif color == "white":
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            # Standard binary: White line is white (255)
            _, mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
            
        # HSV approach for Colors
        elif color in self.colors or color == "red":
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            c_key = color if color in self.colors else "red"
            
            lower = np.array(self.colors[c_key][0], dtype="uint8")
            upper = np.array(self.colors[c_key][1], dtype="uint8")
            mask = cv2.inRange(hsv, lower, upper)
            
            # Special case for Red (wraps around 180)
            if c_key == "red":
                lower2 = np.array(self.colors["red2"][0], dtype="uint8")
                upper2 = np.array(self.colors["red2"][1], dtype="uint8")
                mask2 = cv2.inRange(hsv, lower2, upper2)
                mask = cv2.bitwise_or(mask, mask2)
        else:
            # Fallback to black behavior
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)

        # Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                # Adjust cx relative to ROI width
                error = (cx - (w / 2)) / (w / 2)
                
                self.last_line_result = (True, error)
                return True, error
        
        self.last_line_result = (False, 0)
        return False, 0
    
# ================= MODES =================
class BaseMode(ABC):
    @abstractmethod
    def run(self, frame) -> tuple: pass

class ModeIdle(BaseMode):
    def run(self, frame): return 0, 0, 0

class ModeHuman(BaseMode):
    def __init__(self, vision_processor):
        self.vision = vision_processor
        self.model = YOLO(Config.MODEL_PATH) if YOLO_AVAILABLE else None
    
    def run(self, frame):
        # NEW: Now accepts area
        found, error, area = self.vision.detect_human(frame, self.model)
        
        if found:
            # 1. Angular Control (Always active to face target)
            angular = error * Config.TURN_GAIN_HUMAN * 100 
            if abs(error) < 0.2: angular = 0 # Small deadband for turning
            
            # 2. Distance Control (Box Size)
            # If box is too small -> Move Forward (Dir 0)
            if area < (Config.HUMAN_TARGET_SIZE - Config.HUMAN_BOX_DEADZONE):
                speed = Config.BASE_SPEED
                direction = 0 # Forward
                
            # If box is too big -> Move Backward (Dir 180)
            elif area > (Config.HUMAN_TARGET_SIZE + Config.HUMAN_BOX_DEADZONE):
                speed = Config.BASE_SPEED
                direction = 180 # Backward
                
            # Inside Deadzone -> Stop Linear, Keep Rotating
            else:
                speed = 0
                direction = 0
                
            return speed, direction, angular
            
        return 0, 0, 0

class ModeLine(BaseMode):
    def __init__(self, vision_processor):
        self.vision = vision_processor

    def run(self, frame):
        # NEW: Pass color
        found, error = self.vision.detect_line(frame, Config.LINE_THRESHOLD, Config.LINE_COLOR)
        
        if found:
            # Standard Line Follow
            angular = error * Config.TURN_GAIN_LINE * 100
            return Config.BASE_SPEED, 0, angular
        else:
            # NEW: Default Command if line NOT found
            # Unpack default command tuple (Speed, Direction, Angular)
            ds, dd, da = Config.LINE_DEFAULT_CMD
            return ds, dd, da

class ModeDual(BaseMode):
    def __init__(self, human_mode, vision_processor):
        self.human_logic = human_mode
        self.vision = vision_processor

    def run(self, frame):
        # Check human
        h_found, _, _ = self.vision.detect_human(frame, self.human_logic.model)
        if h_found:
            print("[DUAL] Human Detected! Stop.")
            return 0, 0, 0
            
        # Check line
        found, error = self.vision.detect_line(frame, Config.LINE_THRESHOLD, Config.LINE_COLOR)
        if found:
            angular = error * Config.TURN_GAIN_LINE * 100
            return Config.BASE_SPEED, 0, angular
            
        # Default behavior if neither found (Stop in Dual mode generally safer)
        return 0, 0, 0

# ================= MAIN CONTROLLER =================
class RobotController:
    def __init__(self, drive_type):
        self.drive_type = drive_type
        self.gpio = GPIOHandler()
        self.comms = SerialManager(self.on_command_received)
        
        print(f"[SYS] Init Robot. Drive Type: {self.drive_type}")
        
        self.vision = VisionProcessors()

        self.human_mode = ModeHuman(self.vision)
        self.line_mode = ModeLine(self.vision)
        self.dual_mode = ModeDual(self.human_mode, self.vision)
        self.idle_mode = ModeIdle()
        
        self.current_mode = self.idle_mode
        self.running = True
        
        self._wifi_stop = threading.Event()
        self._setup_signals()
        self._start_wifi_monitor()

    def _start_wifi_monitor(self):
        led1_adapter = LedOutAdapter(self.gpio, "wifi")
        self.wifi_thread = threading.Thread(target=wifi_monitor, args=(led1_adapter, self._wifi_stop), daemon=True)
        self.wifi_thread.start()

    def _setup_signals(self):
        signal.signal(signal.SIGINT, self.handle_exit_signal)
        signal.signal(signal.SIGTERM, self.handle_exit_signal)

    def handle_exit_signal(self, signum, frame):
        self.running = False
        self._wifi_stop.set()

    def on_command_received(self, cmd):
        cmd = cmd.strip()
        print(f"[CMD] {cmd}")
        
        # Example: :M|spd=60|box=30000|col=red|def=40,180,0
        parts = cmd.split('|')
        
        for part in parts:
            if "=" in part:
                key, value = part.split('=', 1)
                key = key.strip().lower()
                val = value.strip()
                
                try:
                    if key == "spd":
                        Config.BASE_SPEED = int(val)
                        print(f"[CFG] Speed: {Config.BASE_SPEED}")
                    
                    elif key == "sen":
                        Config.TURN_GAIN_HUMAN = float(val)
                        Config.TURN_GAIN_LINE = float(val)
                        print(f"[CFG] Gain: {float(val)}")
                        
                    elif key == "thr":
                        Config.LINE_THRESHOLD = int(val)
                        print(f"[CFG] THRESHOLD: {int(val)}")

                    elif key == "type":
                        self.drive_type = val
                        print(f"[CFG] type: {self.drive_type}")
                        
                    elif key == "box":
                        Config.HUMAN_TARGET_SIZE = int(val)
                        print(f"[CFG] Target Box Size: {Config.HUMAN_TARGET_SIZE}")
                        
                    elif key == "dzone":
                        Config.HUMAN_BOX_DEADZONE = int(val)
                        print(f"[CFG] Box Deadzone: {Config.HUMAN_BOX_DEADZONE}")
                        
                    elif key == "col":
                        Config.LINE_COLOR = val.lower()
                        print(f"[CFG] Line Color: {Config.LINE_COLOR}")
                        
                    elif key == "def":
                        # Format: def=speed,direction,angular (e.g., 50,180,0)
                        d_vals = val.split(',')
                        if len(d_vals) == 3:
                            Config.LINE_DEFAULT_CMD = (int(d_vals[0]), int(d_vals[1]), int(d_vals[2]))
                            print(f"[CFG] Line Default Cmd: {Config.LINE_DEFAULT_CMD}")
                        
                except ValueError:
                    print(f"[ERR] Bad value in param: {part}")

        # Mode switching
        if ":M" in cmd:
            if "HM=1" in cmd and "LN=1" in cmd: 
                self.set_mode("DUAL")
            elif "HM=1" in cmd: 
                self.set_mode("HUMAN")
            elif "LN=1" in cmd: 
                self.set_mode("LINE")
            else: 
                self.set_mode("IDLE")

    def set_mode(self, mode_name):
        if mode_name == "HUMAN": self.current_mode = self.human_mode
        elif mode_name == "LINE": self.current_mode = self.line_mode
        elif mode_name == "DUAL": self.current_mode = self.dual_mode
        else: self.current_mode = self.idle_mode
        
        if mode_name != "IDLE": self.gpio.set("status", True)
        else: self.gpio.set("status", False)
        print(f"[SYS] Mode set to: {mode_name}")

    def _find_camera(self):
        print("[CAM] Scanning for cameras...")
        for i in range(10):
            cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    print(f"[CAM] Success! Found working camera at index {i}")
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, Config.CAM_WIDTH)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Config.CAM_HEIGHT)
                    return cap
                else:
                    cap.release()
        return None

    def start(self):
        self.comms.start()
        cap = self._find_camera()
        
        if cap is None:
            print("[SYS] Critical: Camera init failed. Exiting.")
            self.running = False
            self.comms.close()
            self.gpio.cleanup()
            return

        print("[SYS] Loop Started.")
        try:
            while self.running:
                ret, frame = cap.read()
                if not ret: 
                    print("[CAM] Frame read failed. Retrying...")
                    time.sleep(0.5)
                    cap.release()
                    cap = self._find_camera()
                    if cap is None: break
                    continue
                
                s, d, a = self.current_mode.run(frame)
                self.comms.send_drive_command(s, d, a)
        except Exception as e:
            print(f"[SYS] Error: {e}")
        finally:
            if cap: cap.release()
            self.comms.close()
            self.gpio.cleanup()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--type", default="diff", help="Drive type: diff, omni, mecanum")
    args = parser.parse_args()
    bot = RobotController(args.type)
    bot.start()
