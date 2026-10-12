import serial
import csv
import threading
import time
import math
import tkinter as tk
from tkinter import messagebox 
import os

# Configuration
PORT = 'COM12' 
BAUD_RATE = 115200
FILE_NAME = "adrc.csv"

# Calibration constants 
CAL_SLOPE, CAL_OFFSET = 0.001845, -1.1914

# IMU (vertical axis = AZ; use the same axis and sign as ACC_SIGN in the ESP32 firmware)
ACC_SIGN = 1.0
ACC_SCALE_MM = 9806.65 / 16384.0   # mm/s^2 per LSB (+/-2 g range)
REST_SAMPLES = 25                  # rows averaged right after connecting to remove gravity (keep the platform still)
VEL_LEAK_TAU = 3.0                 # s, leak that stops the integrated IMU velocity from drifting away

class AirSpringADRCDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("Air Spring ADRC Control (ESO)")
        self.root.geometry("600x600")
        self.root.configure(bg="#121212")

        # CSV Setup
        if not os.path.exists(FILE_NAME):
            with open(FILE_NAME, "w", newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["PC_Time", "ESP_MS", "Target", "Height", "DutyCycle", "P_In_Raw", "P_Sp_Raw", "AX", "AY", "AZ", "GX", "GY", "GZ",
                                 "Session", "StepID", "StepTime_s", "StepStartHeight"])

        # Run / step separation
        self.session = time.strftime("%Y%m%d_%H%M%S")   # one value per program launch
        self.prev_target = None
        self.step_id = 0
        self.step_t0 = 0
        self.step_start_h = 0.0

        # IMU acceleration / velocity
        self.rest_buf = []
        self.az_rest = None
        self.vel = 0.0
        self.prev_esp_ms = None

        try:
            self.ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
        except:
            messagebox.showerror("Port Error", f"Cannot connect to {PORT}")
            self.root.destroy()
            return

        # --- UI ELEMENTS ---
        tk.Label(root, text="ADRC CONTROL DASHBOARD", font=("Helvetica", 16, "bold"), fg="#FF9900", bg="#121212").pack(pady=20)

        # Target Height
        target_frame = tk.Frame(root, bg="#121212")
        target_frame.pack(pady=20)
        tk.Label(target_frame, text="TARGET HEIGHT (mm):", fg="white", bg="#121212", font=("Arial", 10)).pack(side="left")
        self.target_ent = tk.Entry(target_frame, width=10, font=("Arial", 14), bg="#333", fg="white")
        self.target_ent.pack(side="left", padx=10)
        tk.Button(target_frame, text="SET", command=self.send_target, bg="#4CAF50", fg="white", width=8).pack(side="left")

        # Telemetry Labels 
        self.labels = {}
        data_pts = [
            ("Current Height", "mm"), 
            ("Target", "mm"), 
            ("Inlet Pressure", "Bar"), 
            ("Spring Pressure", "Bar"), 
            ("Acceleration", "m/s²"),
            ("Velocity (IMU)", "mm/s"),
            ("Status", "")
        ]
        
        for pt, unit in data_pts:
            f = tk.Frame(root, bg="#121212")
            f.pack(pady=5, fill="x", padx=100)
            tk.Label(f, text=f"{pt}:", fg="#888", bg="#121212", font=("Arial", 11), width=15, anchor="w").pack(side="left")
            lbl = tk.Label(f, text="--", fg="white", bg="#121212", font=("Arial", 12, "bold"))
            lbl.pack(side="left")
            if unit:
                tk.Label(f, text=f" {unit}", fg="#666", bg="#121212").pack(side="left")
            self.labels[pt] = lbl

        self.running = True
        threading.Thread(target=self.read_serial, daemon=True).start()

    def send_target(self):
        val = self.target_ent.get()
        if val: 
            self.ser.write(f"T{val}\n".encode())
            self.target_ent.delete(0, tk.END)

    def read_serial(self):
        while self.running:
            if self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode().strip()
                    d = line.split(',')
                    if len(d) >= 12:
                        esp_ms = int(d[0])
                        target = float(d[1])
                        height = float(d[2])

                        # Step segmentation on the ESP clock: a new step starts on the row where Target changes
                        if self.prev_target is None or target != self.prev_target:
                            if self.prev_target is not None:
                                self.step_id += 1
                            self.step_t0 = esp_ms
                            self.step_start_h = height
                        self.prev_target = target
                        step_time = (esp_ms - self.step_t0) / 1000.0

                        # Log to CSV
                        with open(FILE_NAME, "a", newline='') as f:
                            csv.writer(f).writerow([time.strftime("%H:%M:%S")] + d[:12] +
                                                   [self.session, self.step_id, f"{step_time:.3f}", self.step_start_h])
                        
                        # Pressure Calculations
                        p_in = (float(d[4]) * CAL_SLOPE) + CAL_OFFSET
                        p_sp = (float(d[5]) * CAL_SLOPE) + CAL_OFFSET

                        # IMU acceleration (gravity removed) and leaky-integrated velocity
                        az = float(d[8])
                        acc_mm = 0.0
                        if self.az_rest is None:
                            self.rest_buf.append(az)
                            if len(self.rest_buf) >= REST_SAMPLES:
                                self.az_rest = sum(self.rest_buf) / len(self.rest_buf)
                        else:
                            acc_mm = ACC_SIGN * (az - self.az_rest) * ACC_SCALE_MM
                            dt = (esp_ms - self.prev_esp_ms) / 1000.0
                            if 0 < dt < 0.5:
                                self.vel = self.vel * math.exp(-dt / VEL_LEAK_TAU) + acc_mm * dt
                        self.prev_esp_ms = esp_ms
                        
                        # Duty Cycle & Status Logic
                        dc = int(d[3])
                        status = "IDLE (DEADZONE)"
                        color = "#4CAF50"
                        if dc > 0: 
                            status = f"FILLING ({dc}%)"
                            color = "#2196F3"
                        elif dc < 0: 
                            status = f"DRAINING ({abs(dc)}%)"
                            color = "#F44336"

                        # Update UI
                        self.root.after(0, self.update_ui, d[2], d[1], p_in, p_sp, status, color, acc_mm / 1000.0, self.vel)
                except Exception as e: 
                    pass 
            time.sleep(0.01)

    def update_ui(self, h, t, pin, psp, st, col, acc, vel):
        self.labels["Current Height"].config(text=f"{h}")
        self.labels["Target"].config(text=f"{t}")
        self.labels["Inlet Pressure"].config(text=f"{pin:.3f}")
        self.labels["Spring Pressure"].config(text=f"{psp:.3f}")
        self.labels["Acceleration"].config(text=f"{acc:.2f}")
        self.labels["Velocity (IMU)"].config(text=f"{vel:.1f}")
        self.labels["Status"].config(text=st, fg=col)

    def on_close(self):
        self.running = False
        if hasattr(self, 'ser'):
            self.ser.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = AirSpringADRCDashboard(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
