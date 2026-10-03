import math
import os
import time
import threading
import traceback
import queue
import ctypes
from datetime import datetime

import numpy as np
import pandas as pd
# Make Windows report the real screen size before Tkinter creates the GUI.
# This prevents fullscreen offset and wrong scaling on 1920x1080 displays.
try:
    ctypes.windll.shcore.SetProcessDpiAwareness(2)  # Per-monitor DPI aware
except Exception:
    try:
        ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import customtkinter as ctk

from serial.tools import list_ports

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import serial

# Set CustomTkinter theme
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

class ControllingDCMotorApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Controlling DC Motor")
        
        # Fit the window to any resolution / Windows scaling (see _fit_window_to_screen).
        self._fit_window_to_screen(pref_w=1450, pref_h=860, min_w=1150, min_h=640)
        
        # Deep space MATLAB-like colors
        self.col_bg = "#06111f"
        self.col_panel = "#0b2135"
        self.col_border = "#164c78"
        self.col_accent = "#38bdf8"
        self.configure(fg_color=self.col_bg)

        # State Variables
        self.board = None
        self.is_connected = False
        self.is_running = False
        self.is_optimizing = False
        self.stop_flag = False
        self.enc_count = 0
        self.serial_lock = threading.Lock()
        
        # Hardware Pins
        self.R_PWM, self.L_PWM = 9, 10
        self.R_EN, self.L_EN = 8, 7
        self.ENC_A, self.ENC_B = 2, 3
        
        # Derived CPR
        self.CPR_MOTOR_4X = 64.0
        self.GEAR_RATIO = 131.25
        self.RPM_AT_OUTPUT = True
        # Restored 4X decoding with Highly Optimized Arduino ISR
        self.CPR_TOTAL = self.CPR_MOTOR_4X * self.GEAR_RATIO
        
        # ===== Feedforward Model =====
        # RPM_AT_MAX_PWM: actual max RPM your motor reaches at PWM=255.
        # Measure it: run at PWM=255 and read the steady-state RPM.
        # From observations the motor peaks ~140 RPM → set to 130 (conservative).
        self.PWM_AT_MAX_RPM = 255.0
        self.RPM_AT_MAX_PWM = 130.0   # ← KEY: actual max output RPM at full PWM
        self.PWM_DEADZONE = 35.0      # Minimum PWM to overcome static friction (490 Hz PWM)
        self.PWM_CORRECTION_LIMIT = 255.0  # Max PID correction ± on top of feedforward
        self.PWM_MIN = 0
        self.PWM_MAX = 255
        
        # ===== PID Control Tuning =====
        self.rpmAlpha    = 0.028391   # Filter for RPM
        self.dAlpha      = 0.050193   # Filter for Derivative
        self.I_LIMIT     = 255.0

        # ===== Shared real-hardware control behavior =====
        # These values are used in both Motor Control and Motor Optimizing trial tests.
        # Keep rpmAlpha and dAlpha fixed here; the optimizer only tunes Kp, Ki, and Kd.
        self.ENABLE_SOFT_START = False
        self.SOFT_START_SEC = 0.0
        self.ENABLE_PWM_SLEW_LIMIT = False
        self.PWM_SLEW_RATE = 0.0
        self.ENABLE_RPM_SLEW_LIMIT = False

        # Optimization trial cleanup.
        # These prevent leftover encoder counts and motor inertia from appearing as spikes
        # at the beginning of each PID candidate test.
        self.OPT_TRIAL_TIME_SEC = 8.0
        self.OPT_TRIAL_PREP_SEC = 0.0
        self.OPT_TRIAL_WARMUP_SEC = 0.0
        self.OPT_LIVE_PLOT_EVERY = 20
        
        # PID & Control State
        self.SAMPLE_MS = 10           # 30ms = 33Hz — Better resolution for low RPM motors
        self.ts_target = self.SAMPLE_MS / 1000.0
        self.target_rpm = 50.0
        self.last_pwm = 0
        
        self.integral = 0.0
        self.prev_err = 0.0
        self.d_filt = 0.0
        self.rpm_filt = 0.0
        self.prev_rpm_filt = 0.0
        self.first_sample = True
        
        self.t_list = []
        self.target_list = []
        self.rpm_list = []
        self.err_list = []
        self.pwm_list = []

        # Build UI
        self._build_ui()
        
        # Initial style setup
        self._style_ax(self.ax_ctrl, "Motor Response", "Time (s)", "RPM / PWM")
        self.canvas_ctrl.draw()
        self._style_ax(self.ax_opt, "Optimization Response", "Time (s)", "RPM / PWM")
        self.canvas_opt.draw()
        
        self.refresh_ports()
        
        # Fullscreen handling
        self.bind("<F11>", self.toggle_fullscreen)
        self.bind("<Escape>", self.exit_fullscreen)

        self.is_fullscreen = False
        self.normal_geometry = self.geometry()
        self._geometry_restore_job = None

        self.bind("<Configure>", self._remember_normal_geometry, add="+")

    # ------------------------------------------------------------
    # Window sizing (works for any resolution and Windows scaling)
    # ------------------------------------------------------------
    def _get_work_area_px(self):
        """Return (x, y, w, h) of the usable desktop area in physical pixels (taskbar excluded on Windows)."""
        try:
            class _Rect(ctypes.Structure):
                _fields_ = [("left", ctypes.c_long), ("top", ctypes.c_long),
                            ("right", ctypes.c_long), ("bottom", ctypes.c_long)]
            rect = _Rect()
            # SPI_GETWORKAREA = 0x0030
            if ctypes.windll.user32.SystemParametersInfoW(0x0030, 0, ctypes.byref(rect), 0):
                return rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top
        except Exception:
            pass
        return 0, 0, self.winfo_screenwidth(), self.winfo_screenheight()

    def _fit_window_to_screen(self, pref_w, pref_h, min_w, min_h):
        """
        Size and center the window so it always fits the usable screen area.

        CustomTkinter scales geometry() and minsize() by the Windows display scaling
        (e.g. 1200 x 720 becomes 1800 x 1080 physical pixels at 150 %), while the
        screen size is reported in physical pixels. All limits are therefore
        converted to CTk logical units before they are applied.
        """
        try:
            scale = ctk.ScalingTracker.get_window_scaling(self)
        except Exception:
            scale = 1.0
        area_x, area_y, area_w, area_h = self._get_work_area_px()

        # Reserve space for the window frame and title bar (physical pixels).
        frame_w = int(round(16 * scale))
        frame_h = int(round(40 * scale))
        avail_w = max(400, int((area_w - frame_w) / scale))
        avail_h = max(300, int((area_h - frame_h) / scale))

        width = min(pref_w, avail_w)
        height = min(pref_h, avail_h)
        self.minsize(min(min_w, width), min(min_h, height))

        # Position offsets are not scaled by CTk, so they stay in physical pixels.
        x = area_x + max(0, (area_w - int(round(width * scale)) - frame_w) // 2)
        y = area_y + max(0, (area_h - int(round(height * scale)) - frame_h) // 2)
        self.geometry(f"{width}x{height}+{x}+{y}")

    def toggle_fullscreen(self, event=None):
        if self.is_fullscreen:
            self.exit_fullscreen()
        else:
            self.enter_fullscreen()
        return "break"

    def enter_fullscreen(self, event=None):
        """
        Enter true fullscreen using native Tk fullscreen.
        """
        if self.is_fullscreen:
            return "break"

        self.update_idletasks()
        self.normal_geometry = self.geometry()
        self.is_fullscreen = True

        self.overrideredirect(False)
        self.attributes("-topmost", False)
        self.state("normal")
        self.update_idletasks()

        self.attributes("-fullscreen", True)

        self.after(80, self._refresh_layout_after_fullscreen)
        return "break"

    def exit_fullscreen(self, event=None):
        """
        Exit fullscreen and restore the previous normal window size.
        """
        if not self.is_fullscreen:
            return "break"

        self.is_fullscreen = False

        self.attributes("-fullscreen", False)
        self.overrideredirect(False)
        self.attributes("-topmost", False)
        self.state("normal")

        if self._geometry_restore_job is not None:
            try:
                self.after_cancel(self._geometry_restore_job)
            except Exception:
                pass

        self._geometry_restore_job = self.after(80, self._restore_normal_geometry)
        return "break"

    def _restore_normal_geometry(self):
        try:
            if self.normal_geometry:
                self.geometry(self.normal_geometry)

            self.update_idletasks()
            self.lift()
            self.focus_force()
        except Exception:
            pass
        finally:
            self._geometry_restore_job = None

    def _refresh_layout_after_fullscreen(self):
        try:
            self.update_idletasks()

            if hasattr(self, "canvas_ctrl"):
                self.canvas_ctrl.draw_idle()

            if hasattr(self, "canvas_opt"):
                self.canvas_opt.draw_idle()

            self.lift()
            self.focus_force()
        except Exception:
            pass

    def _remember_normal_geometry(self, event=None):
        """
        Save normal geometry only when not fullscreen.
        """
        if self.is_fullscreen:
            return

        try:
            geometry = self.geometry()
            if geometry and "x" in geometry and "+" in geometry:
                self.normal_geometry = geometry
        except Exception:
            pass

    def _build_ui(self):
        # Grid layout
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        # Header
        header = ctk.CTkFrame(self, fg_color="transparent")
        header.grid(row=0, column=0, sticky="ew", padx=20, pady=(15, 0))
        ctk.CTkLabel(header, text="Controlling DC Motor", font=("Segoe UI", 28, "bold"), text_color="white").pack(side="left")
        ctk.CTkLabel(header, text=" | Real-time PID Control & Black Hole Optimization via Telemetrix", font=("Segoe UI", 14), text_color="#8bb6d6").pack(side="left", padx=10, pady=(8,0))

        # Main Tabview
        self.tabview = ctk.CTkTabview(self, fg_color=self.col_panel, border_color=self.col_border, border_width=1, corner_radius=8)
        self.tabview.grid(row=1, column=0, sticky="nsew", padx=20, pady=10)
        
        self.tab_conn = self.tabview.add("Connection")
        self.tab_opt = self.tabview.add("Motor Optimizing")
        self.tab_ctrl = self.tabview.add("Motor Control")
        
        self._build_connection_tab()
        self._build_opt_tab()
        self._build_ctrl_tab()

    def _create_section(self, parent, title, row, col, rowspan=1, colspan=1, sticky="nsew", padx=5, pady=5):
        frame = ctk.CTkFrame(parent, fg_color="#06111f", border_width=1, border_color=self.col_border, corner_radius=5)
        frame.grid(row=row, column=col, rowspan=rowspan, columnspan=colspan, sticky=sticky, padx=padx, pady=pady)
        lbl = ctk.CTkLabel(frame, text=title, text_color=self.col_accent, font=("Segoe UI", 14, "bold"))
        lbl.pack(anchor="w", padx=15, pady=(10, 0))
        content = ctk.CTkFrame(frame, fg_color="transparent")
        content.pack(fill="both", expand=True, padx=15, pady=10)
        return content

    def _build_connection_tab(self):
        self.tab_conn.columnconfigure(0, weight=1)
        self.tab_conn.columnconfigure(1, weight=1)
        self.tab_conn.rowconfigure(1, weight=1)
        
        # 1. Config Panel
        cfg_sec = self._create_section(self.tab_conn, "Configuration Panel", 0, 0, colspan=2)
        cfg_sec.columnconfigure(1, weight=1)
        cfg_sec.columnconfigure(3, weight=1)
        
        ctk.CTkLabel(cfg_sec, text="Available Ports:", font=("Segoe UI", 12)).grid(row=0, column=0, sticky="w", pady=5)
        self.ports_var = ctk.StringVar(value="")
        self.cb_ports = ctk.CTkOptionMenu(cfg_sec, variable=self.ports_var, values=[""], width=150)
        self.cb_ports.grid(row=0, column=1, sticky="w", padx=10, pady=5)
        
        ctk.CTkLabel(cfg_sec, text="Boards:", font=("Segoe UI", 12)).grid(row=0, column=2, sticky="w", pady=5, padx=10)
        self.boards_var = ctk.StringVar(value="Uno")
        self.cb_boards = ctk.CTkOptionMenu(cfg_sec, variable=self.boards_var, values=["Uno", "Nano3", "Mega2560", "Leonardo", "Due"], width=150)
        self.cb_boards.grid(row=0, column=3, sticky="w", padx=10, pady=5)

        # 2. Connection Panel
        conn_sec = self._create_section(self.tab_conn, "Connection Panel", 1, 0, sticky="nsew")
        conn_sec.columnconfigure(0, weight=1); conn_sec.columnconfigure(1, weight=1)
        self.btn_connect = ctk.CTkButton(conn_sec, text="Connect", fg_color="#166534", hover_color="#22c55e", font=("Segoe UI", 12, "bold"), command=self.connect_arduino)
        self.btn_connect.grid(row=0, column=0, sticky="ew", padx=5)
        self.btn_disconnect = ctk.CTkButton(conn_sec, text="Disconnect", fg_color="#7f1d1d", hover_color="#ef4444", font=("Segoe UI", 12, "bold"), state="disabled", command=self.disconnect_arduino)
        self.btn_disconnect.grid(row=0, column=1, sticky="ew", padx=5)

        # 3. Testing Panel
        test_sec = self._create_section(self.tab_conn, "Testing Panel", 1, 1, sticky="nsew")
        test_sec.columnconfigure(0, weight=1); test_sec.columnconfigure(1, weight=1)
        self.btn_test = ctk.CTkButton(test_sec, text="Test (Blink D13)", fg_color="#075985", hover_color="#0284c7", state="disabled", command=self.test_connection)
        self.btn_test.grid(row=0, column=0, sticky="ew", padx=5)
        self.btn_refresh = ctk.CTkButton(test_sec, text="Refresh Ports", fg_color="#1f2937", hover_color="#334155", command=self.refresh_ports)
        self.btn_refresh.grid(row=0, column=1, sticky="ew", padx=5)
        
        # 4. Status Panel
        stat_sec = self._create_section(self.tab_conn, "Status", 2, 0, colspan=2, sticky="nsew")
        stat_sec.columnconfigure(0, weight=1)
        self.lbl_conn_status = ctk.CTkLabel(stat_sec, text="Status: Not connected", text_color="#ff4d5d", font=("Segoe UI", 14, "bold"))
        self.lbl_conn_status.pack(side="left", pady=10, padx=10)

    def _build_opt_tab(self):
        self.tab_opt.columnconfigure(0, weight=1)
        self.tab_opt.rowconfigure(1, weight=1)

        # 1. Optimizer Settings & PID Limits
        top_frame = ctk.CTkFrame(self.tab_opt, fg_color="transparent")
        top_frame.grid(row=0, column=0, sticky="ew")
        top_frame.columnconfigure(0, weight=1); top_frame.columnconfigure(1, weight=1)

        set_sec = self._create_section(top_frame, "Optimizer Settings", 0, 0)
        ctk.CTkLabel(set_sec, text="Number of Stars:").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.opt_stars = ctk.CTkEntry(set_sec, width=80); self.opt_stars.insert(0, "5")
        self.opt_stars.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        ctk.CTkLabel(set_sec, text="Iterations:").grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.opt_iters = ctk.CTkEntry(set_sec, width=80); self.opt_iters.insert(0, "5")
        self.opt_iters.grid(row=0, column=3, padx=5, pady=5, sticky="w")
        
        self.btn_opt_start = ctk.CTkButton(set_sec, text="Start", fg_color="#166534", hover_color="#22c55e", width=80, command=self.start_opt, state="disabled")
        self.btn_opt_start.grid(row=0, column=4, padx=10, pady=5)
        self.btn_opt_stop = ctk.CTkButton(set_sec, text="Stop", fg_color="#7f1d1d", hover_color="#ef4444", width=80, state="disabled", command=self.stop_opt)
        self.btn_opt_stop.grid(row=0, column=5, padx=5, pady=5)

        lim_sec = self._create_section(top_frame, "Set PID Limits", 0, 1)
        labels = ["Kp", "Ki", "Kd"]
        self.lim_vars = {}
        for i, l in enumerate(labels):
            ctk.CTkLabel(lim_sec, text=f"Max {l}:").grid(row=0, column=i*2, padx=5, pady=5, sticky="e")
            v_max = ctk.StringVar(value="10")
            ctk.CTkEntry(lim_sec, textvariable=v_max, width=60).grid(row=0, column=i*2+1, padx=5, pady=5, sticky="w")
            ctk.CTkLabel(lim_sec, text=f"Min {l}:").grid(row=1, column=i*2, padx=5, pady=5, sticky="e")
            v_min = ctk.StringVar(value="0")
            ctk.CTkEntry(lim_sec, textvariable=v_min, width=60).grid(row=1, column=i*2+1, padx=5, pady=5, sticky="w")
            self.lim_vars[l] = (v_min, v_max)

        # 2. Results & Graph
        res_sec = self._create_section(self.tab_opt, "Results", 1, 0)
        res_sec.columnconfigure(0, weight=1); res_sec.columnconfigure(1, weight=2)
        res_sec.rowconfigure(0, weight=1)

        # Table
        cols = ["Iteration", "Kp", "Ki", "Kd", "Cost"]
        tree_frame = ctk.CTkFrame(res_sec, fg_color="transparent")
        tree_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        tree_frame.columnconfigure(0, weight=1); tree_frame.rowconfigure(0, weight=1)
        self.opt_tree = ttk.Treeview(tree_frame, columns=cols, show="headings", height=8)
        for c in cols:
            self.opt_tree.heading(c, text=c)
            self.opt_tree.column(c, width=80, minwidth=60, anchor="center", stretch=True)
        self.opt_tree.grid(row=0, column=0, sticky="nsew")
        opt_sb = ttk.Scrollbar(tree_frame, orient="vertical", command=self.opt_tree.yview)
        opt_sb.grid(row=0, column=1, sticky="ns")
        self.opt_tree.configure(yscrollcommand=opt_sb.set)

        # Plot
        self.fig_opt = Figure(figsize=(5, 3), dpi=100, layout="constrained", facecolor=self.col_panel)
        self.ax_opt = self.fig_opt.add_subplot(111)
        self.canvas_opt = FigureCanvasTkAgg(self.fig_opt, master=res_sec)
        self.canvas_opt.get_tk_widget().grid(row=0, column=1, sticky="nsew")

    def _build_ctrl_tab(self):
        self.tab_ctrl.columnconfigure(0, weight=1)
        self.tab_ctrl.rowconfigure(0, weight=1)

        # 1. System Monitoring (Graph & Labels)
        mon_sec = self._create_section(self.tab_ctrl, "System Monitoring", 0, 0)
        mon_sec.columnconfigure(1, weight=1)
        mon_sec.rowconfigure(0, weight=1)

        lbl_frame = ctk.CTkFrame(mon_sec, fg_color="transparent")
        lbl_frame.grid(row=0, column=0, sticky="nw", padx=(0, 10))
        
        self.lbl_pwm = ctk.CTkLabel(lbl_frame, text="PWM: -- / 255", font=("Segoe UI", 16, "bold"), text_color=self.col_accent)
        self.lbl_pwm.pack(pady=10, anchor="w")
        self.lbl_err = ctk.CTkLabel(lbl_frame, text="Error: -- RPM", font=("Segoe UI", 16, "bold"), text_color="#ff4bd8")
        self.lbl_err.pack(pady=10, anchor="w")
        self.lbl_rpm = ctk.CTkLabel(lbl_frame, text="Actual: -- RPM", font=("Segoe UI", 16, "bold"), text_color="#59ff45")
        self.lbl_rpm.pack(pady=10, anchor="w")
        
        self.btn_save = ctk.CTkButton(lbl_frame, text="Save Data", fg_color="#075985", hover_color="#0284c7", state="normal", command=self.save_data)
        self.btn_save.pack(pady=30, anchor="w")

        self.fig_ctrl = Figure(figsize=(6, 4), dpi=100, layout="constrained", facecolor=self.col_panel)
        self.ax_ctrl = self.fig_ctrl.add_subplot(111)
        self.canvas_ctrl = FigureCanvasTkAgg(self.fig_ctrl, master=mon_sec)
        self.canvas_ctrl.get_tk_widget().grid(row=0, column=1, sticky="nsew")

        # 2. System Controlling
        ctrl_sec = self._create_section(self.tab_ctrl, "System Controlling", 1, 0, sticky="ew")
        
        self.sp_target = ctk.CTkEntry(ctrl_sec, width=60); self.sp_target.insert(0, "50")
        ctk.CTkLabel(ctrl_sec, text="RPM Target:").grid(row=0, column=0, padx=5, pady=10, sticky="e")
        self.sp_target.grid(row=0, column=1, padx=5, pady=10, sticky="w")

        ctk.CTkLabel(ctrl_sec, text="Direction:").grid(row=0, column=2, padx=5, pady=10, sticky="e")
        self.cb_dir = ctk.CTkOptionMenu(ctrl_sec, values=["Forward", "Reverse"], width=100)
        self.cb_dir.grid(row=0, column=3, padx=5, pady=10, sticky="w")

        ctk.CTkLabel(ctrl_sec, text="Kp:").grid(row=0, column=4, padx=5, pady=10, sticky="e")
        self.sp_kp = ctk.CTkEntry(ctrl_sec, width=60); self.sp_kp.insert(0, "7.9886")
        self.sp_kp.grid(row=0, column=5, padx=5, pady=10, sticky="w")
        
        ctk.CTkLabel(ctrl_sec, text="Ki:").grid(row=0, column=6, padx=5, pady=10, sticky="e")
        self.sp_ki = ctk.CTkEntry(ctrl_sec, width=60); self.sp_ki.insert(0, "1.7454")
        self.sp_ki.grid(row=0, column=7, padx=5, pady=10, sticky="w")
        
        ctk.CTkLabel(ctrl_sec, text="Kd:").grid(row=0, column=8, padx=5, pady=10, sticky="e")
        self.sp_kd = ctk.CTkEntry(ctrl_sec, width=60); self.sp_kd.insert(0, "0.5373")
        self.sp_kd.grid(row=0, column=9, padx=5, pady=10, sticky="w")

        self.btn_ctrl_run = ctk.CTkButton(ctrl_sec, text="Run", fg_color="#166534", hover_color="#22c55e", width=80, state="disabled", command=self.run_motor)
        self.btn_ctrl_run.grid(row=0, column=10, padx=15, pady=10)
        self.btn_ctrl_stop = ctk.CTkButton(ctrl_sec, text="Stop", fg_color="#7f1d1d", hover_color="#ef4444", width=80, state="disabled", command=self.stop_motor)
        self.btn_ctrl_stop.grid(row=0, column=11, padx=5, pady=10)

    # ==========================================
    # Logic & Telemetrix Communication
    # ==========================================
    def _style_ax(self, ax, title, xl, yl):
        ax.clear()
        ax.set_facecolor(self.col_panel)
        ax.set_title(title, color="white", fontsize=10, fontweight='bold')
        ax.set_xlabel(xl, color="#8bb6d6", fontsize=9)
        ax.set_ylabel(yl, color="#8bb6d6", fontsize=9)
        ax.tick_params(colors="#8bb6d6", labelsize=8)
        for spine in ax.spines.values(): spine.set_color(self.col_border)
        ax.grid(True, color=self.col_border, linestyle="--", alpha=0.5)


    def set_status(self, text, color="#59ff45"):
        self.lbl_conn_status.configure(text=f"Status: {text}", text_color=color)
        self.update_idletasks()

    def refresh_ports(self):
        self.set_status("Scanning ports...", "#ffd400")
        ports = [port.device for port in list_ports.comports()]
        if ports:
            self.cb_ports.configure(values=ports)
            if self.ports_var.get() not in ports:
                self.ports_var.set(ports[0])
            self.set_status(f"Found {len(ports)} ports.", "#38bdf8")
        else:
            self.cb_ports.configure(values=[""])
            self.ports_var.set("")
            self.set_status("No serial ports found.", "#ff4d5d")

    def _encoder_callback(self, data):
        # data[0] = pin type, data[1] = count, data[2] = time
        if data[1] is not None:
            self.enc_count = data[1]

    def connect_arduino(self):
        port = self.ports_var.get()
        if not port:
            messagebox.showerror("Error", "Select a port first.")
            return
        
        self.set_status(f"Connecting to {port}...", "#ffd400")
        self.btn_connect.configure(state="disabled")
        
        def connect_thread():
            try:
                self.board = serial.Serial(port, 115200, timeout=1)
                time.sleep(2) # Wait for Arduino reset
                
                # Test connection
                self.board.write(b"PING\n")
                response = self.board.readline().decode().strip()
                if response != "PONG":
                    raise Exception("Arduino did not respond with PONG. Make sure you uploaded Arduino_DC_Motor.ino")
                
                self.board.write(b"RESET\n")
                self.board.readline()
                
                self.is_connected = True
                self.enc_count = 0
                self.after(0, self._on_connected_success)
            except Exception as e:
                self.is_connected = False
                self.board = None
                err_msg = str(e)
                self.after(0, lambda: self._on_connected_fail(err_msg))
        
        threading.Thread(target=connect_thread, daemon=True).start()

    def _on_connected_success(self):
        self.set_status("Connected", "#59ff45")
        self.btn_disconnect.configure(state="normal")
        self.btn_test.configure(state="normal")
        self.btn_ctrl_run.configure(state="normal")
        self.btn_opt_start.configure(state="normal")

    def _on_connected_fail(self, err):
        self.set_status("Connection failed", "#ff4d5d")
        self.btn_connect.configure(state="normal")
        messagebox.showerror("Connection Error", f"Failed to connect.\nMake sure you uploaded Arduino_DC_Motor.ino!\nError: {err}")

    def disconnect_arduino(self):
        self.stop_motor()
        if self.board:
            try:
                self.board.write(b"PWM:1,0\n")
                self.board.close()
            except: pass
            self.board = None
        self.is_connected = False
        self.set_status("Disconnected", "#ff4d5d")
        self.btn_connect.configure(state="normal")
        self.btn_disconnect.configure(state="disabled")
        self.btn_test.configure(state="disabled")
        self.btn_ctrl_run.configure(state="disabled")
        self.btn_opt_start.configure(state="disabled")

    def test_connection(self):
        if not self.is_connected or not self.board: return
        self.set_status("Testing...", "#38bdf8")
        def test_cmd():
            try:
                self.board.write(b"PING\n")
                resp = self.board.readline().decode().strip()
                if resp == "PONG":
                    self.after(0, lambda: self.set_status("Test OK (PONG)", "#59ff45"))
                else:
                    self.after(0, lambda: self.set_status("Test Failed", "#ff4d5d"))
            except Exception as e:
                self.after(0, lambda: self.set_status("Test Error", "#ff4d5d"))
        threading.Thread(target=test_cmd, daemon=True).start()

    def run_motor(self):
        if not self.is_connected or self.is_running: return
        self.is_running = True
        self.btn_ctrl_run.configure(state="disabled")
        self.btn_ctrl_stop.configure(state="normal")
        self.btn_save.configure(state="disabled")
        
        self.control_thread = threading.Thread(target=self._pid_loop, daemon=True)
        self.control_thread.start()
        
        self.after(200, self._update_plot)

    def stop_motor(self):
        if not self.is_running: return
        self.is_running = False
        self.btn_ctrl_run.configure(state="normal")
        self.btn_ctrl_stop.configure(state="disabled")
        self.btn_save.configure(state="normal")
        
        self._stop_pwm_output()

    def _read_encoder(self):
        """Read encoder count from Arduino. No buffer flush — avoids timing jitter."""
        if not self.board:
            return self.enc_count
        try:
            with self.serial_lock:
                self.board.write(b"GET\n")
                resp = self.board.readline()
            if not resp:
                return self.enc_count
            resp_str = resp.decode(errors="ignore").strip()
            if resp_str.lstrip('-').isdigit():
                return int(resp_str)
            return self.enc_count
        except Exception:
            return self.enc_count

    def _write_pwm(self, direction, pwm_cmd):
        """Send PWM command safely to Arduino."""
        if not self.board:
            return
        try:
            dir_val = 1 if direction == "Forward" else 0
            pwm_cmd = int(max(self.PWM_MIN, min(self.PWM_MAX, pwm_cmd)))
            with self.serial_lock:
                self.board.write(f"PWM:{dir_val},{pwm_cmd}\n".encode())
        except Exception:
            pass

    def _stop_pwm_output(self):
        """Stop motor output safely."""
        if not self.board:
            return
        try:
            with self.serial_lock:
                self.board.write(b"PWM:1,0\n")
        except Exception:
            pass

    def _reset_encoder_count(self):
        """Reset encoder count on Arduino and clear the local cached count."""
        if not self.board:
            self.enc_count = 0
            return
        try:
            with self.serial_lock:
                self.board.write(b"RESET\n")
                self.board.readline()
            self.enc_count = 0
        except Exception:
            self.enc_count = 0

    def _prepare_motor_for_trial(self, wait_sec=None):
        """Make every optimization trial start from the same clean hardware state."""
        if wait_sec is None:
            wait_sec = self.OPT_TRIAL_PREP_SEC
        self._stop_pwm_output()
        time.sleep(max(0.0, wait_sec))
        self._reset_encoder_count()
        time.sleep(0.05)

    def _init_control_state(self):
        """Create a clean control state for one control run or one optimization trial."""
        return {
            "integral": 0.0,
            "prev_err": 0.0,
            "d_filt": 0.0,
            "rpm_filt": 0.0,
            "prev_rpm_filt": 0.0,
            "first_sample": True,
            "last_pwm": 0,
        }

    def _compute_pid_step(self, state, raw_rpm, target, actual_dt, elapsed_time, Kp, Ki, Kd):
        """
        Shared real-hardware PID step.
        Used by both Motor Control and Motor Optimizing so both panels behave the same.
        """
        dt = max(actual_dt, 0.005)
        target = max(0.0, float(target))

        # Convert direction/noise to a positive speed value.
        raw_rpm = abs(raw_rpm)

        # Clamp impossible encoder spikes before filtering.
        # This is applied in both Control and Optimizing panels.
        spike_limit = max(target * 3.0, state["rpm_filt"] * 3.0, 300.0)
        raw_rpm = min(raw_rpm, spike_limit)

        # Slew-limit the measured RPM before the low-pass filter.
        # A single bad encoder/serial sample should not create a visible RPM spike.
        if not state["first_sample"] and self.ENABLE_RPM_SLEW_LIMIT:
            max_rpm_step = max(0.35 * max(target, 1.0), 8.0)
            delta_rpm = raw_rpm - state["rpm_filt"]
            if abs(delta_rpm) > max_rpm_step:
                raw_rpm = state["rpm_filt"] + math.copysign(max_rpm_step, delta_rpm)

        # RPM low-pass filter.
        if state["first_sample"]:
            state["rpm_filt"] = raw_rpm
            state["prev_rpm_filt"] = raw_rpm
            state["integral"] = 0.0
            state["d_filt"] = 0.0
            state["first_sample"] = False
        else:
            state["rpm_filt"] = (
                self.rpmAlpha * raw_rpm +
                (1.0 - self.rpmAlpha) * state["rpm_filt"]
            )

        rpm_filt = state["rpm_filt"]

        if target < 0.5:
            state["integral"] = 0.0
            state["prev_err"] = 0.0
            state["d_filt"] = 0.0
            state["prev_rpm_filt"] = 0.0
            state["last_pwm"] = 0
            return 0, rpm_filt, target - rpm_filt

        err_now = target - rpm_filt

        # Feedforward.
        pwm_ff = self.PWM_DEADZONE + (target / max(self.RPM_AT_MAX_PWM, 1e-9)) * (
            self.PWM_AT_MAX_RPM - self.PWM_DEADZONE
        )
        pwm_ff = max(self.PWM_MIN, min(self.PWM_MAX, pwm_ff))

        # Derivative on measurement with dAlpha filtering.
        d_raw = (rpm_filt - state["prev_rpm_filt"]) / dt
        state["d_filt"] = self.dAlpha * d_raw + (1.0 - self.dAlpha) * state["d_filt"]

        # Integral with anti-windup clamp.
        state["integral"] += err_now * dt
        state["integral"] = max(-self.I_LIMIT, min(self.I_LIMIT, state["integral"]))

        correction = Kp * err_now + Ki * state["integral"] - Kd * state["d_filt"]
        correction = max(-self.PWM_CORRECTION_LIMIT, min(self.PWM_CORRECTION_LIMIT, correction))

        pwm_requested = max(self.PWM_MIN, min(self.PWM_MAX, pwm_ff + correction))

        # Soft-start cap.
        if self.ENABLE_SOFT_START and self.SOFT_START_SEC > 0:
            soft_ratio = max(0.0, min(1.0, elapsed_time / self.SOFT_START_SEC))
            soft_cap = self.PWM_MIN + soft_ratio * (self.PWM_MAX - self.PWM_MIN)
            pwm_requested = min(pwm_requested, soft_cap)

        # PWM slew-rate limiter.
        if self.ENABLE_PWM_SLEW_LIMIT and self.PWM_SLEW_RATE > 0:
            max_step = self.PWM_SLEW_RATE * dt
            prev_pwm = float(state["last_pwm"])
            pwm_requested = max(prev_pwm - max_step, min(prev_pwm + max_step, pwm_requested))

        pwm_cmd = int(round(max(self.PWM_MIN, min(self.PWM_MAX, pwm_requested))))

        state["prev_err"] = err_now
        state["prev_rpm_filt"] = rpm_filt
        state["last_pwm"] = pwm_cmd

        return pwm_cmd, rpm_filt, err_now


    def _pid_loop(self):
        state = self._init_control_state()

        # Keep class variables updated for UI/debugging.
        self.integral = 0.0
        self.prev_err = 0.0
        self.d_filt = 0.0
        self.rpm_filt = 0.0
        self.prev_rpm_filt = 0.0
        self.first_sample = True
        self.last_pwm = 0

        last_enc = self._read_encoder()
        self.enc_count = last_enc

        start_time = time.perf_counter()
        prev_time = start_time

        try:
            Kp = float(self.sp_kp.get())
            Ki = float(self.sp_ki.get())
            Kd = float(self.sp_kd.get())
        except ValueError:
            Kp, Ki, Kd = 0.0, 0.0, 0.0

        self.t_list.clear()
        self.target_list.clear()
        self.rpm_list.clear()
        self.err_list.clear()
        self.pwm_list.clear()

        while self.is_running:
            loop_start = time.perf_counter()

            current_enc = self._read_encoder()
            self.enc_count = current_enc
            delta_enc = current_enc - last_enc
            last_enc = current_enc

            current_time = time.perf_counter()
            actual_dt = max(current_time - prev_time, 0.005)
            prev_time = current_time

            raw_rpm = (delta_enc * 60.0) / (self.CPR_TOTAL * actual_dt)

            try:
                target = float(self.sp_target.get())
            except ValueError:
                target = 0.0

            elapsed_time = loop_start - start_time
            pwm_cmd, rpm_filt, err_now = self._compute_pid_step(
                state, raw_rpm, target, actual_dt, elapsed_time, Kp, Ki, Kd
            )

            self.integral = state["integral"]
            self.prev_err = state["prev_err"]
            self.d_filt = state["d_filt"]
            self.rpm_filt = state["rpm_filt"]
            self.prev_rpm_filt = state["prev_rpm_filt"]
            self.first_sample = state["first_sample"]
            self.last_pwm = state["last_pwm"]

            direction = self.cb_dir.get()
            self._write_pwm(direction, pwm_cmd)

            self.t_list.append(elapsed_time)
            self.target_list.append(target)
            self.rpm_list.append(rpm_filt)
            self.err_list.append(err_now)
            self.pwm_list.append(pwm_cmd)

            self.after(0, lambda p=pwm_cmd, r=rpm_filt, e=err_now: self._update_labels(p, r, e))

            elapsed = time.perf_counter() - loop_start
            sleep_time = self.ts_target - elapsed
            if sleep_time > 0.002:
                time.sleep(sleep_time - 0.002)
            while time.perf_counter() - loop_start < self.ts_target:
                pass

        self._stop_pwm_output()

    def _update_labels(self, pwm, rpm, err):
        self.lbl_pwm.configure(text=f"PWM: {pwm} / 255")
        self.lbl_rpm.configure(text=f"Actual: {rpm:.2f} RPM")
        self.lbl_err.configure(text=f"Error: {err:.2f} RPM")

    def _update_plot(self):
        if not self.is_running:
            return
            
        if self.t_list:
            if not hasattr(self, 'line_target') or self.line_target not in self.ax_ctrl.lines:
                self.ax_ctrl.clear()
                self._style_ax(self.ax_ctrl, "Motor Response", "Time (s)", "RPM / PWM")
                self.line_target, = self.ax_ctrl.plot([], [], color="#ffb703", label="Target", linewidth=2)
                self.line_rpm, = self.ax_ctrl.plot([], [], color="#38bdf8", label="Actual RPM", linewidth=2)
                self.line_pwm, = self.ax_ctrl.plot([], [], color="#ff4bd8", label="PWM", linewidth=1.5, linestyle="--")
                self.ax_ctrl.legend(loc="upper right", facecolor=self.col_panel, edgecolor=self.col_border, labelcolor="white")
            
            t = np.array(self.t_list)
            target = np.array(self.target_list)
            rpm = np.array(self.rpm_list)
            pwm = np.array(self.pwm_list)
            
            n = len(t)
            if n > 1500:
                step = n // 750
                t = t[::step]
                target = target[::step]
                rpm = rpm[::step]
                pwm = pwm[::step]
                
            self.line_target.set_data(t, target)
            self.line_rpm.set_data(t, rpm)
            self.line_pwm.set_data(t, pwm)
            
            if len(t) > 0:
                self.ax_ctrl.set_xlim(0, max(2.0, t[-1]))
                y_max = max(260, np.max(rpm) * 1.2 if len(rpm) > 0 else 0, np.max(target) * 1.2 if len(target) > 0 else 0)
                y_min = min(0, np.min(rpm) * 1.2 if len(rpm) > 0 else 0)
                self.ax_ctrl.set_ylim(y_min, y_max)
                
            self.canvas_ctrl.draw_idle()
            
        self.after(200, self._update_plot)

    def save_data(self):
        if not self.t_list:
            messagebox.showinfo("Info", "No data to save.")
            return
            
        df = pd.DataFrame({"Time(s)": self.t_list, "Target RPM": self.target_list, "Actual RPM": self.rpm_list, "Error": self.err_list, "PWM": self.pwm_list})
        
        file_path = filedialog.asksaveasfilename(defaultextension=".xlsx", filetypes=[("Excel Files", "*.xlsx")])
        if file_path:
            import tempfile
            with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as tmp:
                img_path = tmp.name
            
            try:
                self.fig_ctrl.savefig(img_path)
                df.to_excel(file_path, index=False)
                
                try:
                    import openpyxl
                    from openpyxl.drawing.image import Image as xlImage
                    from openpyxl.chart import LineChart, Reference
                    
                    wb = openpyxl.load_workbook(file_path)
                    ws = wb.active
                    img = xlImage(img_path)
                    ws.add_image(img, "G2")
                    
                    # Create native Excel chart
                    chart = LineChart()
                    chart.title = "Motor Response (Target vs Actual vs PWM)"
                    chart.style = 13
                    chart.y_axis.title = 'RPM / PWM'
                    chart.x_axis.title = 'Time (s)'
                    chart.width = 20
                    chart.height = 10
                    
                    last_row = ws.max_row
                    cats = Reference(ws, min_col=1, min_row=2, max_row=last_row)
                    
                    for col in [2, 3, 5]:  # Target RPM, Actual RPM, PWM
                        data = Reference(ws, min_col=col, min_row=1, max_row=last_row)
                        chart.add_data(data, titles_from_data=True)
                        
                    chart.set_categories(cats)
                    
                    if len(chart.series) >= 1:
                        chart.series[0].graphicalProperties.line.solidFill = "FFB703"
                        chart.series[0].graphicalProperties.line.width = 20000
                    if len(chart.series) >= 2:
                        chart.series[1].graphicalProperties.line.solidFill = "38BDF8"
                        chart.series[1].graphicalProperties.line.width = 25000
                    if len(chart.series) >= 3:
                        chart.series[2].graphicalProperties.line.solidFill = "FF4BD8"
                        chart.series[2].graphicalProperties.line.dashStyle = "dash"
                        chart.series[2].graphicalProperties.line.width = 15000
                        
                    ws.add_chart(chart, "G45")
                    
                    wb.save(file_path)
                except Exception as e:
                    print(f"Failed to embed images or chart in Excel: {e}")
                    
                messagebox.showinfo("Success", f"Data and graph saved successfully to:\n{file_path}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save data: {e}")
            finally:
                try:
                    os.remove(img_path)
                except Exception:
                    pass

    # ==========================================
    # Optimization (BHA Live Hardware Testing)
    # ==========================================
    def start_opt(self):
        if not self.is_connected or self.is_optimizing: return
        
        # If motor is currently running in Control Tab, stop it first.
        if self.is_running:
            self.stop_motor()
            time.sleep(0.5)
            
        self.is_optimizing = True
        self.stop_flag = False
        self.btn_opt_start.configure(state="disabled")
        self.btn_opt_stop.configure(state="normal")
        self.btn_ctrl_run.configure(state="disabled")
        
        # Clear table
        for item in self.opt_tree.get_children(): self.opt_tree.delete(item)
        
        # Start BHA Thread
        threading.Thread(target=self._bha_worker, daemon=True).start()

    def stop_opt(self):
        self.stop_flag = True

    def _bha_worker(self):
        try:
            stars_num = int(self.opt_stars.get())
            max_iters = int(self.opt_iters.get())
            
            # Limits
            lims = {}
            for l in ["Kp", "Ki", "Kd"]:
                vmin, vmax = self.lim_vars[l]
                lims[l] = (float(vmin.get()), float(vmax.get()))
                
            bounds = np.array([
                [lims["Kp"][0], lims["Kp"][1]],
                [lims["Ki"][0], lims["Ki"][1]],
                [lims["Kd"][0], lims["Kd"][1]]
            ])
            
            # 1. Initialize Stars
            stars = np.random.uniform(bounds[:, 0], bounds[:, 1], (stars_num, 3))
            costs = np.zeros(stars_num)
            
            # Evaluate initial
            for i in range(stars_num):
                if self.stop_flag: break
                costs[i] = self._evaluate_pid(stars[i])
                
            best_idx = np.argmin(costs)
            bh_pos = stars[best_idx].copy()
            bh_cost = costs[best_idx]
            
            self.after(0, lambda: self._add_opt_result(0, bh_pos, bh_cost))
            
            # BHA Loop
            for iteration in range(1, max_iters + 1):
                if self.stop_flag: break
                
                # Move stars towards black hole
                for i in range(stars_num):
                    if i == best_idx: continue
                    r = np.random.rand(3)
                    stars[i] = stars[i] + r * (bh_pos - stars[i])
                    # Bound
                    stars[i] = np.clip(stars[i], bounds[:, 0], bounds[:, 1])
                    
                # Evaluate
                for i in range(stars_num):
                    if self.stop_flag: break
                    if i == best_idx: continue
                    c = self._evaluate_pid(stars[i])
                    costs[i] = c
                    if c < bh_cost:
                        bh_cost = c
                        bh_pos = stars[i].copy()
                        best_idx = i
                        
                # Event Horizon
                R = bh_cost / np.sum(costs)
                for i in range(stars_num):
                    if i == best_idx: continue
                    dist = np.linalg.norm(stars[i] - bh_pos)
                    if dist < R:
                        stars[i] = np.random.uniform(bounds[:, 0], bounds[:, 1], 3)
                        costs[i] = self._evaluate_pid(stars[i])
                        
                # Update best again
                best_idx = np.argmin(costs)
                bh_pos = stars[best_idx].copy()
                bh_cost = costs[best_idx]
                
                self.after(0, lambda it=iteration, p=bh_pos, c=bh_cost: self._add_opt_result(it, p, c))
                
            # Apply best to Control tab
            self.after(0, lambda p=bh_pos: self._apply_best_pid(p))
            
        except Exception as e:
            traceback.print_exc()
            msg = str(e)
            self.after(0, lambda m=msg: messagebox.showerror("Optimization Error", m))
        finally:
            self.is_optimizing = False
            self.after(0, self._on_opt_finished)
            
    def _add_opt_result(self, it, pid, cost):
        self.opt_tree.insert("", "end", values=(it, f"{pid[0]:.4f}", f"{pid[1]:.4f}", f"{pid[2]:.4f}", f"{cost:.4f}"))
        self.opt_tree.yview_moveto(1)

    def _apply_best_pid(self, pid):
        self.sp_kp.delete(0, 'end'); self.sp_kp.insert(0, f"{pid[0]:.4f}")
        self.sp_ki.delete(0, 'end'); self.sp_ki.insert(0, f"{pid[1]:.4f}")
        self.sp_kd.delete(0, 'end'); self.sp_kd.insert(0, f"{pid[2]:.4f}")
        messagebox.showinfo("Optimization Finished", f"Best PID found:\nKp: {pid[0]:.4f}\nKi: {pid[1]:.4f}\nKd: {pid[2]:.4f}")

    def _on_opt_finished(self):
        self.btn_opt_start.configure(state="normal")
        self.btn_opt_stop.configure(state="disabled")
        if self.is_connected:
            self.btn_ctrl_run.configure(state="normal")

    def _evaluate_pid(self, pid):
        """
        Evaluate one PID candidate on the real motor.
        This uses the exact same sampling, filtering, feedforward, PID, and PWM command path
        used by the Motor Control tab.

        For this test version:
        - soft start is disabled
        - PWM slew-rate limiting is disabled
        - RPM slew-rate limiting is disabled
        - rpmAlpha and dAlpha are fixed class constants and are not optimized
        - no warmup samples are removed from the plotted/cost data
        """
        Kp, Ki, Kd = [float(x) for x in pid]

        try:
            target = float(self.sp_target.get() if self.sp_target.get() else 50)
        except ValueError:
            target = 50.0

        direction = self.cb_dir.get()
        state = self._init_control_state()

        # Start each candidate from a stopped output, but do not add artificial soft-start or slew limits.
        self._stop_pwm_output()
        if self.OPT_TRIAL_PREP_SEC > 0:
            time.sleep(self.OPT_TRIAL_PREP_SEC)
        self._reset_encoder_count()

        last_enc = self._read_encoder()
        self.enc_count = last_enc

        t_data = []
        rpm_data = []
        pwm_data = []

        start_time = time.perf_counter()
        prev_time = start_time
        max_time = self.OPT_TRIAL_TIME_SEC

        while time.perf_counter() - start_time < max_time and not self.stop_flag:
            loop_start = time.perf_counter()

            cur_enc = self._read_encoder()
            self.enc_count = cur_enc
            delta = cur_enc - last_enc
            last_enc = cur_enc

            current_time = time.perf_counter()
            actual_dt = max(current_time - prev_time, 0.005)
            prev_time = current_time

            raw_rpm = (delta * 60.0) / (self.CPR_TOTAL * actual_dt)
            elapsed_time = loop_start - start_time

            pwm_cmd, rpm_filt, err_now = self._compute_pid_step(
                state, raw_rpm, target, actual_dt, elapsed_time, Kp, Ki, Kd
            )

            self._write_pwm(direction, pwm_cmd)

            # Same as the Motor Control tab: store and plot from the first real sample.
            t_data.append(elapsed_time)
            rpm_data.append(rpm_filt)
            pwm_data.append(pwm_cmd)

            if len(t_data) % self.OPT_LIVE_PLOT_EVERY == 0:
                t_copy = np.array(t_data, dtype=float)
                rpm_copy = np.array(rpm_data, dtype=float)
                pwm_copy = np.array(pwm_data, dtype=float)
                self.after(0, lambda t=t_copy, r=rpm_copy, p=pwm_copy: self._update_opt_plot_live(t, r, p, target))

            elapsed = time.perf_counter() - loop_start
            sleep_time = self.ts_target - elapsed
            if sleep_time > 0.002:
                time.sleep(sleep_time - 0.002)
            while time.perf_counter() - loop_start < self.ts_target:
                pass

        self._stop_pwm_output()

        if self.stop_flag:
            return 999999.0

        if len(t_data) < 8:
            return 999999.0

        t = np.array(t_data, dtype=float)
        rpm = np.array(rpm_data, dtype=float)
        pwm = np.array(pwm_data, dtype=float)
        err = np.abs(target - rpm)
        eps = np.finfo(float).eps

        max_t = max(max_time, 1e-6)
        itae = np.trapezoid(t * err, t)
        itae_norm = itae / max(target * max_t ** 2, eps)

        band = 0.10 * target
        in_band = np.abs(rpm - target) <= band
        settle_idx = len(rpm)
        for k in range(max(0, len(rpm) - 5)):
            if np.all(in_band[k:k + 5]):
                settle_idx = k
                break

        last_part = slice(settle_idx, len(rpm))
        if len(rpm[last_part]) < 3:
            last_part = slice(int(0.7 * len(rpm)), len(rpm))

        ess = np.mean(np.abs(target - rpm[last_part])) / max(target, eps)
        ripple = np.std(rpm[last_part]) / max(target, eps)

        if len(pwm) > 1:
            pwm_move = np.mean(np.abs(np.diff(pwm))) / 255.0
        else:
            pwm_move = 1.0

        overshoot = max(0.0, (np.max(rpm) - target) / max(target, eps))

        idx10_candidates = np.where(rpm >= 0.10 * target)[0]
        idx90_candidates = np.where(rpm >= 0.90 * target)[0]
        if idx10_candidates.size > 0 and idx90_candidates.size > 0:
            idx10 = idx10_candidates[0]
            idx90 = idx90_candidates[0]
            if idx90 > idx10:
                rise_time = t[idx90] - t[idx10]
                rise_norm = rise_time / max(max_time, eps)
            else:
                rise_norm = 1.0
        else:
            rise_norm = 1.0

        cost = (
            0.35 * itae_norm +
            0.25 * ess +
            0.20 * ripple +
            0.10 * overshoot +
            0.05 * rise_norm +
            0.05 * pwm_move
        )

        if np.max(rpm) > 1.35 * target:
            cost *= 4
        if rpm[-1] < 0.5 * target:
            cost *= 4

        cost = float(cost)
        self.after(0, lambda t=t, r=rpm, p=pwm, cost=cost: self._update_opt_plot(t, r, p, target, cost))
        return cost

    def _update_opt_plot(self, t, rpm, pwm, target, cost):
        self.ax_opt.clear()
        self._style_ax(self.ax_opt, f"Current Trial (Cost: {cost:.4f})", "Time (s)", "RPM / PWM")
        self.ax_opt.plot(t, np.full_like(t, target), color="#ffb703", label="Target", linewidth=2)
        self.ax_opt.plot(t, rpm, color="#38bdf8", label="Actual RPM", linewidth=2)
        self.ax_opt.plot(t, pwm, color="#ff4bd8", label="PWM", linewidth=1.5, linestyle="--")
        self.ax_opt.legend(loc="upper right", facecolor=self.col_panel, edgecolor=self.col_border, labelcolor="white")
        self.canvas_opt.draw()

    def _update_opt_plot_live(self, t, rpm, pwm, target):
        self.ax_opt.clear()
        self._style_ax(self.ax_opt, "Evaluating PID (Live)", "Time (s)", "RPM / PWM")
        self.ax_opt.plot(t, np.full_like(t, target), color="#ffb703", label="Target", linewidth=2)
        self.ax_opt.plot(t, rpm, color="#38bdf8", label="Actual RPM", linewidth=2)
        self.ax_opt.plot(t, pwm, color="#ff4bd8", label="PWM", linewidth=1.5, linestyle="--")
        self.ax_opt.legend(loc="upper right", facecolor=self.col_panel, edgecolor=self.col_border, labelcolor="white")
        self.canvas_opt.draw()

if __name__ == "__main__":
    app = ControllingDCMotorApp()
    app.mainloop()
