import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import json
import threading
import time
import math
import fnmatch
import subprocess # Import for running external script

# === CONFIG ===
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.5
ANGLE_MIN = 0
ANGLE_MAX = 350
GET_SERVO_RESPONSE_TIMEOUT = 2.0
DEVICE_INFO_TIMEOUT = 3.0
ACC = 120
SS = 600

# --- NEW CONFIG ---
# IMPORTANT: Replace this with the actual path to your Python camera script.
# Example: PYTHON_CAMERA_SCRIPT_PATH = "/home/user/my_robot_project/camera_app.py"
PYTHON_CAMERA_SCRIPT_PATH = "./running-code-v2.py"
# --- END NEW CONFIG ---

class RoArmController:
    def __init__(self, root):
        self.root = root
        self.root.title("RoArm-M1 Serial Controller")
        self.root.geometry("740x800")
        self.root.resizable(False, False)

        # Instance variables
        self.serial_connection = None
        self.reader_thread = None
        self.reader_thread_stop = threading.Event()
        self.camera_process = None # To hold the camera script subprocess

        # For temporary "send & wait for next response" operations
        self._expect_response = False
        self._response_event = threading.Event()
        self._response_buffer = None

        # Separate mechanism for device-info-on-connect to avoid collisions
        self._expect_device_info = False
        self._device_info_event = threading.Event()
        self._device_info_buffer = None

        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Status: Disconnected")

        # Entry variables
        self.p1_var = tk.StringVar(value="90")
        self.p2_var = tk.StringVar(value="90")
        self.p3_var = tk.StringVar(value="90")
        self.p4_var = tk.StringVar(value="90")

        # --- NEW: Panel management ---
        # List to hold panels that are part of the main interface and can be hidden/shown
        self.main_interface_panels = []
        self.cv_settings_panel = None # Will hold the computer vision settings panel
        # --- END NEW ---

        self.create_widgets()
        self.populate_ports()
        self.update_control_state() # Call to set initial widget states

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def create_widgets(self):
        # Connection Frame (This frame should always be visible)
        self.conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=(10, 5)) # Made self.conn_frame
        self.conn_frame.pack(padx=10, pady=10, fill="x")
        self.conn_frame.columnconfigure(1, weight=1)

        ttk.Label(self.conn_frame, text="Serial Port:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.port_combobox = ttk.Combobox(self.conn_frame, textvariable=self.port_var, state="readonly")
        self.port_combobox.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

        self.refresh_button = ttk.Button(self.conn_frame, text="Refresh", command=self.populate_ports)
        self.refresh_button.grid(row=0, column=2, padx=5, pady=5)

        self.connect_button = ttk.Button(self.conn_frame, text="Connect", command=self.connect_serial)
        self.connect_button.grid(row=1, column=0, padx=5, pady=5)

        self.disconnect_button = ttk.Button(self.conn_frame, text="Disconnect", command=self.disconnect_serial)
        self.disconnect_button.grid(row=1, column=1, padx=5, pady=5, sticky="w")

        # --- NEW SECTION: Computer Vision ---
        self.cv_frame = ttk.LabelFrame(self.root, text="Computer Vision", padding=(10, 5))
        self.cv_frame.pack(padx=10, pady=5, fill="x")
        self.cv_frame.columnconfigure(0, weight=1)
        self.main_interface_panels.append(self.cv_frame)

        self.launch_camera_button = ttk.Button(self.cv_frame, text="Launch Camera", command=self.show_cv_panel)
        self.launch_camera_button.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        # Articulation Control
        self.ctrl_frame = ttk.LabelFrame(self.root, text="Articulation Control (P1-P4 Degrees)", padding=(10, 5))
        self.ctrl_frame.pack(padx=10, pady=5, fill="x")
        self.main_interface_panels.append(self.ctrl_frame)

        ttk.Label(self.ctrl_frame, text="P1 (Base):").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        p1_entry = ttk.Entry(self.ctrl_frame, textvariable=self.p1_var, width=7, justify='center')
        p1_entry.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(self.ctrl_frame, text="P2 (Large Arm):").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        p2_entry = ttk.Entry(self.ctrl_frame, textvariable=self.p2_var, width=7, justify='center')
        p2_entry.grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(self.ctrl_frame, text="P3 (Small Arm):").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        p3_entry = ttk.Entry(self.ctrl_frame, textvariable=self.p3_var, width=7, justify='center')
        p3_entry.grid(row=1, column=1, padx=5, pady=5)

        ttk.Label(self.ctrl_frame, text="P4 (Gripper):").grid(row=1, column=2, padx=5, pady=5, sticky="w")
        p4_entry = ttk.Entry(self.ctrl_frame, textvariable=self.p4_var, width=7, justify='center')
        p4_entry.grid(row=1, column=3, padx=5, pady=5)

        # Servo 5 Control
        self.s5_frame = ttk.LabelFrame(self.root, text="Servo 5 Control (Tool)", padding=(10, 5))
        self.s5_frame.pack(padx=10, pady=5, fill="x")
        self.s5_frame.columnconfigure(0, weight=1)
        self.s5_frame.columnconfigure(1, weight=1)
        self.main_interface_panels.append(self.s5_frame)

        self.s5_open_button = ttk.Button(self.s5_frame, text="Release", command=self.servo5_open)
        self.s5_open_button.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        self.s5_close_button = ttk.Button(self.s5_frame, text="Catch", command=self.servo5_close)
        self.s5_close_button.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

        # Commands
        self.cmd_frame = ttk.LabelFrame(self.root, text="Commands", padding=(10, 5))
        self.cmd_frame.pack(padx=10, pady=5, fill="x")
        self.cmd_frame.columnconfigure(0, weight=1)
        self.cmd_frame.columnconfigure(1, weight=1)
        self.main_interface_panels.append(self.cmd_frame)

        self.torque_on_button = ttk.Button(self.cmd_frame, text="Torque ON", command=self.torque_on)
        self.torque_on_button.grid(row=0, column=0, padx=8, pady=8, sticky="ew")

        self.torque_off_button = ttk.Button(self.cmd_frame, text="Torque OFF", command=self.torque_off)
        self.torque_off_button.grid(row=0, column=1, padx=8, pady=8, sticky="ew")

        self.get_servo_button = ttk.Button(self.cmd_frame, text="Get Servo Data", command=self.get_servo_data)
        self.get_servo_button.grid(row=1, column=0, padx=8, pady=8, sticky="ew")

        self.send_button = ttk.Button(self.cmd_frame, text="Send P1-P4 Position", command=self.send_position_command)
        self.send_button.grid(row=1, column=1, padx=8, pady=8, sticky="ew")

        self.stop_button = tk.Button(self.cmd_frame, text="EMERGENCY STOP", command=self.emergency_stop,
                                       bg="red", fg="white", activebackground="#aa0000", font=('Helvetica', 12, 'bold'))
        self.stop_button.grid(row=2, column=0, columnspan=2, padx=8, pady=12, sticky="ew")

        # RX Log
        self.log_frame = ttk.LabelFrame(self.root, text="Serial Log (RX)", padding=(8, 5))
        self.log_frame.pack(padx=10, pady=5, fill="both", expand=True)
        self.log_frame.columnconfigure(0, weight=1)
        self.log_frame.rowconfigure(1, weight=1)
        self.main_interface_panels.append(self.log_frame)

        log_toolbar = ttk.Frame(self.log_frame)
        log_toolbar.grid(row=0, column=0, sticky="ew", padx=5, pady=(0,5))
        log_toolbar.columnconfigure(0, weight=1)
        self.clear_log_button = ttk.Button(log_toolbar, text="Clear Log", command=self.clear_log)
        self.clear_log_button.grid(row=0, column=1, sticky="e", padx=2)

        self.log_text = scrolledtext.ScrolledText(self.log_frame, height=14, state='disabled', wrap='none')
        self.log_text.grid(row=1, column=0, sticky="nsew", padx=5, pady=2)

        h_scroll = ttk.Scrollbar(self.log_frame, orient='horizontal', command=self.log_text.xview)
        h_scroll.grid(row=2, column=0, sticky="ew", padx=5, pady=(0,5))
        self.log_text.configure(xscrollcommand=h_scroll.set)

        # Status bar
        status_label = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor="w")
        status_label.pack(side="bottom", fill="x", ipady=2)

        # --- NEW: Computer Vision Settings Panel ---
        self.cv_settings_panel = ttk.LabelFrame(self.root, text="Computer Vision Settings", padding=(10, 10))

        ttk.Label(self.cv_settings_panel, text="Computer Vision features will be configured here.").pack(pady=20)
        ttk.Button(self.cv_settings_panel, text="Run Camera Script", command=self.launch_camera_script).pack(pady=5)
        # --- NEW BUTTON ADDED ---
        ttk.Button(self.cv_settings_panel, text="End Script & Return", command=self.end_camera_script).pack(pady=5)
        # ---
        ttk.Button(self.cv_settings_panel, text="Back to Main (no action)", command=self.show_main_panel).pack(pady=10)
        # --- END NEW ---

    # --- METHODS FOR PANEL MANAGEMENT & SCRIPT HANDLING ---
    def hide_all_main_panels(self):
        """Hides all panels that constitute the main control interface."""
        for panel in self.main_interface_panels:
            panel.pack_forget()

    def show_main_panel(self):
        """Shows the main control interface and hides the computer vision settings panel."""
        if self.cv_settings_panel:
            self.cv_settings_panel.pack_forget()

        for i, panel in enumerate(self.main_interface_panels):
            if i == len(self.main_interface_panels) - 1: # This is the log_frame
                panel.pack(padx=10, pady=5, fill="both", expand=True)
            else:
                panel.pack(padx=10, pady=5, fill="x")
        self.update_control_state()

    def show_cv_panel(self):
        """Hides the main control interface and shows the computer vision settings panel."""
        self.hide_all_main_panels()
        self.cv_settings_panel.pack(padx=10, pady=10, fill="both", expand=True)

    def launch_camera_script(self):
        """Launches the external Python camera script using subprocess."""
        if self.camera_process and self.camera_process.poll() is None:
            messagebox.showinfo("Already Running", "The camera script process is already running.")
            return

        script_path = PYTHON_CAMERA_SCRIPT_PATH
        if not script_path or not script_path.endswith(".py"):
            messagebox.showerror("Script Error", "Please configure a valid Python script path in PYTHON_CAMERA_SCRIPT_PATH.")
            return

        try:
            self.camera_process = subprocess.Popen(['python', script_path])
            messagebox.showinfo("Launch Camera", f"Attempting to launch camera script: {script_path}")
            self.status_var.set("Status: Camera script launched.")
        except FileNotFoundError:
            messagebox.showerror("Script Error", f"Python interpreter or script not found at {script_path}.\n"
                                                 "Check the path and ensure Python is in your system's PATH.")
        except Exception as e:
            messagebox.showerror("Script Error", f"Error launching camera script: {e}")

    def end_camera_script(self):
        """Terminates the camera script process and returns to the main panel."""
        if self.camera_process and self.camera_process.poll() is None:
            try:
                self.camera_process.terminate() # Send SIGTERM
                self.camera_process.wait(timeout=2) # Wait for process to terminate
                messagebox.showinfo("Script Ended", "The camera script has been terminated.")
                self.status_var.set("Status: Camera script terminated.")
            except subprocess.TimeoutExpired:
                self.camera_process.kill() # Force kill if it doesn't respond
                messagebox.showwarning("Script Ended", "The camera script did not respond and was force-killed.")
                self.status_var.set("Status: Camera script force-killed.")
            except Exception as e:
                messagebox.showerror("Error", f"Could not terminate script: {e}")
            finally:
                self.camera_process = None
        else:
            messagebox.showinfo("Info", "Camera script is not running or already closed.")
            self.status_var.set("Status: Camera script was not running.")

        # Always return to the main panel
        self.show_main_panel()
    # --- END NEW METHODS ---

    def populate_ports(self):
        """List available serial ports but only include /dev/ttyUSB* devices."""
        try:
            all_ports = [p.device for p in serial.tools.list_ports.comports()]
            usb_ports = [p for p in all_ports if fnmatch.fnmatch(p, "/dev/ttyUSB*")]
            self.port_combobox['values'] = usb_ports
            if usb_ports:
                self.port_combobox.current(0)
                self.port_var.set(usb_ports[0])
            else:
                self.port_combobox.set('')
                self.port_var.set('')
        except Exception as e:
            self.status_var.set(f"Error listing ports: {e}")

    def connect_serial(self):
        """Open serial in a non-blocking thread so GUI stays responsive."""
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Connection Error", "No serial port selected.")
            return

        t = threading.Thread(target=self._connect_worker, args=(port,), daemon=True)
        t.start()

    def _connect_worker(self, port):
        try:
            ser = serial.Serial(port, baudrate=SERIAL_BAUD, timeout=SERIAL_TIMEOUT)
            def on_success():
                self.serial_connection = ser
                self.status_var.set(f"Status: Connected to {port}")
                self.update_control_state()
                self.reader_thread_stop.clear()
                self.reader_thread = threading.Thread(target=self._reader_worker, daemon=True)
                self.reader_thread.start()

                self._device_info_event.clear()
                self._device_info_buffer = None
                self._expect_device_info = True
                self._send_command({'T': 4})

                def device_info_waiter():
                    got = self._device_info_event.wait(DEVICE_INFO_TIMEOUT)
                    if got and self._device_info_buffer is not None:
                        buf = self._device_info_buffer
                        try:
                            pretty = json.dumps(json.loads(buf), indent=2)
                        except Exception:
                            pretty = buf
                        self.root.after(0, lambda: messagebox.showinfo("Device Info Received", pretty))
                    else:
                        self.root.after(0, lambda: messagebox.showinfo("Device Info", f"No device info reply within {DEVICE_INFO_TIMEOUT} s.\nCheck RX log for later messages."))
                threading.Thread(target=device_info_waiter, daemon=True).start()

            self.root.after(0, on_success)

        except (serial.SerialException, PermissionError) as e:
            self.root.after(0, lambda: self._handle_connect_error(port, str(e)))
        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("Connection Error", f"Unexpected error: {e}"))

    def _handle_connect_error(self, port, err_msg):
        self.status_var.set(f"Status: Error - {err_msg}")
        extra = (
            f"Could not open {port}.\n\n"
            "Common fixes:\n"
            " • Make sure the device path is correct.\n"
            " • Add your user to the dialout group (safer):\n"
            "     sudo usermod -a -G dialout $USER\n"
            "   then log out and back in.\n"
            " • As a last resort, temporarily relax permissions:\n"
            "     sudo chmod 666 /dev/ttyUSB0\n\n"
        )
        messagebox.showerror("Connection Error", f"Failed to open {port}:\n{err_msg}\n\n{extra}")

    def disconnect_serial(self):
        """Close serial and stop reader thread."""
        if self.serial_connection:
            try:
                self.reader_thread_stop.set()
                if self.serial_connection.is_open:
                    self.serial_connection.close()
                self.status_var.set("Status: Disconnected")
            except Exception as e:
                self.status_var.set(f"Status: Error - {e}")
        self.serial_connection = None
        self.update_control_state()

    def _reader_worker(self):
        """Continuously read lines from the serial port and append to log."""
        ser = self.serial_connection
        while not self.reader_thread_stop.is_set() and ser and ser.is_open:
            try:
                line = ser.readline()
                if not line:
                    continue
                text = line.decode('utf-8', errors='replace').strip()
                self.root.after(0, lambda t=text: self.append_log("RX: " + t))

                if self._expect_device_info:
                    self._device_info_buffer = text
                    self._expect_device_info = False
                    self._device_info_event.set()
                    continue

                if self._expect_response:
                    self._response_buffer = text
                    self._expect_response = False
                    self._response_event.set()

            except Exception:
                break

    def append_log(self, text):
        self.log_text.configure(state='normal')
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{timestamp}] {text}\n")
        self.log_text.see("end")
        self.log_text.configure(state='disabled')

    def clear_log(self):
        """Clears the serial RX/TX log."""
        self.log_text.configure(state='normal')
        self.log_text.delete('1.0', tk.END)
        self.log_text.configure(state='disabled')

    def _send_command(self, command_dict):
        """Serialize/encode and send command. Adds newline and flushes."""
        if not (self.serial_connection and getattr(self.serial_connection, "is_open", False)):
            self.status_var.set("Status: Not connected. Cannot send command.")
            return

        try:
            json_string = json.dumps(command_dict)
            data = (json_string + "\n").encode('utf-8')
            self.serial_connection.write(data)
            self.serial_connection.flush()
            self.status_var.set(f"Status: Command sent: {json_string}")
            self.append_log("TX: " + json_string)
        except Exception as e:
            self.status_var.set(f"Status: Error sending command - {e}")
            self.disconnect_serial()

    def _safe_int(self, s):
        try:
            return int(s)
        except (ValueError, TypeError):
            raise ValueError("Not an integer")

    def send_position_command(self):
        """Read entries, validate & clamp angles, send command."""
        try:
            p1 = self._safe_int(self.p1_var.get())
            p2 = self._safe_int(self.p2_var.get())
            p3 = self._safe_int(self.p3_var.get())
            p4 = self._safe_int(self.p4_var.get())
        except ValueError:
            messagebox.showerror("Input Error", "All angle values must be valid integers.")
            return

        def clamp(v): return max(ANGLE_MIN, min(ANGLE_MAX, v))
        p1c, p2c, p3c, p4c = clamp(p1), clamp(p2), clamp(p3), clamp(p4)

        if (p1c, p2c, p3c, p4c) != (p1, p2, p3, p4):
            messagebox.showinfo("Clamped", f"Some values were clamped to [{ANGLE_MIN}, {ANGLE_MAX}]")

        command = {'T': 1,'P1':p1c,'P2':p2c,'P3':p3c,'P4':p4c,'P5':180,'S1':SS,'S2':SS,'S3':SS,'S4':SS,'S5':SS,'A1':ACC,'A2':ACC,'A3':ACC,'A4':ACC,'A5':ACC}
        self._send_command(command)

    def servo5_open(self):
        pos = int(2047 + (-1 * (1 * math.pi / 180) / math.pi) * 2048 + 0.5)
        self._send_command({'T':9,'P1':5,'P2':2,'P3':pos})

    def servo5_close(self):
        pos = int(2047 + ((92 * math.pi / 180) / math.pi) * 2048 + 0.5)
        self._send_command({'T':9,'P1':5,'P2':2,'P3':pos})

    def emergency_stop(self):
        for _ in range(3):
            self._send_command({'T': 0})
            time.sleep(0.03)

    def get_servo_data(self):
        """Sends {'T': 5} to request servo data and waits for a response."""
        if not (self.serial_connection and getattr(self.serial_connection, "is_open", False)):
            messagebox.showerror("Not connected", "Serial port not connected.")
            return

        self._response_event.clear()
        self._response_buffer = None
        self._expect_response = True
        self._send_command({'T': 5})

        def waiter():
            if self._response_event.wait(GET_SERVO_RESPONSE_TIMEOUT) and self._response_buffer:
                self.root.after(0, lambda: messagebox.showinfo("Servo Data Received", self._response_buffer))
            else:
                self.root.after(0, lambda: messagebox.showinfo("No Response", f"No servo data reply within {GET_SERVO_RESPONSE_TIMEOUT}s.\nCheck RX log for later messages."))

        threading.Thread(target=waiter, daemon=True).start()

    def torque_on(self):
        self._send_command({'T': 8, 'P1': 1})

    def torque_off(self):
        self._send_command({'T': 8, 'P1': 0})

    def update_control_state(self):
        """Enable/disable widgets depending on connection status."""
        is_connected = self.serial_connection and getattr(self.serial_connection, "is_open", False)

        # Connection frame widgets
        self.connect_button.config(state=tk.DISABLED if is_connected else tk.NORMAL)
        self.disconnect_button.config(state=tk.NORMAL if is_connected else tk.DISABLED)
        self.refresh_button.config(state=tk.DISABLED if is_connected else tk.NORMAL)
        self.port_combobox.config(state=tk.DISABLED if is_connected else "readonly")

        arm_control_state = tk.NORMAL if is_connected else tk.DISABLED

        for frame in self.main_interface_panels:
            if frame == self.cv_frame:
                self.launch_camera_button.config(state=tk.NORMAL)
            elif frame == self.log_frame:
                self.clear_log_button.configure(state=tk.NORMAL)
                self.log_text.configure(state='disabled')
            else: # ctrl_frame, s5_frame, cmd_frame
                for child in frame.winfo_children():
                    try:
                        child.configure(state=arm_control_state)
                    except (tk.TclError, TypeError):
                        pass

    def on_closing(self):
        # --- MODIFIED: Ensure camera script is terminated on exit ---
        if self.camera_process and self.camera_process.poll() is None:
            self.camera_process.terminate()
        # ---
        self.disconnect_serial()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = RoArmController(root)
    root.mainloop()