import tkinter as tk
import customtkinter as ctk
import serial
import threading
import time
import math

# --- Note Frequency Data (A4 = 440 Hz, C3 to B5) ---
NUM_NOTES_IN_TABLE = 36
note_names_py = [
    "C3", "C#3/Db3", "D3", "D#3/Eb3", "E3", "F3", "F#3/Gb3", "G3", "G#3/Ab3", "A3", "A#3/Bb3", "B3",
    "C4", "C#4/Db4", "D4", "D#4/Eb4", "E4", "F4", "F#4/Gb4", "G4", "G#4/Ab4", "A4", "A#4/Bb4", "B4",
    "C5", "C#5/Db5", "D5", "D#5/Eb5", "E5", "F5", "F#5/Gb5", "G5", "G#5/Ab5", "A5", "A#5/Bb5", "B5"
]
note_ideal_frequencies_py = [
    130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 185.00, 196.00, 207.65, 220.00, 233.08, 246.94,
    261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392.00, 415.30, 440.00, 466.16, 493.88,
    523.25, 554.37, 587.33, 622.25, 659.25, 698.46, 739.99, 783.99, 830.61, 880.00, 932.33, 987.77
]
note_name_to_ideal_freq_map = {name: freq for name, freq in zip(note_names_py, note_ideal_frequencies_py)}

class TunerApp(ctk.CTk):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.title("STM32 Tuner")
        self.geometry("450x500")
        ctk.set_appearance_mode("dark") 
        ctk.set_default_color_theme("blue")

        self.serial_port = None
        self.is_reading = False
        self.read_thread = None

        # --- Animation Variables ---
        self.current_needle_x = 150.0  # Current X position of the needle (float for precision)
        self.target_needle_x = 150.0   # Target X position for the needle
        self.animation_speed_factor = 0.05 # How fast the needle moves to the target (0.0 to 1.0)
                                         # Smaller = smoother but slower. Larger = faster but more jerky.
        self.animation_interval_ms = 20  # Update interval for animation (e.g., 20ms for 50 FPS)


        # --- UI Elements ---
        self.serial_frame = ctk.CTkFrame(self)
        self.serial_frame.pack(pady=10, padx=10, fill="x")

        self.port_label = ctk.CTkLabel(self.serial_frame, text="Serial Port:")
        self.port_label.pack(side=tk.LEFT, padx=5)
        self.port_entry = ctk.CTkEntry(self.serial_frame, placeholder_text="e.g., 1103 or /dev/tty.usbmodemXXXX")
        self.port_entry.pack(side=tk.LEFT, expand=True, fill="x", padx=5)
        
        self.baud_label = ctk.CTkLabel(self.serial_frame, text="Baud:")
        self.baud_label.pack(side=tk.LEFT, padx=5)
        self.baud_entry = ctk.CTkEntry(self.serial_frame, width=80)
        self.baud_entry.insert(0, "115200")
        self.baud_entry.pack(side=tk.LEFT, padx=5)

        self.connect_button = ctk.CTkButton(self.serial_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=5)

        self.tuner_display_frame = ctk.CTkFrame(self)
        self.tuner_display_frame.pack(pady=10, padx=10, fill="both", expand=True)
        
        self.note_name_label = ctk.CTkLabel(self.tuner_display_frame, text="--", font=("Arial", 70, "bold"))
        self.note_name_label.pack(pady=15)

        self.freq_label = ctk.CTkLabel(self.tuner_display_frame, text="Detected Freq: --- Hz", font=("Arial", 18))
        self.freq_label.pack(pady=3)
        
        self.ideal_freq_label = ctk.CTkLabel(self.tuner_display_frame, text="Ideal Freq: --- Hz", font=("Arial", 16))
        self.ideal_freq_label.pack(pady=3)

        self.cents_label = ctk.CTkLabel(self.tuner_display_frame, text="Cents: ---", font=("Arial", 20))
        self.cents_label.pack(pady=8)

        self.meter_canvas = ctk.CTkCanvas(self.tuner_display_frame, width=300, height=50, bg="gray20", highlightthickness=0)
        self.meter_canvas.pack(pady=15)
        self.needle = self.meter_canvas.create_line(self.current_needle_x, 0, self.current_needle_x, 50, width=3, fill="lightgray") # Initial color
        for i in range(-2, 3): 
            x_pos = 150 + i * (300/5) 
            if i == 0:
                 self.meter_canvas.create_line(x_pos, 10, x_pos, 40, width=2, fill="white")
                 self.meter_canvas.create_text(x_pos, 45, text="0", fill="white", font=("Arial", 8))
            else:
                 self.meter_canvas.create_line(x_pos, 15, x_pos, 35, width=1, fill="gray50")
        
        self.status_label = ctk.CTkLabel(self, text="Status: Disconnected", font=("Arial", 12))
        self.status_label.pack(pady=5, side=tk.BOTTOM, fill="x")

        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.animate_needle() # Start the animation loop

    def animate_needle(self):
        """Continuously animates the needle towards the target position."""
        # Calculate the difference
        dx = self.target_needle_x - self.current_needle_x
        
        # If the difference is very small, snap to target and don't move excessively
        if abs(dx) < 0.5: # Threshold to reduce micro-movements
            self.current_needle_x = self.target_needle_x
        else:
            # Move a fraction of the remaining distance
            self.current_needle_x += dx * self.animation_speed_factor
        
        # Update the canvas item
        self.meter_canvas.coords(self.needle, self.current_needle_x, 0, self.current_needle_x, 50)
        
        # Reschedule the animation
        self.after(self.animation_interval_ms, self.animate_needle)

    def toggle_connection(self):
        # ... (這部分不變) ...
        if self.serial_port and self.serial_port.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        # ... (這部分不變, 但注意 macOS 的 port_entry 修改) ...
        port_suffix = self.port_entry.get()
        # Assuming macOS like port naming, adjust if needed for other OS
        if port_suffix.startswith("/dev/"): # User entered full path
            port = port_suffix
        elif port_suffix: # User entered just the suffix like "1103" or "usbmodemXXXX"
             # Try common macOS prefixes; make this more robust for other OS if needed
            if "usbmodem" in port_suffix or "cu." in port_suffix: # Heuristic
                 port = f"/dev/tty.{port_suffix}" if not port_suffix.startswith("cu.") else f"/dev/{port_suffix}"
                 if not "cu." in port: # Prefer cu. for macOS
                     port_cu_version = f"/dev/cu.{port_suffix.replace('tty.','')}"
                     # Very basic check, ideally would test port existence
                     # For now, just use the constructed one, user needs to be precise
                     # A dropdown list of available ports would be a better UX
            else: # Fallback for Windows-like COM ports or other patterns
                 port = port_suffix # For COM ports, it's just "COMx"
        else:
            self.status_label.configure(text="Status: Please enter a serial port.")
            return

        # Corrected port entry for macOS (example)
        # port = "/dev/tty.usbmodem" + self.port_entry.get() # Original macOS specific line
        # For more general use:
        # port = self.port_entry.get() # User enters full path like COM3 or /dev/tty.usbmodemXXXX

        try:
            baudrate = int(self.baud_entry.get())
        except ValueError:
            self.status_label.configure(text="Status: Invalid baud rate")
            return

        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.is_reading = True
            self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.read_thread.start()
            self.connect_button.configure(text="Disconnect")
            self.status_label.configure(text=f"Status: Connected to {port}")
            self.port_entry.configure(state="disabled")
            self.baud_entry.configure(state="disabled")
        except serial.SerialException as e:
            self.status_label.configure(text=f"Status: Error - {e}")
            self.serial_port = None


    def disconnect_serial(self):
        # ... (這部分不變) ...
        if self.serial_port and self.serial_port.is_open:
            self.is_reading = False 
            if self.read_thread and self.read_thread.is_alive():
                 self.read_thread.join(timeout=1) 
            self.serial_port.close()
            self.serial_port = None
        self.connect_button.configure(text="Connect")
        self.status_label.configure(text="Status: Disconnected")
        self.port_entry.configure(state="normal")
        self.baud_entry.configure(state="normal")
        # When disconnecting, smoothly move needle to center
        self.target_needle_x = 150.0 
        self.update_gui_safe("--", 0.0, 0.0, 0.0, is_unknown=True) # Update labels and needle color


    def read_serial_data(self):
        # ... (這部分不變) ...
        print("Serial reading thread started.")
        while self.is_reading and self.serial_port and self.serial_port.is_open:
            line = "" 
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    parts = line.split(',')
                    if len(parts) == 2:
                        note_name_str = parts[0].strip()
                        ratio_str = parts[1].strip()
                        if note_name_str == "-1" and ratio_str == "-1":
                            self.after(0, self.update_gui_safe, "N/A", 0.0, 0.0, 0.0, None, True)
                        else:
                            ratio = float(ratio_str)
                            cents = ratio * 100.0
                            ideal_freq = note_name_to_ideal_freq_map.get(note_name_str, 0.0)
                            detected_freq_approx = 0.0
                            if ideal_freq > 0:
                                detected_freq_approx = ideal_freq * math.pow(2, cents / 1200.0)
                            else: 
                                note_name_str = "N/A"
                            self.after(0, self.update_gui_safe, note_name_str, detected_freq_approx, ideal_freq, cents, None, False)
                    else:
                        print(f"Invalid format: {line}")
                        self.after(0, self.update_gui_safe, "ERR", 0.0, 0.0, 0.0, "Format Error", False)
            except ValueError: 
                print(f"Could not parse: {line}")
                self.after(0, self.update_gui_safe, "ERR", 0.0, 0.0, 0.0, "Data Error", False)
            except serial.SerialException as e:
                print(f"SerialException in read_serial_data: {e}")
                self.after(0, self.disconnect_serial) 
                break 
            except Exception as e:
                print(f"Error in serial thread: {e}")
                self.after(0, self.update_gui_safe, "ERR", 0.0, 0.0, 0.0, str(e), False)
        print("Serial reading thread finished.")
    
    def update_gui_safe(self, note_name, detected_freq, ideal_freq, cents, error_msg=None, is_unknown=False):
        # Update labels (this part remains mostly the same)
        if error_msg:
            self.note_name_label.configure(text="ERR")
            self.freq_label.configure(text=f"Detected Freq: --- Hz")
            self.ideal_freq_label.configure(text=f"Ideal Freq: --- Hz")
            self.cents_label.configure(text=f"Cents: ---")
            self.status_label.configure(text=f"Status: Error - {error_msg}")
            self.target_needle_x = 150.0 # Center needle on error
            self.meter_canvas.itemconfig(self.needle, fill="red") # Keep needle color for error
        elif is_unknown:
            self.note_name_label.configure(text="--")
            self.freq_label.configure(text=f"Detected Freq: --- Hz")
            self.ideal_freq_label.configure(text=f"Ideal Freq: --- Hz")
            self.cents_label.configure(text=f"Cents: ---")
            self.target_needle_x = 150.0 # Center needle for unknown
            self.meter_canvas.itemconfig(self.needle, fill="lightgray")
        else:
            display_note_name = note_name.split('/')[0] 
            self.note_name_label.configure(text=display_note_name)
            self.freq_label.configure(text=f"Detected Freq: {detected_freq:.2f} Hz")
            self.ideal_freq_label.configure(text=f"Ideal Freq: {ideal_freq:.2f} Hz")
            self.cents_label.configure(text=f"Cents: {cents:.1f}")
            
            # --- DON'T directly move needle here. Instead, set the TARGET position ---
            new_target_x = 150 + (cents * 2.5) # Adjust multiplier for sensitivity
            self.target_needle_x = max(10, min(290, new_target_x)) # Clamp target

            # Update needle color based on cents (this can still be immediate)
            # You can also choose to make color changes smooth/delayed if desired, but immediate is often fine.
            # Using relaxed thresholds as an example, adjust as needed:
            if abs(cents) < 10:  # Relaxed "in-tune" range to +/- 10 cents
                self.meter_canvas.itemconfig(self.needle, fill="green")
            elif abs(cents) < 25: # Relaxed "slightly off" range
                 self.meter_canvas.itemconfig(self.needle, fill="yellow")
            else: 
                self.meter_canvas.itemconfig(self.needle, fill="red")
            
        if self.serial_port and self.serial_port.is_open and not error_msg:
                 self.status_label.configure(text=f"Status: Connected to {self.serial_port.port}")


    def on_closing(self):
        # ... (這部分不變) ...
        print("Closing application...")
        self.is_reading = False # Signal animation loop and serial thread to stop (if after is part of it)
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=0.5) # Give some time for thread to exit
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.destroy()

if __name__ == "__main__":
    app = TunerApp()
    app.mainloop()