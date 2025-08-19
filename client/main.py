import socket
from encode import encode_message
import tkinter as tk
from tkinter import simpledialog, messagebox
from tkinter.scrolledtext import ScrolledText

ESP32_IP = "192.168.1.214"  # Replace with your ESP32's IP
ESP32_PORT = 8080             # Port used by ESP32

def send_message(message):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(5)
        sock.connect((ESP32_IP, ESP32_PORT))
        sock.sendall(encode_message(message))

        # Manually collect bytes until null
        data = bytearray()
        while True:
            chunk = sock.recv(4096)
            if not chunk:  # no more data
                break
            data.extend(chunk)

        return data.decode(errors="replace")


class DeviceGUI:
    def __init__(self, root):
        self.root = root
        root.title("Device Controller")
        root.minsize(500, 400)

        # Top frame for buttons
        btn_frame = tk.Frame(root)
        btn_frame.pack(side="top", fill="x", padx=8, pady=8)

        # Function buttons with optional arg types: 'int' or 'str'
        functions = [
            ("Get Lock", "get_lock"),
            ("Get Position", "get_position"),
            ("Debug Push", "debug_push", int),
            ("Debug Retract", "debug_retract", int),
            ("Zero", "zero"),
            ("Get Max", "get_max"),
            ("Set Max", "set_max", int),
            ("Unlock", "unlock"),
            ("Get Time", "get_time"),
            ("Get State", "get_state"),
            ("Safe Mode", "safe_mode"),
            ("Calibrate Mode", "calibrate_mode"),
            ("Get Thermostat", "get_thermo"),
            ("Set Thermostat", "set_thermo", int),
            ("Read Temperature", "read_temperature"),
            ("Get MAC Address", "get_mac_address"),
            ("Schedule state", "schedule_state"),
            ("Test state", "test_state"),
            ("Soft Reset", "soft_reset"),
            ("Reset Reason", "get_reset_reason"),
            ("Registers dump", "m_reg_dump"),
            ("Panic Reason", "m_panic_reason"),
            ("Stack dump", "m_stack_dump"),
            ("Sync Locally", "local_sync", int),
            ("Change schedule", "change_schedule", str),
            ("Get Schedule choice", "get_choice"),
            ("Get state up time", "state_duration"),
            ("Short Boost", "short_boost"),
            ("Long Boost", "long_boost"),
            ("Set Short Boost", "set_short_boost", int),
            ("Set Long Boost", "set_long_boost", int),
            ("Set Timezone", "set_timezone", str),
        ]

        functions = [
            ("Unlock", "Unlock"),
            ("Get lock", "GetLock"),
            ("Get position", "GetPosition"),
            ("Calibrate push", "CalibratePush", int),
            ("Calibrate pull", "CalibratePull", int),
            ("Zero", "Zero"),
            ("Get max", "GetMax"),
            ("Set max", "SetMax", int),
            ("Get thermostat", "GetThermostat"),
            ("Set thermostat", "SetThermostat", float),
            ("Read temperature", "ReadTemperature"),
            ("Get MAC address", "GetMacAddress"),
            ("Get reset reason", "GetResetReason"),
            ("Soft reset", "SoftReset"),
            ("Sync time", "SyncTime"),
            ("Test", "Test"),
        ]

        # Create a button for each function
        for label, name, *args in functions:
            arg_type = args[0] if args else None
            btn = tk.Button(
                btn_frame,
                text=label,
                width=25,
                command=lambda n=name, t=arg_type: self.send(n, t)
            )
            btn.pack(pady=2, fill="x")

        # Multi-line, resizable output at the bottom
        self.output = ScrolledText(
            root,
            wrap="word",
            height=10
        )
        self.output.pack(side="bottom", fill="both", expand=True, padx=8, pady=(0, 8))
        self.output.configure(state="disabled")

    def _append_output(self, text: str):
        self.output.configure(state="normal")
        self.output.insert("end", text + "\n")
        self.output.see("end")
        self.output.configure(state="disabled")

    def send(self, name, arg_type):
        arg_str = ""
        if arg_type:
            user_input = simpledialog.askstring("Input", f"Enter {arg_type} argument for {name}:")
            if user_input is None:
                return
            arg_str = user_input.strip()
            if not arg_str:
                messagebox.showerror("Invalid input", "Argument cannot be empty")
                return

            if arg_type is int:
                if not arg_str.isdigit():
                    messagebox.showerror("Invalid input", "Expected an integer")
                    return
            elif arg_type is str:
                if not (arg_str.startswith("'") and arg_str.endswith("'")):
                    arg_str = f"'{arg_str}'"
            else:
                messagebox.showerror("Invalid function definition", f"Unknown argument type: {arg_type}")
                return

        full_call = f"{name}({arg_str})"
        result = send_message(full_call)

        self._append_output(f"> {full_call}")
        self._append_output(f"{name}: {result if result is not None else '[no response]'}")


# Run it
if __name__ == "__main__":
    root = tk.Tk()
    gui = DeviceGUI(root)
    root.mainloop()
