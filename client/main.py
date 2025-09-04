import socket
from encode import encode_message
import tkinter as tk
from tkinter import simpledialog, messagebox
from tkinter.scrolledtext import ScrolledText

ESP32_IP = "192.168.1.214"  # Replace with your ESP32's IP
ESP32_PORT = 8080           # Port used by ESP32

def send_message(message):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(5)
        sock.connect((ESP32_IP, ESP32_PORT))
        sock.sendall(encode_message(message))

        # Collect until socket closes (server ends response)
        data = bytearray()
        while True:
            chunk = sock.recv(4096)
            if not chunk:
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

        # Function buttons with optional arg types
        functions = [

            ("Add summer slot", "AddSummerSlot", str),
            ("Add winter slot", "AddWinterSlot", str),
            ("Add brightness slot", "AddBrightnessSlot", str),

            ("Remove summer slot", "RemoveSummerSlot", str),
            ("Remove winter slot", "RemoveWinterSlot", str),
            ("Remove brightness slot", "RemoveBrightnessSlot", str),

            ("Clear Summer Schedule", "ClearSummer", str),
            ("Clear winter Schedule", "ClearWinter", str),
            ("Clear brightness Schedule", "ClearBrightness", str),

            ("Show Summer Schedule", "GetSummer"),
            ("Show winter Schedule", "GetWinter"),
            ("Show brightness Schedule", "GetBrightness"),

            ("Unlock & Zero", "UnlockAndZero"),#
            ("Lock", "Lock"),#
            ("Get lock", "GetLock"),#
            ("Get position", "GetPosition"),#
            ("Calibrate push", "CalibratePush", int),#
            ("Calibrate pull", "CalibratePull", int),#
            ("Get max", "GetMax"),#
            ("Set max", "SetMax", int),#
            ("Get thermostat", "GetThermostat"),#
            ("Set thermostat", "SetThermostat", float),#
            ("Read temperature", "ReadTemperature"),#
            ("Get MAC address", "GetMacAddress"),#
            ("Get reset reason", "GetResetReason"),#
            ("Soft reset", "SoftReset"),#
            ("Sync time", "SyncTime", int),#
            ("Get time", "GetTime"),#
            ("Enter Descale", "Descale"),#
            ("Enter Calibrate", "Calibrate"),#
            ("Enter Safe Mode", "SafeMode"),#
            ("Enter Rainbow!", "Rainbow"),#
            ("Cancel", "Cancel"),#
            ("Get state", "CurrentState"),#
            ("Get boost duration", "GetBoostDuration"),#
            ("Set short boost duration", "SetShortDuration", int),#
            ("Set long boost duration", "SetLongDuration", int),#
            ("Change schedule", "SetVariant", str),#
            ("Get schedule type", "GetVariant"),#

            ("PANIC!", "StartPanic", str),#
            ("EXCEPTION", "StartException", str),#

            ("Show Brightness", "GetLEDBrightness"),#
            ("Get Up time", "GetUpTime"),#

        ]

        # ----- 3-column grid layout for buttons -----
        BTN_COLUMNS = 3
        for c in range(BTN_COLUMNS):
            btn_frame.grid_columnconfigure(c, weight=1, uniform="btncols")

        for idx, (label, name, *args) in enumerate(functions):
            arg_type = args[0] if args else None
            r, c = divmod(idx, BTN_COLUMNS)
            tk.Button(
                btn_frame,
                text=label,
                command=lambda n=name, t=arg_type: self.send(n, t),
            ).grid(row=r, column=c, padx=4, pady=4, sticky="ew")
        # --------------------------------------------

        # Multi-line, resizable output at the bottom
        self.output = ScrolledText(root, wrap="word", height=10)
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
            user_input = simpledialog.askstring("Input", f"Enter {arg_type.__name__} argument for {name}:")
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
            elif arg_type is float:
                try:
                    float(arg_str)
                except ValueError:
                    messagebox.showerror("Invalid input", "Expected a float")
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
