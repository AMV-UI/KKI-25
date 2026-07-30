import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import subprocess
import sys
import os
import threading

# ================= CONFIGURATION =================
IMAGE_PATH = "airframe_overlay.png"
SOURCE_CODE_PATH = "/path/to/ardupilot/libraries/AP_Motors/AP_Motors6DOF.cpp"
DOCKER_CONTAINER_NAME = "ardupilot_dev_container"
ARDUPILOT_WORKSPACE = "/ardupilot"  # Path inside the docker container

# Coordinates for the input fields on your specific image (x, y)
# Adjust these based on your 8-prop vectored airframe image
MOTOR_COORDS = {
    1: (150, 100),  # Front Right
    2: (450, 100),  # Front Left
    3: (150, 400),  # Rear Right
    4: (450, 400),  # Rear Left
    5: (200, 200),  # Vertical Front Right
    6: (400, 200),  # Vertical Front Left
    7: (200, 300),  # Vertical Rear Right
    8: (400, 300),  # Vertical Rear Left
}

DIMENSIONS = ["Roll", "Pitch", "Yaw", "Throttle", "Forward", "Lateral"]
# =================================================


class MotorMatrixTuner:
    def __init__(self, root):
        self.root = root
        self.root.title("ArduSub Motor Matrix Tuner")

        # Initialize data structure holding 8 motors x 6 DOFs
        self.matrix_data = {
            m: {dim: 0.0 for dim in DIMENSIONS} for m in MOTOR_COORDS.keys()
        }

        self.current_dim = tk.StringVar(value=DIMENSIONS[4])  # Default to Forward
        self.entry_widgets = {}

        self._build_top_frame()
        self._build_canvas_frame()
        self._build_bottom_frame()

        # Load initial values into entries
        self.load_entries()

    def _build_top_frame(self):
        top_frame = tk.Frame(self.root, pady=10)
        top_frame.pack(side=tk.TOP, fill=tk.X)

        tk.Label(top_frame, text="Select Dimension:", font=("Arial", 12, "bold")).pack(
            side=tk.LEFT, padx=10
        )

        for dim in DIMENSIONS:
            rb = tk.Radiobutton(
                top_frame,
                text=dim,
                variable=self.current_dim,
                value=dim,
                command=self.on_dimension_change,
                font=("Arial", 10),
            )
            rb.pack(side=tk.LEFT, padx=5)

    def _build_canvas_frame(self):
        canvas_frame = tk.Frame(self.root)
        canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Load Image
        try:
            self.bg_image = Image.open(IMAGE_PATH)
            self.bg_photo = ImageTk.PhotoImage(self.bg_image)
            width, height = self.bg_image.size
        except FileNotFoundError:
            # Fallback if image is missing so the program still runs
            width, height = 600, 500
            self.bg_photo = tk.PhotoImage(width=width, height=height)
            print(f"Warning: {IMAGE_PATH} not found. Using blank background.")

        self.canvas = tk.Canvas(canvas_frame, width=width, height=height)
        self.canvas.pack()
        self.canvas.create_image(0, 0, image=self.bg_photo, anchor=tk.NW)

        # Create Entry widgets at coordinates
        for motor_id, (x, y) in MOTOR_COORDS.items():
            # Label
            lbl = tk.Label(self.canvas, text=f"M{motor_id}", bg="white")
            self.canvas.create_window(x - 30, y, window=lbl)

            # Entry
            ent = tk.Entry(self.canvas, width=6, justify="center")
            self.canvas.create_window(x + 15, y, window=ent)
            self.entry_widgets[motor_id] = ent

    def _build_bottom_frame(self):
        bottom_frame = tk.Frame(self.root, pady=15)
        bottom_frame.pack(side=tk.BOTTOM, fill=tk.X)

        self.run_btn = tk.Button(
            bottom_frame,
            text="Patch Source & Flash (Pixhawk1)",
            font=("Arial", 12, "bold"),
            bg="green",
            fg="white",
            command=self.execute_workflow,
        )
        self.run_btn.pack()

    def on_dimension_change(self):
        # Save current UI entries to data structure, then load the new ones
        self.save_entries()
        self.load_entries()

    def save_entries(self):
        # We need to know which dimension we are saving.
        # Using the current_dim variable might be tricky if it already changed.
        # But Tkinter updates current_dim immediately on click.
        # To be safe, we track the *previous* dimension.
        pass  # Handled smoothly below by passing previous dim, but for simplicity:

    def load_entries(self):
        dim = self.current_dim.get()
        for motor_id, ent in self.entry_widgets.items():
            ent.delete(0, tk.END)
            ent.insert(0, str(self.matrix_data[motor_id][dim]))

    # Capture the previous dimension before the radiobutton switches
    last_dim = DIMENSIONS[4]

    def on_dimension_change(self):
        # Save values for the dimension we just left
        for motor_id, ent in self.entry_widgets.items():
            try:
                val = float(ent.get())
                self.matrix_data[motor_id][self.last_dim] = val
            except ValueError:
                pass  # Ignore invalid inputs

        self.last_dim = self.current_dim.get()
        self.load_entries()

    def generate_cpp_code(self):
        # Make sure current view is saved
        self.on_dimension_change()

        code_lines = []
        for i in range(1, 9):
            d = self.matrix_data[i]
            # add_motor_raw(motor_num, roll, pitch, yaw, throttle, forward, lateral, testing_order);
            line = f"    add_motor_raw(AP_MOTORS_MOT_{i}, {d['Roll']:>4}, {d['Pitch']:>4}, {d['Yaw']:>4}, {d['Throttle']:>4}, {d['Forward']:>4}, {d['Lateral']:>4}, {i});"
            code_lines.append(line)
        return "\n".join(code_lines) + "\n"

    def execute_workflow(self):
        self.run_btn.config(state=tk.DISABLED, text="Running...")

        # 1. Patch the source code
        try:
            self.patch_source_code()
        except Exception as e:
            messagebox.showerror("File Error", f"Failed to patch source:\n{e}")
            self.run_btn.config(state=tk.NORMAL, text="Patch Source & Flash")
            return

        # 2. Run Docker in a separate thread so UI doesn't freeze
        threading.Thread(target=self.run_docker_exec, daemon=True).start()

    def patch_source_code(self):
        """
        Assumes your AP_Motors6DOF.cpp has markers like:
        // --- CUSTOM MOTOR MATRIX START ---
        ...
        // --- CUSTOM MOTOR MATRIX END ---
        """
        if not os.path.exists(SOURCE_CODE_PATH):
            print(
                f"Warning: {SOURCE_CODE_PATH} not found. Skipping file write for testing."
            )
            print("Generated Code:\n" + self.generate_cpp_code())
            return

        with open(SOURCE_CODE_PATH, "r") as file:
            lines = file.readlines()

        start_idx = -1
        end_idx = -1
        for i, line in enumerate(lines):
            if "// --- CUSTOM MOTOR MATRIX START ---" in line:
                start_idx = i
            elif "// --- CUSTOM MOTOR MATRIX END ---" in line:
                end_idx = i

        if start_idx != -1 and end_idx != -1:
            new_code = self.generate_cpp_code()
            lines = lines[: start_idx + 1] + [new_code] + lines[end_idx:]

            with open(SOURCE_CODE_PATH, "w") as file:
                file.writelines(lines)
            print("Successfully patched C++ source code.")
        else:
            raise ValueError("Could not find start/end markers in the C++ file.")

    def run_docker_exec(self):
        print("\n" + "=" * 50)
        print("Starting Docker Compile and Upload...")
        print("=" * 50 + "\n")

        cmd = [
            "docker",
            "exec",
            DOCKER_CONTAINER_NAME,
            "bash",
            "-c",
            f"cd {ARDUPILOT_WORKSPACE} && ./waf configure --board Pixhawk1 && ./waf sub --upload",
        ]

        try:
            # Setting stdout=sys.stdout pipes the docker stream directly to the terminal running this python script
            process = subprocess.Popen(
                cmd, stdout=sys.stdout, stderr=sys.stderr, text=True
            )
            process.wait()

            if process.returncode == 0:
                print("\n✅ Firmware compiled and uploaded successfully!")
            else:
                print(f"\n❌ Docker exited with code {process.returncode}")

        except Exception as e:
            print(f"\n❌ Failed to run Docker command: {e}")

        finally:
            self.root.after(
                0,
                lambda: self.run_btn.config(
                    state=tk.NORMAL, text="Patch Source & Flash"
                ),
            )


if __name__ == "__main__":
    root = tk.Tk()
    app = MotorMatrixTuner(root)
    root.mainloop()
