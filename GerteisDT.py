import configparser
import os
import cv2
import numpy as np
from opcua import Server, ua
from simple_pid import PID
from collections import deque
import tkinter as tk
from tkinter import Button, Label, Frame
from PIL import Image, ImageTk
import threading
import logging
import datetime

# Set up logging
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s [%(levelname)s] %(message)s")

# Configuration
VIDEO_DIR = '/home/bmsadmin/video_storage'
MAX_STORAGE_BYTES = 128 * 1024 * 1024 * 1024  # 128GB storage limit
is_recording = False
last_valid_thickness = 0
reference_frame = None
CAMERA_SOURCE = 0  # Default camera
cap = None
output_file = None

# Ensure video directory exists
if not os.path.exists(VIDEO_DIR):
    os.makedirs(VIDEO_DIR)

# Load configuration
config_file_path = os.path.join(os.path.dirname(__file__), 'config.ini')
def load_config():
    config = configparser.ConfigParser()
    config.read(config_file_path)
    return {
        'true_density': float(config['CompactionSettings']['true_density']),
        'mass_flow_rate': float(config['CompactionSettings']['mass_flow_rate']),
        'ribbon_width': float(config['CompactionSettings']['ribbon_width']),
        'roller_speed': float(config['CompactionSettings']['roller_speed']),
        'set_pressure': float(config['CompactionSettings']['set_pressure']),
        'allowable_deviation': float(config['CompactionSettings']['allowable_deviation']),
        'loop_enable': config['CompactionSettings'].getboolean('loop_enable'),
        'desired_solid_fraction': float(config['CompactionSettings']['desired_solid_fraction']),
        'camera_distance': float(config['CompactionSettings']['camera_distance']),
        'curvature_threshold': float(config['CompactionSettings']['curvature_threshold']),
        'relaxation_multiplier': float(config['CompactionSettings']['relaxation_multiplier']),
        'sensitivity': float(config['CompactionSettings']['sensitivity'])
    }

config = load_config()

# Suppress detailed logging for the opcua library
opc_logger = logging.getLogger("opcua")
opc_logger.setLevel(logging.WARNING)

# Optionally suppress OpenCV verbose warnings
cv2_logger = logging.getLogger("cv2")
cv2_logger.setLevel(logging.WARNING)


# OPC UA Server Setup
server = Server()
server.set_endpoint("opc.tcp://0.0.0.0:4840/freeopcua/server/")
idx = server.register_namespace("RibbonAnalysis")
compaction_obj = server.nodes.objects.add_object(idx, "CompactionSettings")
config_nodes = {}

for key, value in config.items():
    config_nodes[key] = compaction_obj.add_variable(idx, key.capitalize(), value, ua.VariantType.Float)
    config_nodes[key].set_writable()

config_nodes['thickness'] = compaction_obj.add_variable(idx, "RibbonThickness", 0.0, ua.VariantType.Float)
config_nodes['thickness'].set_writable()

# Start the server
server.start()


# PID Controller
pid = PID(Kp=1, Ki=0.1, Kd=0.05, setpoint=config['desired_solid_fraction'])
pid.output_limits = (-config['allowable_deviation'], config['allowable_deviation'])

def normalize_reference():
    """Capture the reference frame for analysis."""
    global reference_frame, cap
    logging.info("Attempting to normalize reference frame...")
    if cap is None or not cap.isOpened():
        logging.error("Camera is not initialized or not opened.")
        return

    ret, frame = cap.read()
    if not ret:
        logging.error("Failed to capture a frame from the camera.")
        return

    reference_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if reference_frame is not None:
        logging.info("Reference frame successfully captured.")
    else:
        logging.error("Failed to process the captured frame.")


def calculate_cross_section(contour):
    rect = cv2.minAreaRect(contour)
    (width, height) = rect[1]
    return min(width, height)

def analyze_ribbon(frame):
    global last_valid_thickness, reference_frame

    if reference_frame is None:
        logging.warning("No reference frame found. Using last valid thickness.")
        return last_valid_thickness

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(reference_frame, gray)
    _, threshold = cv2.threshold(diff, config['sensitivity'], 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 5000:  # Minimum area threshold for detection
            thickness_mm = calculate_cross_section(largest_contour) * (config['camera_distance'] / 1000) * config['relaxation_multiplier'] * 0.2  # Lens Reduction
            last_valid_thickness = thickness_mm

            # Draw the contour and bounding rectangle on the frame
            cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)  # Draw contour in green
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw bounding rectangle in green
        else:
            thickness_mm = last_valid_thickness
    else:
        thickness_mm = last_valid_thickness

    config_nodes['thickness'].set_value(ua.Variant(thickness_mm, ua.VariantType.Float))
    logging.debug(f"Ribbon Thickness: {thickness_mm:.2f} mm")
    return thickness_mm




def start_recording():
    global is_recording, output_file
    is_recording = True

    # Generate a filename with the current date and time
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_path = os.path.join(VIDEO_DIR, f"recording_{timestamp}.avi")

    # Initialize the video writer
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    output_file = cv2.VideoWriter(output_path, fourcc, 20.0, (640, 480))
    print(f"Recording started: {output_path}")

def stop_recording():
    global is_recording, output_file
    is_recording = False
    if output_file:
        output_file.release()
        output_file = None
        print("Recording stopped.")

def on_closing():
    global cap, is_recording
    logging.info("Application closing.")
    if cap:
        cap.release()
    is_recording = False
    server.stop()
    root.destroy()

# Set up logging
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s [%(levelname)s] %(message)s")

# Detect screen resolution
root = tk.Tk()
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()

# Dynamically set window size to fit within screen
window_width = int(screen_width)  # 100% of screen width
window_height = int(screen_height * 0.9)  # 90% of screen height
root.geometry(f"{window_width}x{window_height}")

logging.info(f"Screen size: {screen_width}x{screen_height}")
logging.info(f"Window size: {window_width}x{window_height}")

# Divide the window into a grid layout
root.grid_rowconfigure(0, weight=3)  # Video frame
root.grid_rowconfigure(1, weight=1)  # Control frame
root.grid_columnconfigure(0, weight=1)

# Video frame for video feed
video_frame = Frame(root, bg="black", width=window_width, height=int(window_height * 0.75))
video_frame.grid(row=0, column=0, sticky="nsew")

# Control frame for buttons
control_frame = Frame(root, bg="lightgrey", width=window_width, height=int(window_height * 0.25))
control_frame.grid(row=1, column=0, sticky="nsew")

# Video label for displaying video feed
video_label = Label(video_frame, bg="black")
video_label.pack(fill="both", expand=True)

# Control buttons
button_font_size = max(6, int(screen_width / 60))  # Scale font size based on screen width
# Control buttons
button_font_size = max(6, int(screen_width / 60))  # Scale font size based on screen width

thickness_label = Label(
    control_frame, text="Thickness: 0.00 mm", font=("Helvetica", button_font_size), bg="lightgrey"
)
thickness_label.grid(row=0, column=0, padx=2, pady=2)

normalize_button = Button(
    control_frame, text="Normalize", font=("Helvetica", button_font_size), command=normalize_reference
)
normalize_button.grid(row=0, column=1, padx=2, pady=2)

start_button = Button(
    control_frame, text="Start Recording", font=("Helvetica", button_font_size), command=start_recording
)
start_button.grid(row=0, column=2, padx=2, pady=2)

stop_button = Button(
    control_frame, text="Stop Recording", font=("Helvetica", button_font_size), command=stop_recording
)
stop_button.grid(row=0, column=3, padx=2, pady=2)

exit_button = Button(
    control_frame, text="Exit", font=("Helvetica", button_font_size), command=on_closing
)
exit_button.grid(row=0, column=4, padx=2, pady=2)


# Resize the widgets dynamically
control_frame.grid_rowconfigure(0, weight=1)
for col in range(5):
    control_frame.grid_columnconfigure(col, weight=1)

# OpenCV Video Capture
CAMERA_SOURCE = 0
cap = cv2.VideoCapture(CAMERA_SOURCE)


def update_video_feed():
    """Update the video feed in the GUI and perform analysis."""
    global cap
    try:
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                # Perform ribbon analysis and log results
                thickness_mm = analyze_ribbon(frame)
                logging.debug(f"Ribbon Thickness: {thickness_mm:.2f} mm")

                # Update the thickness label dynamically
                thickness_label.config(text=f"Thickness: {thickness_mm:.2f} mm")

                # Resize and display frame
                frame = cv2.resize(frame, (video_label.winfo_width(), video_label.winfo_height()))
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame)
                imgtk = ImageTk.PhotoImage(image=img)
                video_label.imgtk = imgtk
                video_label.configure(image=imgtk)
    except Exception as e:
        logging.error(f"Error in video feed: {e}")

    video_label.after(10, update_video_feed)


# Start updating the video feed
update_video_feed()

# Mainloop
root.protocol("WM_DELETE_WINDOW", root.destroy)
root.mainloop()
cap.release()
