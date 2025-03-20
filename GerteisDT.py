import configparser
import os
import cv2
import numpy as np
from opcua import Server, ua, Client
from simple_pid import PID
import tkinter as tk
from tkinter import Button, Label, Frame, Entry
from PIL import Image, ImageTk
import threading
import logging
import datetime
import time
import csv
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")


BASE_DIR = os.path.dirname(__file__)
VIDEO_DIR = os.path.join(BASE_DIR, "video_storage")

MAX_STORAGE_BYTES = 128 * 1024 * 1024 * 1024  # 128GB storage limit

is_recording = False
reference_frame = None
cap = None
output_file = None
thickness_px = None

last_valid_thickness = 0.0
process_running = False

if not os.path.exists(VIDEO_DIR):
    os.makedirs(VIDEO_DIR)

config_file_path = os.path.join(BASE_DIR, 'config.ini')

_config_parser = configparser.ConfigParser()

def load_config():
    _config_parser.read(config_file_path)
    true_density = float(_config_parser['CompactionSettings'].get('true_density', 1500.0))
    relaxation_multiplier = float(_config_parser['CompactionSettings'].get('relaxation_multiplier', 1.0))
    camera_distance = float(_config_parser['CompactionSettings'].get('camera_distance', 43.0))
    ribbon_width = float(_config_parser['CompactionSettings'].get('ribbon_width', 1.0))
    roller_speed = float(_config_parser['CompactionSettings'].get('roller_speed', 0.1))
    allowable_deviation = float(_config_parser['CompactionSettings'].get('allowable_deviation', 5.0))
    desired_solid_fraction = float(_config_parser['CompactionSettings'].get('desired_solid_fraction', 0.65))
    sensitivity = float(_config_parser['CompactionSettings'].get('sensitivity', 80.0))
    stabilization_frames = int(_config_parser['CompactionSettings'].get('stabilization_frames', 10))
    min_contour_area = int(_config_parser['CompactionSettings'].get('min_contour_area', 5000))
    roi_x = int(_config_parser['CompactionSettings'].get('roi_x', 300))
    roi_y = int(_config_parser['CompactionSettings'].get('roi_y', 200))
    roi_width = int(_config_parser['CompactionSettings'].get('roi_width', 400))
    roi_height = int(_config_parser['CompactionSettings'].get('roi_height', 300))
    roi_display_mode = _config_parser['CompactionSettings'].get('roi_display_mode', 'shade')
    base_color = int(_config_parser['CompactionSettings'].get('base_color', 128))  # Default mid-gray
    color_deviation = int(_config_parser['CompactionSettings'].get('color_deviation', 30))  # Default ±30

    return {
        'true_density': true_density,
        'relaxation_multiplier': relaxation_multiplier,
        'camera_distance': camera_distance,
        'ribbon_width': ribbon_width,
        'roller_speed': roller_speed,
        'allowable_deviation': allowable_deviation,
        'desired_solid_fraction': desired_solid_fraction,
        'sensitivity': sensitivity,
        'stabilization_frames': stabilization_frames,
        'min_contour_area': min_contour_area,
        'roi_x': roi_x,
        'roi_y': roi_y,
        'roi_width': roi_width,
        'roi_height': roi_height,
        'roi_display_mode': roi_display_mode,
        'base_color': base_color,
        'color_deviation': color_deviation
    }



def save_config(key, value):
    if 'CompactionSettings' not in _config_parser:
        _config_parser['CompactionSettings'] = {}
    _config_parser['CompactionSettings'][key] = str(value)

    with open(config_file_path, 'w') as f:
        _config_parser.write(f)

config = load_config()
# OPC UA Server
server = Server()
server.set_endpoint("opc.tcp://0.0.0.0:4840/freeopcua/server/")
idx = server.register_namespace("RibbonAnalysis")
compaction_obj = server.nodes.objects.add_object(idx, "CompactionSettings")

config_nodes = {}

for key, val in config.items():
    config_nodes[key] = compaction_obj.add_variable(idx, key.capitalize(), val, ua.VariantType.Float)
    config_nodes[key].set_writable()

config_nodes['thickness'] = compaction_obj.add_variable(idx, "RibbonThickness", 0.0, ua.VariantType.Float)
config_nodes['thickness'].set_writable()
config_nodes['density'] = compaction_obj.add_variable(idx, "RibbonDensity", 0.0, ua.VariantType.Float)
config_nodes['density'].set_writable()
config_nodes['throughput'] = compaction_obj.add_variable(idx, "Throughput", 0.0, ua.VariantType.Float)
config_nodes['throughput'].set_writable()
config_nodes['solid_fraction'] = compaction_obj.add_variable(idx, "RibbonSolidFraction", 0.0, ua.VariantType.Float)
config_nodes['solid_fraction'].set_writable()

server.start()
logging.info("Built-in OPC UA server started.")

# PID Controller Logic - not used
pid = PID(Kp=1, Ki=0.1, Kd=0.05, setpoint=config['desired_solid_fraction'])
pid.output_limits = (-config['allowable_deviation'], config['allowable_deviation'])
EXTERNAL_OPC_ENDPOINT = "opc.tcp://127.0.0.1:49321"
external_opcua_client = None
opc_connected = False


def connect_external_opc():# For connecting to the Gerteis
    global external_opcua_client, opc_connected
    try:
        external_opcua_client = Client(EXTERNAL_OPC_ENDPOINT)
        external_opcua_client.connect()
        logging.info(f"Connected to external OPC UA server at {EXTERNAL_OPC_ENDPOINT}")
        opc_connected = True
    except Exception as e:
        logging.error(f"Failed to connect to external OPC UA server: {e}")
        opc_connected = False

def disconnect_external_opc():
    global external_opcua_client, opc_connected
    if external_opcua_client:
        try:
            external_opcua_client.disconnect()
            logging.info("Disconnected from external OPC UA server.")
        except Exception as e:
            logging.error(f"Error disconnecting external OPC UA client: {e}")
    opc_connected = False
def browse_external_opc():
    if not opc_connected or not external_opcua_client:
        logging.warning("Cannot browse external OPC UA server: Not connected.")
        return []

    try:
        root_node = external_opcua_client.get_objects_node()
        parameters = root_node.get_children()
        parameter_info = []
        for param in parameters:
            browse_name = param.get_browse_name().Name
            parameter_info.append({"Node": param, "Name": browse_name})
            logging.info(f"Found parameter: {browse_name}")
        return parameter_info
    except Exception as e:
        logging.error(f"Error browsing external OPC UA server: {e}")
        return []

def read_external_opc_value(node):
    try:
        value = node.get_value()
        logging.info(f"Read value: {value} from node: {node}")
        return value
    except Exception as e:
        logging.error(f"Failed to read value from node: {e}")
        return "NoData"

stop_logging_thread = False
logging_thread = None
current_log_file = None


def start_data_logging():
    global stop_logging_thread, logging_thread, current_log_file
    if logging_thread and logging_thread.is_alive():
        logging.info("Data logging already running.")
        return

    if not current_log_file:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        current_log_file = os.path.join(BASE_DIR, f"data_log_{timestamp}.csv")
        logging.info(f"📝 Created new log file: {current_log_file}")

    # Resume logging
    stop_logging_thread = False
    logging_thread = threading.Thread(target=data_logging_loop, args=(current_log_file,), daemon=True)
    logging_thread.start()


def stop_data_logging():
    global stop_logging_thread
    stop_logging_thread = True
    if logging_thread is not None:
        logging_thread.join(timeout=2)
    logging.info("Data logging stopped.")


def data_logging_loop(log_file):
    logging.info(f"Logging data to {log_file}")

    try:
        with open(log_file, mode="a", newline="") as f:
            fieldnames = ["Timestamp"] + list(config_nodes.keys())
            writer = csv.DictWriter(f, fieldnames=fieldnames)

            if f.tell() == 0:
                writer.writeheader()

            while not stop_logging_thread:
                row = {"Timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}

                try:
                    thickness_val = config_nodes["thickness"].get_value()
                    if thickness_val is None:
                        logging.warning("Skipping data log entry (thickness is N/A).")
                        time.sleep(0.2)
                        continue
                    for var_name, var_node in config_nodes.items():
                        try:
                            row[var_name] = float(var_node.get_value())
                        except (ValueError, TypeError):
                            row[var_name] = "NaN"
                    writer.writerow(row)
                    f.flush()
                    logging.debug(f"Logged Data: {row}")

                except Exception as e:
                    logging.error(f"Error logging data: {e}")

                time.sleep(0.2)

    except Exception as e:
        logging.error(f"Error opening/writing CSV file: {e}")

    logging.info("🔚 Data logging loop stopped.")

def normalize_reference():
    global reference_frame, cap

    logging.info("Normalizing reference frame...")

    if cap is None or not cap.isOpened():
        logging.error("Camera is not initialized or not opened.")
        return

    roi_x, roi_y = config['roi_x'], config['roi_y']
    roi_width, roi_height = config['roi_width'], config['roi_height']

    stabilization_frames = config['stabilization_frames']
    captured_frames = []

    logging.info(f"Capturing {stabilization_frames} frames for reference stabilization in ROI...")

    for _ in range(stabilization_frames):
        ret, frame = cap.read()
        if not ret:
            logging.warning("Frame capture failed during normalization.")
            continue
        roi_frame = frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width].copy()
        gray_frame = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
        captured_frames.append(gray_frame)

        cv2.waitKey(50)

    if not captured_frames:
        logging.error("No valid frames were captured for normalization.")
        return
    reference_frame = np.median(captured_frames, axis=0).astype(np.uint8)

    logging.info("Reference frame successfully captured inside ROI.")




def stabilize_frame(current_frame, reference_frame):
    try:
        gray_current = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        gray_reference = reference_frame

        orb = cv2.ORB_create()
        kp1, des1 = orb.detectAndCompute(gray_current, None)
        kp2, des2 = orb.detectAndCompute(gray_reference, None)

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)

        if len(matches) > 10:  # Ensure we have enough matches for reliable transformation
            src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

            matrix, mask = cv2.estimateAffinePartial2D(src_pts, dst_pts, method=cv2.RANSAC)

            if matrix is not None:
                stabilized_frame = cv2.warpAffine(current_frame, matrix, (current_frame.shape[1], current_frame.shape[0]))
                return stabilized_frame

    except Exception as e:
        logging.error(f"Error stabilizing frame: {e}")

    return current_frame


def analyze_ribbon(roi_frame):
    global reference_frame, stop_logging_thread

    if reference_frame is None:
        logging.warning("No reference frame found. Stopping logging and showing 'N/A'.")
        thickness_label.config(text="Thickness: N/A")
        density_label.config(text="Density: N/A")
        sf_label.config(text="Solid Fraction: N/A")
        stop_data_logging()
        return None, roi_frame
    gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
    base_color = config['base_color']
    color_deviation = config['color_deviation']
    lower_bound = max(0, base_color - color_deviation)
    upper_bound = min(255, base_color + color_deviation)

    grayscale_mask = cv2.inRange(gray, lower_bound, upper_bound)
    diff = cv2.absdiff(reference_frame, gray)
    diff_filtered = cv2.bitwise_and(diff, diff, mask=grayscale_mask)

    sensitivity = config.get('sensitivity', 50)
    _, threshold = cv2.threshold(diff_filtered, sensitivity, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    object_detected = False
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > config['min_contour_area']:
            object_detected = True  # Object detected

            logging.debug(f"Largest contour area: {cv2.contourArea(largest_contour)}")
            rect = cv2.minAreaRect(largest_contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(roi_frame, [box], -1, (0, 255, 0), 2)  # Green Box
            roi_frame[threshold > 0] = [0, 255, 0]  # Green Highlight
            width, height = rect[1]
            object_width = min(width, height)

            if object_width > 0:
                thickness_mm = (
                    object_width
                    * (config['camera_distance'] / 1000)
                    * config['relaxation_multiplier']
                )
                density, solid_fraction, _ = compute_density_and_solid_fraction(thickness_mm)

                thickness_label.config(text=f"Thickness: {thickness_mm:.2f} mm" if thickness_mm is not None else "Thickness: N/A")
                density_label.config(text=f"Density: {density:.5f} g/mm³" if density is not None else "Density: N/A")
                sf_label.config(text=f"Solid Fraction: {solid_fraction:.3f}" if solid_fraction is not None else "Solid Fraction: N/A")

                if stop_logging_thread:
                    logging.info("Object detected. Resuming data logging.")
                    start_data_logging()

                config_nodes['thickness'].set_value(ua.Variant(thickness_mm, ua.VariantType.Float))
                config_nodes['density'].set_value(ua.Variant(density, ua.VariantType.Float))
                config_nodes['solid_fraction'].set_value(ua.Variant(solid_fraction, ua.VariantType.Float))
                # ✅ Return the Proces
                return thickness_mm, roi_frame

    if not object_detected:
        logging.warning("No valid contours detected. Stopping logging.")
        thickness_label.config(text="Thickness: N/A")
        density_label.config(text="Density: N/A")
        sf_label.config(text="Solid Fraction: N/A")
        stop_data_logging()

    return None, roi_frame

    logging.warning("No valid contours detected. Stopping logging and displaying 'N/A'.")
    thickness_label.config(text="Thickness: N/A")
    density_label.config(text="Density: N/A")
    sf_label.config(text="Solid Fraction: N/A")

    stop_logging_thread = True
    return None, roi_frame

is_recording = False

def start_recording():
    global is_recording, output_file
    is_recording = True
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_path = os.path.join(VIDEO_DIR, f"recording_{timestamp}.avi")

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
def start_process():
    global process_running
    process_running = True
    logging.info("Process started.")

def stop_process():
    global process_running
    process_running = False
    logging.info("Process stopped.")

def compute_throughput():

    try:
        throughput_str = throughput_var.get()
        if throughput_str:
            throughput = float(throughput_str)
            logging.info(f"Using operator-entered throughput: {throughput} g/s")
            return throughput
        throughput = config_nodes["throughput"].get_value()
        logging.info(f"Using OPC UA throughput: {throughput} g/s")
        return throughput

    except Exception as e:
        logging.error(f"Error retrieving throughput value: {e}")
        return 0.0


def compute_density_and_solid_fraction(thickness_mm):
    if thickness_mm is None or thickness_mm <= 0:
        return 0.0, 0.0, 0.0

    volume_flow = thickness_mm * config['ribbon_width'] * config['roller_speed']
    if volume_flow <= 0:
        return 0.0, 0.0, 0.0

    throughput = compute_throughput()
    actual_density = throughput / volume_flow if volume_flow > 0 else 0.0
    solid_fraction = actual_density / config['true_density'] if config['true_density'] > 0 else 0.0
    gap_correction = pid(solid_fraction)

    return actual_density, solid_fraction, gap_correction

root = tk.Tk()
screen_width = 1920
screen_height = 1080
root.geometry(f"{screen_width}x{screen_height}")

root.grid_rowconfigure(0, weight=3)
root.grid_rowconfigure(1, weight=1)
root.grid_columnconfigure(0, weight=1)

video_frame = Frame(root, bg="black", width=screen_width, height=int(screen_height * 0.75))
video_frame.grid(row=0, column=0, sticky="nsew")

control_frame = Frame(root, bg="lightgrey", width=screen_width, height=int(screen_height * 0.25))
control_frame.grid(row=1, column=0, sticky="nsew")

video_label = Label(video_frame, bg="black")
video_label.pack(fill="both", expand=True)

control_frame.grid_rowconfigure(0, weight=1)
control_frame.grid_rowconfigure(1, weight=1)
for c in range(8):
    control_frame.grid_columnconfigure(c, weight=1)

button_font_size = 12
thickness_label = Label(control_frame, text="Thickness: 0.00 mm",
                        font=("Helvetica", button_font_size), bg="lightgrey")
thickness_label.grid(row=0, column=0, padx=2, pady=2)

density_label = Label(control_frame, text="Density: 0.000 g/mm³",
                      font=("Helvetica", button_font_size), bg="lightgrey")
density_label.grid(row=0, column=1, padx=2, pady=2)

sf_label = Label(control_frame, text="Solid Fraction: 0.00",
                 font=("Helvetica", button_font_size), bg="lightgrey")
sf_label.grid(row=0, column=2, padx=2, pady=2)

normalize_button = Button(control_frame, text="Normalize", font=("Helvetica", button_font_size), command=normalize_reference)
normalize_button.grid(row=0, column=3, padx=2, pady=2)

start_proc_button = Button(control_frame, text="Start Process", font=("Helvetica", button_font_size), command=start_process)
start_proc_button.grid(row=0, column=4, padx=2, pady=2)

stop_proc_button = Button(control_frame, text="Stop Process", font=("Helvetica", button_font_size), command=stop_process)
stop_proc_button.grid(row=0, column=5, padx=2, pady=2)

start_button = Button(control_frame, text="Start Recording", font=("Helvetica", button_font_size), command=start_recording)
start_button.grid(row=0, column=6, padx=2, pady=2)

stop_button = Button(control_frame, text="Stop Recording", font=("Helvetica", button_font_size), command=stop_recording)
stop_button.grid(row=0, column=7, padx=2, pady=2)

# ROW 1: True Density & Relaxation Factor user input
Label(control_frame, text="True Density (g/mm³):", font=("Helvetica", button_font_size), bg="lightgrey").grid(row=1, column=0, padx=2, pady=2, sticky='e')
true_density_var = tk.StringVar(value=str(config['true_density']))
true_density_entry = Entry(control_frame, textvariable=true_density_var, font=("Helvetica", button_font_size), width=10)
true_density_entry.grid(row=1, column=1, padx=2, pady=2, sticky='w')

Label(control_frame, text="Relaxation Factor:", font=("Helvetica", button_font_size), bg="lightgrey").grid(row=1, column=2, padx=2, pady=2, sticky='e')
relax_factor_var = tk.StringVar(value=str(config['relaxation_multiplier']))
relax_factor_entry = Entry(control_frame, textvariable=relax_factor_var, font=("Helvetica", button_font_size), width=10)
relax_factor_entry.grid(row=1, column=3, padx=2, pady=2, sticky='w')

Label(control_frame, text="Throughput (g/s):", font=("Helvetica", button_font_size), bg="lightgrey").grid(row=2, column=0, padx=2, pady=2, sticky='e')
throughput_var = tk.StringVar(value=str(config.get('throughput', 0.0)))
throughput_entry = Entry(control_frame, textvariable=throughput_var, font=("Helvetica", button_font_size), width=10)
throughput_entry.grid(row=2, column=1, padx=2, pady=2, sticky='w')

Label(control_frame, text="Ribbon Width (mm):", font=("Helvetica", button_font_size), bg="lightgrey").grid(row=2, column=2, padx=2, pady=2, sticky='e')
ribbon_width_var = tk.StringVar(value=str(config.get('ribbon_width', 1.0)))
ribbon_width_entry = Entry(control_frame, textvariable=ribbon_width_var, font=("Helvetica", button_font_size), width=10)
ribbon_width_entry.grid(row=2, column=3, padx=2, pady=2, sticky='w')

Label(control_frame, text="Roll Speed (mm/s):", font=("Helvetica", button_font_size), bg="lightgrey").grid(row=2, column=4, padx=2, pady=2, sticky='e')
roll_speed_var = tk.StringVar(value=str(config.get('roller_speed', 0.1)))
roll_speed_entry = Entry(control_frame, textvariable=roll_speed_var, font=("Helvetica", button_font_size), width=10)
roll_speed_entry.grid(row=2, column=5, padx=2, pady=2, sticky='w')


exit_button = Button(control_frame, text="Exit", font=("Helvetica", button_font_size),
                     command=lambda: on_closing())
exit_button.grid(row=1, column=7, padx=2, pady=2)

def on_true_density_change(*args):
    val_str = true_density_var.get()
    try:
        val = float(val_str)
        config['true_density'] = val
        config_nodes['true_density'].set_value(ua.Variant(val, ua.VariantType.Float))
        save_config('true_density', val)
    except ValueError:
        pass

def on_relax_factor_change(*args):
    val_str = relax_factor_var.get()
    try:
        val = float(val_str)
        config['relaxation_multiplier'] = val
        config_nodes['relaxation_multiplier'].set_value(ua.Variant(val, ua.VariantType.Float))
        save_config('relaxation_multiplier', val)
    except ValueError:
        pass

true_density_var.trace_add("write", on_true_density_change)
relax_factor_var.trace_add("write", on_relax_factor_change)

def on_throughput_change(*args):
    try:
        val = float(throughput_var.get())
        config['throughput'] = val
        config_nodes['throughput'].set_value(ua.Variant(val, ua.VariantType.Float))
        save_config('throughput', val)
    except ValueError:
        logging.warning("Invalid throughput value entered.")

def on_ribbon_width_change(*args):
    try:
        val = float(ribbon_width_var.get())
        config['ribbon_width'] = val
        config_nodes['ribbon_width'].set_value(ua.Variant(val, ua.VariantType.Float))
        save_config('ribbon_width', val)
    except ValueError:
        pass

def on_roll_speed_change(*args):
    try:
        val = float(roll_speed_var.get())
        config['roller_speed'] = val
        config_nodes['roller_speed'].set_value(ua.Variant(val, ua.VariantType.Float))
        save_config('roller_speed', val)
    except ValueError:
        pass

throughput_var.trace_add("write", on_throughput_change)
ribbon_width_var.trace_add("write", on_ribbon_width_change)
roll_speed_var.trace_add("write", on_roll_speed_change)


config_nodes['ribbon_width'] = compaction_obj.add_variable(idx, "RibbonWidth", 1.0, ua.VariantType.Float)
config_nodes['ribbon_width'].set_writable()

config_nodes['roller_speed'] = compaction_obj.add_variable(idx, "RollerSpeed", 0.1, ua.VariantType.Float)
config_nodes['roller_speed'].set_writable()

def update_video_feed():
    global cap, last_valid_thickness

    try:
        if cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                logging.warning("Frame capture failed.")
                return

            processed_frame = frame.copy()
            roi_x, roi_y = config['roi_x'], config['roi_y']
            roi_width, roi_height = config['roi_width'], config['roi_height']
            display_mode = config['roi_display_mode']

            roi_x = max(0, min(roi_x, frame.shape[1] - 1))
            roi_y = max(0, min(roi_y, frame.shape[0] - 1))
            roi_width = max(1, min(roi_width, frame.shape[1] - roi_x))
            roi_height = max(1, min(roi_height, frame.shape[0] - roi_y))

            overlay = processed_frame.copy()
            overlay[:, :] = (0, 255, 255)

            overlay[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width] = frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]

            alpha = 0.5
            cv2.addWeighted(overlay, alpha, processed_frame, 1 - alpha, 0, processed_frame)

            roi_frame = frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width].copy()

            thickness_mm, roi_with_detection = analyze_ribbon(roi_frame)

            processed_frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width] = roi_with_detection

            label_w, label_h = video_label.winfo_width(), video_label.winfo_height()
            display_frame = resize_preserving_aspect(processed_frame, label_w, label_h)

            display_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(display_frame)
            imgtk = ImageTk.PhotoImage(image=img)

            video_label.imgtk = imgtk
            video_label.configure(image=imgtk)

            thickness_label.config(text=f"Thickness: {thickness_mm:.2f} mm")

    except Exception as e:
        logging.error(f"Error in video feed: {e}")
    video_label.after(50, update_video_feed)



def resize_preserving_aspect(frame, target_w, target_h):

    import numpy as np

    h, w = frame.shape[:2]
    aspect_frame = w / h
    if target_h == 0:
        return frame

    aspect_label = target_w / target_h

    if aspect_frame > aspect_label:
        new_w = target_w
        new_h = int(new_w / aspect_frame)
    else:
        new_h = target_h
        new_w = int(new_h * aspect_frame)

    new_w = max(1, new_w)
    new_h = max(1, new_h)

    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)

    display_image = np.zeros((target_h, target_w, 3), dtype=np.uint8)

    y_off = (target_h - new_h) // 2
    x_off = (target_w - new_w) // 2
    display_image[y_off:y_off+new_h, x_off:x_off+new_w] = resized

    return display_image

def on_closing():
    global cap, is_recording, process_running
    logging.info("Application closing.")

    process_running = False
    stop_recording()

    stop_data_logging()
    disconnect_external_opc()

    if cap:
        cap.release()

    server.stop()
    root.destroy()

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
if not cap.isOpened():
    print("Error: Could not open camera.")

fourcc_mjpeg = cv2.VideoWriter_fourcc(*'MJPG')
cap.set(cv2.CAP_PROP_FOURCC, fourcc_mjpeg)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

cap.set(cv2.CAP_PROP_FPS, 30)

connect_external_opc()

start_data_logging()

update_video_feed()

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
cap.release()
