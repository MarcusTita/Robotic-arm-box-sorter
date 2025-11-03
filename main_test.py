import time
import serial
from kinematics import Kinematics
from vision import QRScanner

def predict_future_position(results, camera_to_robot_distance_x=3, robot_y_offset=13, robot_z_offset=13, calibration=None):
    if "real_world_speed" in results and results["real_world_speed"] is not None and results["height_above_belt"] is not None:
        pos_x, pos_y = results["final_position"]
        robot_x_time = (camera_to_robot_distance_x + pos_x / (calibration["pixels_per_cm_belt"] + calibration["change_in_pixels_per_cm"] * results["height_above_belt"])) / results["real_world_speed"]
        robot_y_cm = (pos_y - calibration["top_edge_y"]) / calibration["pixels_per_cm_belt"] + robot_y_offset
        robot_z = results["height_above_belt"] - robot_z_offset
        return (robot_x_time, robot_y_cm, robot_z)
    print("Cannot predict real-world position: calibration data missing")
    return None

def l1_deg_to_voltage(degree):
    return ((degree + 12) / 360) * 5

def l2_deg_to_voltage(degree):
    return ((170 - degree) / 360) * 5

def send_sequence(arduino, sequence):
    get_time = time.monotonic
    current_step = 0
    next_time = get_time()

    while current_step < len(sequence):
        now = get_time()
        if now >= next_time:
            step = sequence[current_step]
            command = step["command"]()
            arduino.write(command.encode())
            print(f"Sent {step['desc']} command to Arduino: {command.strip()}")
            next_time = now + step["delay"]
            current_step += 1
        time.sleep(0.01)

def main():
    kin = Kinematics(L1=20, L2=20)
    arduino_port = '/dev/tty.usbmodem2101'
    baud_rate = 115200

    try:
        arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
        
        arduino.flushInput()
        sequenceStart = [
                {"delay": 0.1, "command": lambda: "enable\n", "desc": "enable"},
                {"delay": 1.5, "command": lambda: f"1.61,1.22,.3,0,0\n", "desc": "lift"},
                {"delay": 0.5, "command": lambda: "disable\n", "desc": "bin move"},
            ]

        send_sequence(arduino, sequenceStart)
    except serial.SerialException as e:
        print("Failed to connect to Arduino:", e)
        return

    scanner = QRScanner(camera_id=0, width=1280, height=720, position_history_size=15)
    calibration = scanner.calibrate(known_qr_size_cm=7, height_difference_cm=11, display=True)
    if not calibration:
        print("Calibration failed.")
        return

    try:
        while True:
            results = scanner.scan_and_track(display=True, min_frames=5, reliable_frames=10, min_speed=5, max_variance=0.3, max_frames_without_detection=3, fps=60, calibration=calibration)
            if not results or not results["qr_data"]:
                print("No QR code detected. Trying again...")
                continue

            future_pos = predict_future_position(results, camera_to_robot_distance_x=5, robot_y_offset=13, calibration=calibration)
            if not future_pos:
                print("Unable to predict future position.")
                continue
            
            
            robot_x_time, robot_y, robot_z = future_pos
            gripper_cm = 8
            raised_hover_cm = 5
            robot_z += gripper_cm

            if robot_x_time > 1:
                robot_x_time /= 2
            elif robot_x_time > 0.7:
                robot_x_time /= 1.5

            theta1_deg, theta2_deg = kin.inverse_kinematics(robot_y, robot_z + raised_hover_cm)
            theta1_deg_lower, theta2_deg_lower = kin.inverse_kinematics(robot_y, robot_z)
            theta1_deg_raised, theta2_deg_raised = kin.inverse_kinematics(robot_y, robot_z + raised_hover_cm * 3)

            theta1_command = l1_deg_to_voltage(theta1_deg)
            theta2_command = l2_deg_to_voltage(theta2_deg)

            theta1_command_lower = l1_deg_to_voltage(theta1_deg_lower)
            theta2_command_lower = l2_deg_to_voltage(theta2_deg_lower)

            theta1_command_raised = l1_deg_to_voltage(theta1_deg_raised)
            theta2_command_raised = l2_deg_to_voltage(theta2_deg_raised)
            
            bin2_x = 20
            bin_hover_y = 16
            bin_deg1, bin_deg2 = kin.inverse_kinematics(bin2_x, bin_hover_y)
            theta1_command_bin = l1_deg_to_voltage(bin_deg1)
            theta2_command_bin = l2_deg_to_voltage(bin_deg2)
            arduino.flushInput()
            sequence = [
                {"delay": .01, "command": lambda: f"enable\n", "desc": "hover"},
                {"delay": .3, "command": lambda: f"1.61,1.22,.3,0,1\n", "desc": "vac on"},
                {"delay": .4, "command": lambda: f"1.73,1,.6,0,1\n", "desc": "go down"},
                {"delay": .2, "command": lambda: f"1.6,1.2,.9,0,1\n", "desc": "go up slighlty"},
                {"delay": .3, "command": lambda: f"1.5,1.4,1.2,0,1\n", "desc": "go up more"},
                {"delay": .5, "command": lambda: f"1.3,1.5,1.5,0,1\n", "desc": "up and over"},
                {"delay": .5, "command": lambda: f"1.6,1.6,1.5,0,1\n", "desc": "goto bin"},
                {"delay": .2, "command": lambda: f"1.6,1.6,1.5,0,0\n", "desc": "bin drop"},
                {"delay": 0.2, "command": lambda: f"disable\n", "desc": "disable"},
                {"delay": .01, "command": lambda: f"enable\n", "desc": "hover"},
                {"delay": 2, "command": lambda: f"1.61,1.22,.3,0,0\n", "desc": "return to hover"},
                {"delay": 0.5, "command": lambda: f"disable\n", "desc": "disable"}, 
            ]
            send_sequence(arduino, sequence)

    except KeyboardInterrupt:
        print("\nInterrupted by user. Shutting down...")
    finally:
        arduino.close()
        print("Arduino connection closed.")

if __name__ == "__main__":
    main()
