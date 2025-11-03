# main.py
import time
import serial
import math
from kinematics import Kinematics
from vision import QRScanner
import numpy as np
import cv2

def predict_future_position(results, camera_to_robot_distance_x=25, robot_y_offset=13, robot_z_offset=11, calibration=None):
    """
    Predict where the QR code will be after a specified time.
    
    Args:
        results: Dictionary from scan_and_track containing position, speed, and direction
        camera_to_robot_distance: Distance in cm from camera to robot arm base
        calibration: Webcam calibration results
        
    Returns:
        (x_time, y, z) tuple in robot's reference frame
        x_time equals number of seconds it takes for the box to reach the robot's idle position
    """
    if "real_world_speed" in results and results["real_world_speed"] is not None and results["height_above_belt"] is not None:
        pos_x, pos_y = results["final_position"]
        print(f"POS_X: {pos_x}")
        # robot_x_time = max(0, (camera_to_robot_distance_x / results["real_world_speed"] - 0.7)/2)
        robot_x_time = max(0, (camera_to_robot_distance_x / results["real_world_speed"] - pos_x/1000)/2)
        robot_y_cm = (pos_y-calibration["top_edge_y"])/calibration["pixels_per_cm_belt"]+robot_y_offset
        robot_z = results["height_above_belt"] - robot_z_offset
        return (robot_x_time, robot_y_cm, robot_z)
    print("Cannot predict real-world position: calibration data missing")
    return None

def l1_deg_to_voltage(degree):
    # Convert from degrees (0-360) to voltage (0-5) for lift1
    return round(((degree+12) / 360) * 5,2)

def l2_deg_to_voltage(degree):
    # Convert from degrees (0-360) to voltage (0-5) for lift2
    return round(((170-degree) / 360) * 5,2)

def safe_sleep(arduino, seconds):
    """Sleep without closing Arduino connection"""
    start_time = time.time()
    while time.time() - start_time < seconds:
        if arduino.in_waiting:
            data = arduino.readline().decode().strip()
            print("Arduino:", data)
        time.sleep(0.01)

# Send 5 comma-separated commands (turret angle, lift1 angle, lift2 angle, servo (never changed), vacuum)
# Changes: don't send servo value, curently 0-180 degrees corresponds to 0-4 command
def main():
    L1 = 20
    L2 = 20
    kin = Kinematics(L1=L1, L2=L2) # Initialize kinematics class with arm lengths (in cm)
    arduino_port = '/dev/tty.usbmodem2101'
    baud_rate = 115200
    
    scanner = QRScanner(camera_id=0, width=1280, height=720, position_history_size=15)
    print("Starting calibration...")
    print("Place QR code on conveyor belt, then raise it when prompted")
    calibration = scanner.calibrate(known_qr_size_cm=7, height_difference_cm=11, display=True)
    if not calibration:
        print("Calibration failed.")
        return
    print("Calibration successful.")
    
    # default position (arbitrary)
    turret_voltage_straight = 0.42
    theta1_default = 108
    theta2_default = 78
    default_y, default_z = kin.forward_kinematics(theta1_default, theta2_default)
    l1_voltage_default = l1_deg_to_voltage(theta1_default)
    l2_voltage_default = l2_deg_to_voltage(theta2_default)
    default_command = f"{l1_voltage_default},{l2_voltage_default},{turret_voltage_straight},0,0\n"
    try:
        arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
        safe_sleep(arduino,2)
        arduino.flushInput()
        arduino.write('enable\n'.encode())
        print("PC connected to Arduino!")
        safe_sleep(arduino,2)
        arduino.write(default_command.encode())
        print(f"Sent default command to Arduino: {default_command.strip()}")
    except serial.SerialException as e:
        print("Failed to connect to Arduino:", e)
        return

    try:
        # Main operational loop
        while True:
            print("\nScanning for QR codes...")
            results = scanner.scan_and_track(
                display=True,
                min_frames=5,
                reliable_frames=10,
                min_speed=5,
                max_variance=0.3,
                max_frames_without_detection=3,
                fps=60,
                calibration=calibration
            )
            if not results or not results["qr_data"]:
                print("No QR code detected. Trying again...")
                continue
            print(f"QR Data: {results['qr_data']}")
            print(f"Speed: {results['speed']} {results['unit']}")
            if "real_world_speed" in results and results["real_world_speed"] is not None:
                print(f"Real-world speed: {results['real_world_speed']} cm/s")
            if results["final_position"]:
                pos_x, pos_y = results["final_position"]
                print(f"Current Position (pixels): ({pos_x:.2f}, {pos_y:.2f})")
            if "height_above_belt" in results and results["height_above_belt"] is not None:
                print(f"Height above belt: {results['height_above_belt']:.2f} cm")
    
            future_pos = predict_future_position(results, camera_to_robot_distance_x=25, robot_y_offset=13, robot_z_offset=11, calibration=calibration)
            if future_pos:
                robot_x_time, robot_y, robot_z = future_pos
                try:
                    gripper_cm = 6
                    raise_cm = 10
                    robot_z += gripper_cm

                    # Out of reach target
                    if robot_y > math.sqrt((L1 + L2)**2 - robot_z**2) - 2:
                        print("Target is far")
                        robot_z += .2
                        robot_y = math.sqrt((L1 + L2)**2 - robot_z**2) - 2
                        print(robot_y,robot_z)
        
                    robot_z_hover = (default_z+robot_z)/2
                    robot_y_hover = (default_y*2+robot_y)/3

                    robot_z_raise = robot_z + raise_cm
                    if robot_y > math.sqrt((L1 + L2)**2 - robot_z_raise**2) - 2:
                        print("Adjusting robot_y_raise")
                    robot_y_raise = min(robot_y, math.sqrt((L1 + L2)**2 - robot_z_raise**2) - 2)
                    print(f"Robot raise: ({robot_y_raise:.2f}, {robot_z_raise:.2f})")

                    print(f"Robot_x_time: {robot_x_time:.2f} s")

                    # Wait time normalization
                    # if robot_x_time>1:
                    #     robot_x_time *= robot_x_time/(robot_x_time+.4)
                    # elif robot_x_time>.7:
                    #     robot_x_time *= robot_x_time/(robot_x_time+.2)

                    # Calculate arm angles using kinematics
                    theta1_deg_hover, theta2_deg_hover = kin.inverse_kinematics(robot_y_hover, robot_z_hover)
                    theta1_deg_lower, theta2_deg_lower = kin.inverse_kinematics(robot_y, robot_z)
                    theta1_deg_lift, theta2_deg_lift = kin.inverse_kinematics((robot_y+robot_y_raise)/2, (robot_z + robot_z_raise)/2)
                    theta1_deg_raised, theta2_deg_raised = kin.inverse_kinematics(robot_y_raise, robot_z_raise)
                    print(f"Robot arm hover angles: θ1={theta1_deg_hover:.2f}°, θ2={theta2_deg_hover:.2f}°")
                    print(f"Robot arm grab angles: θ1={theta1_deg_lower:.2f}°, θ2={theta2_deg_lower:.2f}°")
                    print(f"Robot arm lift angles: θ1={theta1_deg_raised:.2f}°, θ2={theta2_deg_raised:.2f}°")
                    print()

                    # Convert from degrees to voltage (0-5), lift2 actuator is inverted and goes from 0-4
                    theta1_command_hover = l1_deg_to_voltage(theta1_deg_hover)
                    theta2_command_hover = l2_deg_to_voltage(theta2_deg_hover)
                    theta1_command_lower = l1_deg_to_voltage(theta1_deg_lower)
                    theta2_command_lower = l2_deg_to_voltage(theta2_deg_lower)
                    theta1_command_lift = l1_deg_to_voltage(theta1_deg_lift)
                    theta2_command_lift = l2_deg_to_voltage(theta2_deg_lift)
                    theta1_command_raised = l1_deg_to_voltage(theta1_deg_raised)
                    theta2_command_raised = l2_deg_to_voltage(theta2_deg_raised)
                    
                    bin_x = 25
                    bin_deg1, bin_deg2 = kin.inverse_kinematics(bin_x, min(math.sqrt((L1+L2)**2-bin_x**2)-2,robot_z_hover))
                    turret_voltage_bin = None
                    if int(results["qr_data"])%2 == 0:
                        turret_voltage_bin = 1.5
                    else:
                        turret_voltage_bin = 2.3
                    theta1_command_bin = l1_deg_to_voltage(bin_deg1)
                    theta2_command_bin = l2_deg_to_voltage(bin_deg2)
                    arduino.flushInput()
                    safe_sleep(arduino,.2)

                    # Send command to Arduino (lift1_angle, lift2_angle, turret, servo angle, vacuum on/off)
                    command = f"{theta1_command_hover:.2f},{theta2_command_hover:.2f},{turret_voltage_straight},0,1\n"
                    arduino.write(command.encode())
                    print(f"Sent hover command to Arduino: {command.strip()}")
                    safe_sleep(arduino,robot_x_time)
                    # safe_sleep(arduino,robot_x_time)

                    command = f"{theta1_command_lower:.2f},{theta2_command_lower:.2f},{turret_voltage_straight},0,1\n"
                    arduino.write(command.encode())
                    print(f"Sent pickup command to Arduino: {command.strip()}")
                    safe_sleep(arduino,.4)

                    command = f"{theta1_command_lift:.2f},{theta2_command_lift:.2f},{turret_voltage_straight},0,1\n"
                    arduino.write(command.encode())
                    print(f"Sent slight lift command to Arduino: {command.strip()}")
                    safe_sleep(arduino,.2)

                    command = f"{theta1_command_raised:.2f},{theta2_command_raised:.2f},{turret_voltage_straight},0,1\n"
                    arduino.write(command.encode())
                    print(f"Sent raise command to Arduino: {command.strip()}")
                    safe_sleep(arduino,.4)

                    command = f"{theta1_command_bin:.2f},{theta2_command_bin:.2f},{turret_voltage_bin},0,1\n"
                    arduino.write(command.encode())
                    print(f"Sent bin command to Arduino: {command.strip()}")
                    safe_sleep(arduino,1.7)

                    command = f"{theta1_command_bin:.2f},{theta2_command_bin:.2f},{turret_voltage_bin},0,0\n"
                    arduino.write(command.encode())
                    print(f"Sent bin drop command to Arduino: {command.strip()}")
                    safe_sleep(arduino,.4)

                    arduino.write("disable\n".encode())
                    safe_sleep(arduino,.1)
                    arduino.write("enable\n".encode())
                    safe_sleep(arduino,.1)

                    command = default_command
                    arduino.write(command.encode())
                    print(f"Sent default command to Arduino: {command.strip()}")

                except Exception as e:
                    print(f"Kinematics calculation error: {e}")
            else:
                print("Unable to predict future position.")
            safe_sleep(arduino,2)
    except KeyboardInterrupt:
        print("\nInterrupted by user. Shutting down...")
    
    finally:
        arduino.close()
        print("Arduino connection closed.")

if __name__ == "__main__":
    main()
