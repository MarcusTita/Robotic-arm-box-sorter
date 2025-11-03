import cv2
from pyzbar.pyzbar import decode
from vision import QRScanner
import numpy as np

if __name__ == "__main__":
    scanner = QRScanner(camera_id=0, width=1920, height=1080, position_history_size=15)

    # Calibrate the QR height/size
    calibration = None
    calibration = scanner.calibrate(known_qr_size_cm=7, height_difference_cm=11)

    results = scanner.scan_and_track(
        display=True,
        min_frames=5,
        reliable_frames=10,
        min_speed=0.5,
        max_variance=0.2
    )
    
    if results and results["qr_data"]:
        print(f"\nResults:")
        print(f"QR Data: {results['qr_data']}")
        print(f"Speed: {results['speed']} {results['unit']}")
        if results["final_position"]:
            print(f"Final Position: ({results['final_position'][0]:.2f}, {results['final_position'][1]:.2f})")
        if "real_world_speed" in results and results["real_world_speed"] is not None:
            print(f"Real-world speed: {results['real_world_speed']} {results['real_world_speed_unit']}")
        if results["direction"]:
            print(f"Direction: ({results['direction'][0]:.2f}, {results['direction'][1]:.2f})")
        if results["box_size"]:
            print(f"Box Size (pixels): {results['box_size']['width']:.2f} × {results['box_size']['height']:.2f} px")
            print(f"Box Area (pixels): {results['box_size']['area']:.2f} px²")
        if results["real_world_size"]:
            print(f"Real-world size: {results['real_world_size']['width']:.2f} × {results['real_world_size']['height']:.2f} cm")
            print(f"Real-world area: {results['real_world_size']['area']:.2f} cm²")
        if results["height_above_belt"] is not None:
            print(f"Height above conveyor belt: {results['height_above_belt']:.2f} cm")
    else:
        print("No QR code detected")
