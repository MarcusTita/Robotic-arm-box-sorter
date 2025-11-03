import cv2
from pyzbar.pyzbar import decode
from pyzbar.pyzbar import ZBarSymbol
import numpy as np
import time
from collections import deque

class QRScanner:
    def __init__(self, camera_id=0, width=1920, height=1080, position_history_size=10):
        """
        Initialize the QR code speed tracker.
        
        Args:
            camera_id: Camera device ID (default 0)
            width: Frame width
            height: Frame height
            position_history_size: Number of positions to store for speed calculation
        """
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.position_history = deque(maxlen=position_history_size)
        self.timestamp_history = deque(maxlen=position_history_size)
        self.last_qr_data = None
        
    def calculate_center(self, bbox):
        """Calculate the center point of a bounding box."""
        if not bbox or len(bbox) != 4:
            return None
        
        x_coords = [p[0] for p in bbox]
        y_coords = [p[1] for p in bbox]
        center_x = sum(x_coords) / len(x_coords)
        center_y = sum(y_coords) / len(y_coords)
        
        return (center_x, center_y)
        
    def calculate_box_size(self, bbox):
        """Calculate the size (area) of a bounding box in pixels."""
        if not bbox or len(bbox) != 4:
            return 0
        x_coords = [p[0] for p in bbox]
        y_coords = [p[1] for p in bbox]
        
        width1 = np.sqrt((x_coords[1] - x_coords[0])**2 + (y_coords[1] - y_coords[0])**2)
        width2 = np.sqrt((x_coords[2] - x_coords[3])**2 + (y_coords[2] - y_coords[3])**2)
        avg_width = (width1 + width2) / 2
        height1 = np.sqrt((x_coords[3] - x_coords[0])**2 + (y_coords[3] - y_coords[0])**2)
        height2 = np.sqrt((x_coords[2] - x_coords[1])**2 + (y_coords[2] - y_coords[1])**2)
        avg_height = (height1 + height2) / 2
        return {
            "width": avg_width,
            "height": avg_height,
            "area": avg_width * avg_height
        }
    
    def calculate_speed(self):
        """Calculate speed from position history in pixels/frame."""
        if len(self.position_history) < 2:
            return 0.0, None, "px/frame"
        distances = []
        for i in range(1, len(self.position_history)):
            prev_pos = self.position_history[i-1]
            curr_pos = self.position_history[i]
            dx = curr_pos[0] - prev_pos[0]
            dy = curr_pos[1] - prev_pos[1]
            distance = np.sqrt(dx*dx + dy*dy)
            distances.append(distance)
        
        if not distances:
            return 0.0, None, "px/frame"
            
        # Calculate average speed in pixels per frame
        avg_speed_px = sum(distances) / len(distances)
            
        # Direction vector from first to last point (for overall movement direction)
        if len(self.position_history) >= 2:
            first_pos = self.position_history[0]
            last_pos = self.position_history[-1]
            dir_x = last_pos[0] - first_pos[0]
            dir_y = last_pos[1] - first_pos[1]
            
            # Normalize vector
            magnitude = np.sqrt(dir_x*dir_x + dir_y*dir_y)
            if magnitude > 0:
                dir_x /= magnitude
                dir_y /= magnitude
            direction = (dir_x, dir_y)
        else:
            direction = None
        return avg_speed_px, direction, "px/frame"
    
    def calibrate(self, known_qr_size_cm=7, height_difference_cm=11, display=True):
        """
        Calibrate the system to determine camera distance to conveyor belt and pixel:cm ratio.
        
        Args:
            known_qr_size_cm: The actual size of the QR code in centimeters
            height_difference_cm: Height difference when raising the QR code above the belt
            display: Whether to display the video feed during calibration
            
        Returns:
            Dictionary with calibration results (distance_to_belt, pixel_per_cm)
        """
        if height_difference_cm is None or height_difference_cm <= 0:
            print("Error: You must provide a positive height difference in centimeters")
            return None
            
        # Open the webcam
        cap = cv2.VideoCapture(self.camera_id, cv2.CAP_AVFOUNDATION)
        cap.set(3, self.width)
        cap.set(4, self.height)
        if not cap.isOpened():
            print("Failed to open camera")
            return None
        
        calibration_results = {
            "known_qr_size": known_qr_size_cm,
            "height_difference": height_difference_cm,
            "pixels_per_cm_belt": None,
            "pixels_per_cm_raised": None,
            "change_in_pixels_per_cm": None,
            "bot_edge_y": None,
            "top_edge_y": None
        }
        
        # Step 1: Measure QR code on the close edge of the belt
        belt_measurement, qr_bbox = self._capture_qr_measurement(
            cap, 
            "Place QR code on the conveyor belt edge far from the arm, then press any key...",
            "QR on belt",
            display
        )
        if not belt_measurement:
            cap.release()
            cv2.destroyAllWindows()
            print("Failed to detect QR code on the belt")
            return None
        print(f"QR on belt size: {belt_measurement['width']:.2f} x {belt_measurement['height']:.2f} pixels")
        if qr_bbox and len(qr_bbox) >= 4:
            sorted_points = sorted(qr_bbox, key=lambda point: point[1])
            bottommost_points = sorted_points[2:]
            avg_y = (bottommost_points[0][1] + bottommost_points[1][1]) / 2
            print(f"Average y-coordinate of bottom-most points: {avg_y:.2f}")
            calibration_results["bot_edge_y"] = avg_y
        else:
            print(len(qr_bbox))

        # Step 2: Measure QR code on the far edge of the belt
        belt_measurement2, qr_bbox2 = self._capture_qr_measurement(
            cap, 
            "Place QR code on the conveyor belt edge close to the arm, then press any key...",
            "QR on belt",
            display
        )
        if not belt_measurement2:
            cap.release()
            cv2.destroyAllWindows()
            print("Failed to detect QR code on the belt")
            return None
        if qr_bbox2 and len(qr_bbox2) >= 4:
            sorted_points = sorted(qr_bbox2, key=lambda point: point[1])
            topmost_points = sorted_points[:2]
            avg_y = (topmost_points[0][1] + topmost_points[1][1]) / 2
            print(f"Average y-coordinate of top-most points: {avg_y:.2f}")
            calibration_results["top_edge_y"] = avg_y
        
        # Step 3: Measure QR code at predefined height above belt
        raised_message = f"Raise QR code {height_difference_cm}cm above belt, then press any key..."
        raised_measurement, _ = self._capture_qr_measurement(
            cap, 
            raised_message,
            "QR raised",
            display
        )
        if not raised_measurement:
            cap.release()
            cv2.destroyAllWindows()
            print("Failed to detect QR code at raised position")
            return None
        print(f"QR at raised position size: {raised_measurement['width']:.2f} x {raised_measurement['height']:.2f} pixels")
        cap.release()
        if display:
            cv2.destroyAllWindows()
        
        # Calculate calibration values:
        belt_size_px = (belt_measurement['width'] + belt_measurement['height']+ 3) / 2
        raised_size_px = (raised_measurement['width'] + raised_measurement['height']) / 2
        
        calibration_results["pixels_per_cm_belt"] = belt_size_px / known_qr_size_cm
        calibration_results["pixels_per_cm_raised"] = raised_size_px / known_qr_size_cm
        calibration_results["change_in_pixels_per_cm"] = (calibration_results["pixels_per_cm_raised"]-calibration_results["pixels_per_cm_belt"])/height_difference_cm
        
        print("\nCalibration Results:")
        print(f"Pixel to cm ratio at belt level: {calibration_results['pixels_per_cm_belt']} pixels/cm")
        print(f"Pixel to cm ratio at {height_difference_cm}cm raised level: {calibration_results['pixels_per_cm_raised']} pixels/cm")
        print(f"Change in pixels per cm: {calibration_results['change_in_pixels_per_cm']} pixels/cm")
        return calibration_results
    
    def _capture_qr_measurement(self, cap, prompt_message, window_name, display):
        """Helper method to capture a QR code measurement during calibration."""
        print(prompt_message)
        if display:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        # Wait for key press while showing camera feed
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                return None
            if display:
                cv2.putText(frame, prompt_message, (30, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.imshow(window_name, frame)
                if cv2.waitKey(1) & 0xFF != 0xFF:
                    break
            else:
                time.sleep(0.5)
                break
        
        # Capture QR code
        qr_detected = False
        measurement = None
        attempts = 0
        max_attempts = 30
        while not qr_detected and attempts < max_attempts:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                return None
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            qr_codes = decode(gray, symbols=[ZBarSymbol.QRCODE])

            for qr in qr_codes:
                pts = qr.polygon
                if len(pts) == 4:
                    qr_detected = True
                    qr_bbox = [(p.x, p.y) for p in pts]
                    center = self.calculate_center(qr_bbox)
                    measurement = self.calculate_box_size(qr_bbox)
                    if display:
                        # Draw detected QR code
                        cv2.polylines(frame, [np.array(qr_bbox, dtype=np.int32)], True, (0, 255, 0), 3)
                        size_text = f"Size: {measurement['width']:.1f} x {measurement['height']:.1f} px"
                        cv2.putText(frame, size_text, (int(center[0]), int(center[1]) - 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(frame, "QR code detected!", (30, 60), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        cv2.imshow(window_name, frame)
                        cv2.waitKey(500)
                    break
            if not qr_detected and display:
                cv2.putText(frame, "Searching for QR code...", (30, 60), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
                cv2.imshow(window_name, frame)
                cv2.waitKey(2)
            attempts += 1
        if not qr_detected:
            print("Could not detect QR code")
            return None
        return measurement, qr_bbox
    
    def scan_and_track(self, display=True, exit_key='q', min_frames=10, 
                      reliable_frames=10, min_speed=0.5, max_variance=0.2,
                      max_frames_without_detection=3, fps=60, calibration=None):
        """
        Scan for QR codes and track their speed, returning once reliable data is collected.
        
        Args:
            display: Whether to display the video feed
            exit_key: Key to press to exit the loop
            min_frames: Minimum frames to collect before calculating speed
            reliable_frames: Number of consecutive frames needed for reliable tracking
            min_speed: Minimum speed (px/frame) to consider the object as moving
            max_variance: Maximum allowed variance in speed measurements
            max_frames_without_detection: Maximum allowed consecutive frames without detection
                                         before resetting the consecutive detection counter
            fps: Frames per second of camera being used
            calibration: Calibration results from the calibrate() method (optional)
            
        Returns:
            Dictionary with QR data, speed, direction, final position, and box size information
        """
        # Open the webcam
        cap = cv2.VideoCapture(self.camera_id, cv2.CAP_AVFOUNDATION)
        cap.set(3, self.width)
        cap.set(4, self.height)
        if not cap.isOpened():
            print("Failed to open camera")
            return None
            
        results = {
            "qr_data": None,
            "speed": 0.0,
            "direction": None,
            "unit": "px/frame",
            "final_position": None,
            "box_size": None,
            "height_above_belt": None
        }
        self.position_history.clear()
        self.timestamp_history.clear()
        
        # Tracking reliability metrics
        consecutive_detections = 0
        frames_without_detection = 0
        max_consecutive_detections = 0
        speed_measurements = []
        tracking_stable = False
        box_sizes = []
        last_known_center = None
        last_known_bbox = None
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Failed to grab frame")
                    break
                current_time = time.time()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                qr_codes = decode(gray, symbols=[ZBarSymbol.QRCODE])
                qr_detected = False
                
                # Process QR codes
                for qr in qr_codes:
                    qr_detected = True
                    frames_without_detection = 0
                    qr_data = qr.data.decode('utf-8')
                    results["qr_data"] = qr_data
                    self.last_qr_data = qr_data
                    pts = qr.polygon
                    
                    if len(pts) == 4:
                        qr_bbox = [(p.x, p.y) for p in pts]
                        last_known_bbox = qr_bbox  # Save this for when detection fails
                        center = self.calculate_center(qr_bbox)
                        last_known_center = center  # Save this for when detection fails
                        box_size = self.calculate_box_size(qr_bbox)
                        box_sizes.append(box_size)
                        results["final_position"] = center
                        if box_sizes:
                            avg_width = sum(bs["width"] for bs in box_sizes) / len(box_sizes)
                            avg_height = sum(bs["height"] for bs in box_sizes) / len(box_sizes)
                            avg_area = sum(bs["area"] for bs in box_sizes) / len(box_sizes)
                            results["box_size"] = {
                                "width": round(avg_width, 2),
                                "height": round(avg_height, 2),
                                "area": round(avg_area, 2)
                            }
                            if calibration and calibration.get("pixels_per_cm_belt"):
                                # width_cm = avg_width / calibration["pixels_per_cm"]
                                # height_cm = avg_height / calibration["pixels_per_cm"]
                                # results["real_world_size"] = {
                                #     "width": round(width_cm, 2),
                                #     "height": round(height_cm, 2),
                                #     "area": round(width_cm * height_cm, 2)
                                # }
                                # known_qr_size_cm = calibration.get("known_qr_size_cm", width_cm)
                                # qr_avg_size_px = (avg_width + avg_height) / 2
                                # distance_to_qr = (calibration["focal_length_px"] * known_qr_size_cm) / qr_avg_size_px
                                # height_above_belt = calibration["distance_to_belt_cm"] - distance_to_qr
                                # results["height_above_belt"] = round(height_above_belt, 2)
                                box_size_px = (avg_width+avg_height)/2
                                print(f"QR size: {avg_width} x {avg_height} pixels")
                                height_above_belt = (box_size_px/calibration["known_qr_size"]-calibration["pixels_per_cm_belt"])/calibration["change_in_pixels_per_cm"]
                                # if height_above_belt > calibration["height_difference"]:
                                #     height_above_belt = (height_above_belt+calibration["height_difference"])/2
                                results["height_above_belt"] = round(height_above_belt, 2)
                        if center:
                            # results["height_above_belt"] += (8*(800-center[1])/800) - 4
                            if results["height_above_belt"] < 0:
                                results["height_above_belt"] = 4
                            self.position_history.append(center)
                            self.timestamp_history.append(current_time)
                            if len(self.position_history) >= min_frames:
                                speed, direction, unit = self.calculate_speed()
                                results["speed"] = round(speed, 2)
                                results["direction"] = direction
                                results["unit"] = unit
                                # if calibration and calibration.get("pixels_per_cm_belt") and len(self.timestamp_history) > 1:
                                #     box_pixels_per_cm = box_size_px/calibration["known_qr_size"]
                                #     # speed_cm = fps * speed / box_pixels_per_cm
                                #     speed_cm = fps / (self.timestamp_history[-1]-self.timestamp_history[0]) * speed / box_pixels_per_cm
                                #     results["real_world_speed"] = round(speed_cm, 2)
                                #     results["real_world_speed_unit"] = "cm/s"
                                speed_measurements.append(speed)
                                if len(speed_measurements) > reliable_frames:
                                    speed_measurements.pop(0)
                        
                        # Draw QR bounding box
                        if display:
                            cv2.polylines(frame, [np.array(qr_bbox, dtype=np.int32)], True, (0, 255, 0), 3)
                            if center and "speed" in results:
                                # Draw speed info
                                speed_text = f"Speed: {results['speed']} {results['unit']}"
                                cv2.putText(frame, speed_text, (int(center[0]), int(center[1]) - 20), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                                
                                # Draw QR data
                                data_text = f"QR: {qr_data}"
                                cv2.putText(frame, data_text, (int(center[0]), int(center[1]) - 50), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                                if "box_size" in results:
                                    size_text = f"Size: {results['box_size']['width']:.1f} x {results['box_size']['height']:.1f} px"
                                    cv2.putText(frame, size_text, (int(center[0]), int(center[1]) - 110), 
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                                    if "height_above_belt" in results and results["height_above_belt"] is not None:
                                        height_text = f"Height: {results['height_above_belt']:.1f} cm"
                                        cv2.putText(frame, height_text, (int(center[0]), int(center[1]) - 140), 
                                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # No current detection but we have a recent detection
                if not qr_detected and last_known_center and frames_without_detection < max_frames_without_detection:
                    frames_without_detection += 1
                    if display and last_known_bbox:
                        cv2.polylines(frame, [np.array(last_known_bbox, dtype=np.int32)], True, (0, 165, 255), 3)
                        center_x, center_y = last_known_center
                        cv2.putText(frame, "Last known position", (int(center_x), int(center_y) - 140), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                        cv2.putText(frame, f"Frames without detection: {frames_without_detection}/{max_frames_without_detection}", 
                                    (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                elif not qr_detected:
                    frames_without_detection += 1
                
                # Update consecutive detection counter
                if qr_detected:
                    consecutive_detections += 1
                    max_consecutive_detections = max(max_consecutive_detections, consecutive_detections)
                elif frames_without_detection >= max_frames_without_detection:
                    consecutive_detections = 0
                
                # Display tracking statistics
                if display:
                    count_text = f"Consecutive detections: {consecutive_detections}/{reliable_frames} (Max: {max_consecutive_detections})"
                    cv2.putText(frame, count_text, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
                    # Status of tracking stability
                    if tracking_stable:
                        stability_text = "Tracking: STABLE"
                        color = (0, 255, 0)
                    else:
                        stability_text = f"Tracking: ACQUIRING"
                        color = (0, 165, 255)
                    cv2.putText(frame, stability_text, (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    
                    # Movement trail
                    if len(self.position_history) >= 2:
                        points = [np.array([int(x), int(y)]) for x, y in self.position_history]
                        for i in range(1, len(points)):
                            cv2.line(frame, tuple(points[i-1]), tuple(points[i]), (255, 0, 0), 2)
                
                # Check tracking mean/variance/stability
                if (consecutive_detections >= reliable_frames or 
                    max_consecutive_detections >= reliable_frames*2) and len(speed_measurements) >= min(reliable_frames, len(self.position_history)):
                    if speed_measurements:
                        mean_speed = sum(speed_measurements) / len(speed_measurements)
                        if len(speed_measurements) > 1:
                            variance = sum((s - mean_speed) ** 2 for s in speed_measurements) / len(speed_measurements)
                            normalized_variance = variance / (mean_speed ** 2) if mean_speed > 0 else float('inf')
                            if max_consecutive_detections >= reliable_frames*2:
                                tracking_stable = (mean_speed >= min_speed * 0.5 and normalized_variance <= max_variance * 2)
                            else:
                                tracking_stable = (mean_speed >= min_speed and normalized_variance <= max_variance)
                            
                            # Return once stable tracking is achieved
                            if tracking_stable:
                                if calibration and calibration.get("pixels_per_cm_belt"):
                                    box_pixels_per_cm = box_size_px/calibration["known_qr_size"]
                                    # speed_cm = fps * speed / box_pixels_per_cm
                                    speed_cm = (abs(self.position_history[-1][0]-self.position_history[0][0])+abs(self.position_history[-1][1]-self.position_history[0][1])) / (self.timestamp_history[-1]-self.timestamp_history[0]) / box_pixels_per_cm - (results["height_above_belt"]/3)
                                    results["real_world_speed"] = round(speed_cm, 2)
                                    results["real_world_speed_unit"] = "cm/s"
                                if display:
                                    frame_copy = frame.copy()
                                    cv2.putText(frame_copy, "TRACKING STABLE - RETURNING", (30, 120), 
                                              cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                                    cv2.imshow('QR Code Scanner', frame_copy)
                                break
                
                # Display the frame
                if display:
                    cv2.imshow('QR Code Scanner', frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord(exit_key):
                        break
        finally:
            cap.release()
            if display:
                cv2.destroyAllWindows()
        return results
