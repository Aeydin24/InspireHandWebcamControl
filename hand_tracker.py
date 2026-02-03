# hand_tracker.py - With clickable calibration buttons and fixed thumb
import cv2
import mediapipe as mp
import socket
import json
import numpy as np
import math
import os

# UDP Configuration
UDP_IP = "127.0.0.1"
UDP_PORT = 5065

# Initialize MediaPipe
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Calibration file
CALIBRATION_FILE = "hand_calibration.json"


class Button:
    """Simple clickable button for OpenCV window"""
    def __init__(self, x, y, width, height, text, color=(100, 100, 100), hover_color=(150, 150, 150)):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.text = text
        self.color = color
        self.hover_color = hover_color
        self.is_hovered = False
    
    def contains(self, px, py):
        return self.x <= px <= self.x + self.width and self.y <= py <= self.y + self.height
    
    def draw(self, frame):
        color = self.hover_color if self.is_hovered else self.color
        cv2.rectangle(frame, (self.x, self.y), (self.x + self.width, self.y + self.height), color, -1)
        cv2.rectangle(frame, (self.x, self.y), (self.x + self.width, self.y + self.height), (200, 200, 200), 2)
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        text_size = cv2.getTextSize(self.text, font, font_scale, thickness)[0]
        text_x = self.x + (self.width - text_size[0]) // 2
        text_y = self.y + (self.height + text_size[1]) // 2
        cv2.putText(frame, self.text, (text_x, text_y), font, font_scale, (255, 255, 255), thickness)


class HandCalibration:
    """Stores min/max values for each finger measurement"""
    def __init__(self):
        self.finger_ranges = {
            'pinky': [0.5, 2.2],
            'ring': [0.5, 2.2],
            'middle': [0.5, 2.2],
            'index': [0.5, 2.2],
            'thumb_bend': [0.3, 1.5],
            'thumb_rotation': [0.15, 0.85]
        }
        self.load_calibration()
    
    def save_calibration(self):
        try:
            with open(CALIBRATION_FILE, 'w') as f:
                json.dump(self.finger_ranges, f, indent=2)
            print(f"Calibration saved to {CALIBRATION_FILE}")
            return True
        except Exception as e:
            print(f"Failed to save calibration: {e}")
            return False
    
    def load_calibration(self):
        try:
            if os.path.exists(CALIBRATION_FILE):
                with open(CALIBRATION_FILE, 'r') as f:
                    loaded = json.load(f)
                    self.finger_ranges.update(loaded)
                print(f"Calibration loaded from {CALIBRATION_FILE}")
                return True
        except Exception as e:
            print(f"Failed to load calibration: {e}")
        return False
    
    def update_min(self, finger_name, value):
        if finger_name in self.finger_ranges:
            self.finger_ranges[finger_name][0] = value
    
    def update_max(self, finger_name, value):
        if finger_name in self.finger_ranges:
            self.finger_ranges[finger_name][1] = value
    
    def normalize(self, finger_name, raw_value):
        if finger_name not in self.finger_ranges:
            return 500
        
        min_val, max_val = self.finger_ranges[finger_name]
        
        if abs(max_val - min_val) < 0.001:
            return 500
        
        normalized = (raw_value - min_val) / (max_val - min_val)
        normalized = max(0, min(1, normalized))
        return int(normalized * 1000)
    
    def reset_defaults(self):
        self.finger_ranges = {
            'pinky': [0.5, 2.2],
            'ring': [0.5, 2.2],
            'middle': [0.5, 2.2],
            'index': [0.5, 2.2],
            'thumb_bend': [0.3, 1.5],
            'thumb_rotation': [0.15, 0.85]
        }


class AngleSmoother:
    def __init__(self, alpha=0.4):
        self.alpha = alpha
        self.smoothed = [500] * 6
        self.initialized = False
    
    def smooth(self, angles):
        if not self.initialized:
            self.smoothed = list(angles)
            self.initialized = True
            return [int(a) for a in self.smoothed]
        
        for i in range(len(angles)):
            self.smoothed[i] = self.alpha * angles[i] + (1 - self.alpha) * self.smoothed[i]
        
        return [int(a) for a in self.smoothed]
    
    def reset(self):
        self.initialized = False


def get_landmark_array(landmarks, idx):
    lm = landmarks[idx]
    return np.array([lm.x, lm.y, lm.z])


def calculate_finger_raw_value(landmarks, mcp_idx, pip_idx, dip_idx, tip_idx, wrist_idx=0):
    """
    Calculate raw finger extension value using distance ratio.
    Higher value = more extended/open
    """
    wrist = get_landmark_array(landmarks, wrist_idx)
    mcp = get_landmark_array(landmarks, mcp_idx)
    tip = get_landmark_array(landmarks, tip_idx)
    
    tip_to_wrist = np.linalg.norm(tip - wrist)
    mcp_to_wrist = np.linalg.norm(mcp - wrist)
    
    if mcp_to_wrist < 0.001:
        return 1.0
    
    return tip_to_wrist / mcp_to_wrist


def calculate_thumb_bend_raw(landmarks):
    """
    Calculate thumb bend using distance from thumb tip to palm center.
    Higher value = more extended/open
    """
    thumb_tip = get_landmark_array(landmarks, 4)
    index_mcp = get_landmark_array(landmarks, 5)
    middle_mcp = get_landmark_array(landmarks, 9)
    wrist = get_landmark_array(landmarks, 0)
    
    palm_center = (index_mcp + middle_mcp + wrist) / 3
    thumb_to_palm = np.linalg.norm(thumb_tip - palm_center)
    
    hand_size = np.linalg.norm(middle_mcp - wrist)
    if hand_size < 0.001:
        return 1.0
    
    return thumb_to_palm / hand_size


def calculate_thumb_rotation_raw(landmarks):
    """
    Calculate thumb rotation/abduction.
    Higher value = thumb more away from index finger
    """
    thumb_tip = get_landmark_array(landmarks, 4)
    index_mcp = get_landmark_array(landmarks, 5)
    pinky_mcp = get_landmark_array(landmarks, 17)
    
    thumb_to_index_2d = math.sqrt(
        (thumb_tip[0] - index_mcp[0])**2 + 
        (thumb_tip[1] - index_mcp[1])**2
    )
    
    palm_width = math.sqrt(
        (index_mcp[0] - pinky_mcp[0])**2 + 
        (index_mcp[1] - pinky_mcp[1])**2
    )
    
    if palm_width < 0.001:
        return 0.5
    
    return thumb_to_index_2d / palm_width


# Global variables for mouse callback
mouse_x, mouse_y = 0, 0
mouse_clicked = False

def mouse_callback(event, x, y, flags, param):
    global mouse_x, mouse_y, mouse_clicked
    mouse_x, mouse_y = x, y
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_clicked = True


def main():
    global mouse_x, mouse_y, mouse_clicked
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    calibration = HandCalibration()
    smoother = AngleSmoother(alpha=0.35)
    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    hands = mp_hands.Hands(
        model_complexity=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.6,
        max_num_hands=1
    )
    
    # Create window and set mouse callback
    window_name = 'Hand Tracker - Click buttons to calibrate'
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)
    
    # Create buttons
    btn_width = 130
    btn_height = 40
    btn_y = 10
    
    btn_open = Button(640 - btn_width - 10, btn_y, btn_width, btn_height, 
                      "Calibrate OPEN", (0, 120, 0), (0, 180, 0))
    btn_closed = Button(640 - btn_width - 10, btn_y + btn_height + 10, btn_width, btn_height,
                        "Calibrate CLOSED", (0, 0, 120), (0, 0, 180))
    btn_save = Button(640 - btn_width - 10, btn_y + (btn_height + 10) * 2, btn_width, btn_height,
                      "Save Calibration", (120, 80, 0), (180, 120, 0))
    btn_reset = Button(640 - btn_width - 10, btn_y + (btn_height + 10) * 3, btn_width, btn_height,
                       "Reset Defaults", (80, 80, 80), (120, 120, 120))
    btn_mirror = Button(640 - btn_width - 10, btn_y + (btn_height + 10) * 4, btn_width, btn_height,
                        "Toggle Mirror", (80, 80, 80), (120, 120, 120))
    
    buttons = [btn_open, btn_closed, btn_save, btn_reset, btn_mirror]
    
    print("=" * 60)
    print("Hand Tracker with Calibration")
    print("=" * 60)
    print("")
    print("CALIBRATION INSTRUCTIONS:")
    print("")
    print("1. Hold your hand with ALL fingers EXTENDED (spread open)")
    print("   - Thumb should be stretched OUT, away from palm")
    print("   - Click 'Calibrate OPEN'")
    print("")
    print("2. Make a tight FIST")
    print("   - Thumb should be bent IN, tucked against fingers")
    print("   - Click 'Calibrate CLOSED'")
    print("")
    print("3. Click 'Save Calibration' to remember settings")
    print("=" * 60)
    
    mirror_mode = True
    message = ""
    message_time = 0
    message_color = (0, 255, 255)
    
    finger_names = ['pinky', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rotation']
    current_raw_values = {}
    
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            continue
        
        if mirror_mode:
            frame = cv2.flip(frame, 1)
        
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb_frame)
        
        data = {
            "detected": False,
            "angles": [1000, 1000, 1000, 1000, 1000, 500],
            "landmarks": []
        }
        
        hand_detected = False
        
        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            landmarks = hand_landmarks.landmark
            hand_detected = True
            
            # Calculate raw values
            raw_pinky = calculate_finger_raw_value(landmarks, 17, 18, 19, 20)
            raw_ring = calculate_finger_raw_value(landmarks, 13, 14, 15, 16)
            raw_middle = calculate_finger_raw_value(landmarks, 9, 10, 11, 12)
            raw_index = calculate_finger_raw_value(landmarks, 5, 6, 7, 8)
            raw_thumb_bend = calculate_thumb_bend_raw(landmarks)
            raw_thumb_rotation = calculate_thumb_rotation_raw(landmarks)
            
            current_raw_values = {
                'pinky': raw_pinky,
                'ring': raw_ring,
                'middle': raw_middle,
                'index': raw_index,
                'thumb_bend': raw_thumb_bend,
                'thumb_rotation': raw_thumb_rotation
            }
            
            # Normalize using calibration
            pinky_norm = calibration.normalize('pinky', raw_pinky)
            ring_norm = calibration.normalize('ring', raw_ring)
            middle_norm = calibration.normalize('middle', raw_middle)
            index_norm = calibration.normalize('index', raw_index)
            thumb_bend_norm = calibration.normalize('thumb_bend', raw_thumb_bend)
            thumb_rotation_norm = calibration.normalize('thumb_rotation', raw_thumb_rotation)
            
            raw_angles = [pinky_norm, ring_norm, middle_norm, index_norm, thumb_bend_norm, thumb_rotation_norm]
            smoothed_angles = smoother.smooth(raw_angles)
            
            data["detected"] = True
            data["angles"] = smoothed_angles
            
            for lm in landmarks:
                data["landmarks"].append({"x": lm.x, "y": lm.y, "z": lm.z})
            
            # Draw hand landmarks
            mp_drawing.draw_landmarks(
                frame, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style()
            )
            
            # Display angle values
            y_pos = 30
            display_names = ["Pinky", "Ring", "Middle", "Index", "ThumbBend", "ThumbRot"]
            
            for i, (name, angle) in enumerate(zip(display_names, smoothed_angles)):
                raw_val = list(current_raw_values.values())[i]
                
                if angle < 200:
                    value_color = (0, 0, 255)  # Red = closed
                elif angle > 800:
                    value_color = (0, 255, 0)  # Green = open
                else:
                    value_color = (0, 200, 255)  # Orange = middle
                
                cv2.putText(frame, f"{name}: {angle}", (10, y_pos), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, value_color, 2)
                cv2.putText(frame, f"({raw_val:.2f})", (130, y_pos), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
                
                # Draw bar
                bar_x = 185
                bar_width = int(angle / 1000 * 80)
                cv2.rectangle(frame, (bar_x, y_pos - 12), (bar_x + 80, y_pos + 2), (50, 50, 50), -1)
                cv2.rectangle(frame, (bar_x, y_pos - 12), (bar_x + bar_width, y_pos + 2), value_color, -1)
                cv2.rectangle(frame, (bar_x, y_pos - 12), (bar_x + 80, y_pos + 2), (100, 100, 100), 1)
                
                y_pos += 24
            
            # Show calibration info
            y_pos += 10
            cv2.putText(frame, "Calibration:", (10, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            y_pos += 15
            for fname in ['pinky', 'thumb_bend']:
                min_v, max_v = calibration.finger_ranges[fname]
                cv2.putText(frame, f"  {fname}: {min_v:.2f}-{max_v:.2f}", (10, y_pos), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
                y_pos += 14
        
        # Send data via UDP
        json_data = json.dumps(data)
        sock.sendto(json_data.encode(), (UDP_IP, UDP_PORT))
        
        # Update button hover states
        for btn in buttons:
            btn.is_hovered = btn.contains(mouse_x, mouse_y)
        
        # Draw buttons
        for btn in buttons:
            btn.draw(frame)
        
        # Handle button clicks
        current_time = cv2.getTickCount() / cv2.getTickFrequency()
        
        if mouse_clicked:
            mouse_clicked = False
            
            if btn_open.contains(mouse_x, mouse_y):
                if current_raw_values:
                    for fname in finger_names:
                        if fname in current_raw_values:
                            calibration.update_max(fname, current_raw_values[fname])
                    message = "OPEN hand calibrated!"
                    message_color = (0, 255, 0)
                    message_time = current_time
                    smoother.reset()
                    print("\nOpen hand calibration captured:")
                    for fname in finger_names:
                        print(f"  {fname}: max = {calibration.finger_ranges[fname][1]:.3f}")
                else:
                    message = "No hand detected!"
                    message_color = (0, 0, 255)
                    message_time = current_time
            
            elif btn_closed.contains(mouse_x, mouse_y):
                if current_raw_values:
                    for fname in finger_names:
                        if fname in current_raw_values:
                            calibration.update_min(fname, current_raw_values[fname])
                    message = "CLOSED fist calibrated!"
                    message_color = (0, 255, 0)
                    message_time = current_time
                    smoother.reset()
                    print("\nClosed fist calibration captured:")
                    for fname in finger_names:
                        print(f"  {fname}: min = {calibration.finger_ranges[fname][0]:.3f}")
                else:
                    message = "No hand detected!"
                    message_color = (0, 0, 255)
                    message_time = current_time
            
            elif btn_save.contains(mouse_x, mouse_y):
                if calibration.save_calibration():
                    message = "Calibration SAVED!"
                    message_color = (0, 255, 255)
                else:
                    message = "Save FAILED!"
                    message_color = (0, 0, 255)
                message_time = current_time
            
            elif btn_reset.contains(mouse_x, mouse_y):
                calibration.reset_defaults()
                if os.path.exists(CALIBRATION_FILE):
                    try:
                        os.remove(CALIBRATION_FILE)
                    except:
                        pass
                message = "Reset to defaults!"
                message_color = (0, 200, 255)
                message_time = current_time
                smoother.reset()
                print("\nCalibration reset to defaults")
            
            elif btn_mirror.contains(mouse_x, mouse_y):
                mirror_mode = not mirror_mode
                message = f"Mirror: {'ON' if mirror_mode else 'OFF'}"
                message_color = (200, 200, 200)
                message_time = current_time
        
        # Display status
        status = "Hand Detected" if hand_detected else "No Hand - Show your hand!"
        status_color = (0, 255, 0) if hand_detected else (0, 0, 255)
        cv2.putText(frame, status, (10, frame.shape[0] - 15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        
        # Show message
        if message and (current_time - message_time) < 3.0:
            cv2.putText(frame, message, (10, frame.shape[0] - 45),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, message_color, 2)
        
        cv2.imshow(window_name, frame)
        
        # Handle keyboard
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('m'):
            mirror_mode = not mirror_mode
            message = f"Mirror: {'ON' if mirror_mode else 'OFF'}"
            message_time = current_time
        elif key == ord('1') and current_raw_values:
            for fname in finger_names:
                if fname in current_raw_values:
                    calibration.update_max(fname, current_raw_values[fname])
            message = "OPEN hand calibrated!"
            message_color = (0, 255, 0)
            message_time = current_time
            smoother.reset()
        elif key == ord('2') and current_raw_values:
            for fname in finger_names:
                if fname in current_raw_values:
                    calibration.update_min(fname, current_raw_values[fname])
            message = "CLOSED fist calibrated!"
            message_color = (0, 255, 0)
            message_time = current_time
            smoother.reset()
        elif key == ord('s'):
            calibration.save_calibration()
            message = "Calibration SAVED!"
            message_time = current_time
        elif key == ord('r'):
            calibration.reset_defaults()
            message = "Reset to defaults!"
            message_time = current_time
            smoother.reset()
    
    cap.release()
    cv2.destroyAllWindows()
    sock.close()
    print("\nHand tracker stopped.")


if __name__ == "__main__":
    main()
