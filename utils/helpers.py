import cv2
import socket

# Hand skeleton connections for Pure OpenCV drawing
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),    # Thumb
    (0, 5), (5, 6), (6, 7), (7, 8),    # Index
    (5, 9), (9, 10), (10, 11), (11, 12), # Middle
    (9, 13), (13, 14), (14, 15), (15, 16), # Ring
    (13, 17), (0, 17), (17, 18), (18, 19), (19, 20) # Pinky/Palm
]

def draw_landmarks_opencv(frame, landmarks):
    """Draws hand skeleton using pure OpenCV (No mp.solutions)."""
    h, w, _ = frame.shape
    for connection in HAND_CONNECTIONS:
        start_idx, end_idx = connection
        pt1 = (int(landmarks[start_idx].x * w), int(landmarks[start_idx].y * h))
        pt2 = (int(landmarks[end_idx].x * w), int(landmarks[end_idx].y * h))
        cv2.line(frame, pt1, pt2, (255, 255, 255), 2)
    for lm in landmarks:
        cx, cy = int(lm.x * w), int(lm.y * h)
        cv2.circle(frame, (cx, cy), 5, (0, 255, 120), -1)

def find_camera(max_index=5):
    """Scans for cameras, skipping common IR camera resolutions."""
    for i in [2, 3, 4, 5, 0, 1]: # Prioritize 0 and 2, check 1 (often IR) last
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                if width > 400: # Simple check: IR cameras are usually very low res
                    print(f"📷 Webcam found at index {i}")
                    return cap
            cap.release()
    return None

def get_ip_address():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except: return "127.0.0.1"
