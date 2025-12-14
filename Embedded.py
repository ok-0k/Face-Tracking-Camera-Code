import cv2
import mediapipe as mp
import serial
import time
import sys

try:
    ser = serial.Serial('COM6', 115200, timeout=0.1)
    time.sleep(2)
except:
    print("Arduino not found so sad")
    sys.exit()

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
W = 640 
H = 480
cap.set(3, W)
cap.set(4, H)
mp_face = mp.solutions.face_detection.FaceDetection(0.5)
curr_p, curr_t = 90.0, 90.0
last_p, last_t = 90, 90
P_MIN, P_MAX = 10, 170
T_MIN, T_MAX = 60, 150

print("Running... press q to quit")

while True:
    ret, frame = cap.read()
    if not ret: break 
    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = mp_face.process(rgb)

    if results.detections:
        d = results.detections[0]
        bbox = d.location_data.relative_bounding_box
        cx = int((bbox.xmin + bbox.width /2)*W)
        cy = int((bbox.ymin + bbox.height /2)*H)
        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
        speed = 0.03 
        deadzone = 40
        err_x = cx - (W // 2)
        err_y = cy - (H // 2)
        if abs(err_x) > deadzone:
            curr_p += err_x * speed
        if abs(err_y) > deadzone:
            curr_t -= err_y * speed
        curr_p = max(P_MIN, min(P_MAX, curr_p))
        curr_t = max(T_MIN, min(T_MAX, curr_t))
        if abs(int(curr_p) - last_p) > 1 or abs(int(curr_t) - last_t) > 1:
            cmd = f"{int(curr_p)},{int(curr_t)}\n"
            try:
                ser.write(cmd.encode())
                if ser.in_waiting: 
                    ser.read_all()
            except:
                pass
            
            last_p, last_t = int(curr_p), int(curr_t)
    cv2.line(frame, (W//2, 0), (W//2, H), (255, 0, 0), 1)
    cv2.line(frame, (0, H//2), (W, H//2), (255, 0, 0), 1)
    cv2.imshow('cam', frame)
    if cv2.waitKey(1) == ord('q'): break
cap.release()
cv2.destroyAllWindows()
ser.close()