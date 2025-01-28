import cv2 
import os


video_path = r'./data/Pipe2.avi'
output_dir = r'./amb_Sequences/output/'
frame_count = 0

os.makedirs(output_dir, exist_ok=True)
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Could not read video")
    exit()





while True:
    ret, frame = cap.read() # If ret is True, it means the frame was successfully read.

    if not ret:
        break

    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    filename = os.path.join(output_dir, f'frame_{frame_count:05d}.jpg') # 04: Pad the number with zeros to make it at least 4 digits long.
    cv2.imwrite(filename, rgb_frame)
    frame_count += 1

cap.release()
print("success")
