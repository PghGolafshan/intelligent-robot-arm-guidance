import torch
import os
from ultralytics import YOLO
import cv2

# Open webcam (0 = default, 1 = external camera)
cap = cv2.VideoCapture(0)

# Check if the webcam opened successfully
if not cap.isOpened():
    print("❌ Error: Cannot open webcam.")
    exit()

# Load YOLOv8 model
model = YOLO("C:\\Users\\Pegah\\Desktop\\yolo\\best.pt")

if __name__ == '__main__':
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Error: Failed to read frame.")
            break

        # Run YOLOv8 inference on the frame
        results = model(frame, stream=True)  # Make sure 'results' is iterable

        # Plot and display results
        for r in results:
            annotated_frame = r.plot()
            cv2.imshow("YOLOv8 Real-Time Detection", annotated_frame)
            break  # Only process first result per frame


        # Exit on ESC key
        if cv2.waitKey(1) == 27:
            break

# Release the webcam and destroy windows
cap.release()
cv2.destroyAllWindows()