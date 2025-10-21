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
            # Annotate the frame with detection results
            annotated_frame = r.plot()

            # Loop through each detection result
            for det in r.boxes.data:  # Detected boxes
                # Get the coordinates of the bounding box
                x1, y1, x2, y2 = map(int, det[:4])  # xmin, ymin, xmax, ymax

                # Calculate the center of the bounding box
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                # Draw a circle at the center
                cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 0, 255), -1)

                # Draw the coordinates near the center
                cv2.putText(annotated_frame, f"({center_x}, {center_y})", 
                            (center_x + 10, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                            (0, 0, 255), 2)

            # Display the frame with center annotation
            cv2.imshow("YOLOv8 Real-Time Detection", annotated_frame)
            break  # Only process the first result per frame

        # Exit on ESC key
        if cv2.waitKey(1) == 27:
            break

# Release the webcam and destroy windows
cap.release()
cv2.destroyAllWindows()
