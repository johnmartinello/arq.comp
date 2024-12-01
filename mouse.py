import cv2
import numpy as np

# Initialize video capture
cap = cv2.VideoCapture(0)

# Read the first frame
ret, frame1 = cap.read()
if not ret:
    print("Failed to grab frame")
    exit()

# Convert frame to grayscale and blur it
gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
gray1 = cv2.GaussianBlur(gray1, (21, 21), 0)

while True:
    # Read the next frame
    ret, frame2 = cap.read()
    if not ret:
        break

    # Convert frame to grayscale and blur it
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.GaussianBlur(gray2, (21, 21), 0)

    # Compute the absolute difference between the two frames
    diff = cv2.absdiff(gray1, gray2)

    # Threshold the difference image
    _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)

    # Dilate the thresholded image to fill in holes
    thresh = cv2.dilate(thresh, None, iterations=2)

    # Find contours on the thresholded image
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Optional: Filter out small contours
        filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]
        if filtered_contours:
            # Combine all contours into one
            all_contours = np.vstack(filtered_contours)
            # Get the bounding rectangle for the combined contours
            x, y, w, h = cv2.boundingRect(all_contours)
            # Draw the bounding rectangle
            cv2.rectangle(frame2, (x, y), (x + w, y + h), (0, 255, 0), 2)
    

    # Show the result
    cv2.imshow("Motion Tracking", frame2)

    # Update the previous frame
    gray1 = gray2.copy()

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
