import cv2
import numpy as np

def nothing(x):
    pass

# Open webcam
cap = cv2.VideoCapture(0)

# Create window for trackbars
cv2.namedWindow("Trackbars")

# Create 6 trackbars (HSV min/max)
cv2.createTrackbar("H Min", "Trackbars", 35, 179, nothing)
cv2.createTrackbar("H Max", "Trackbars", 85, 179, nothing)
cv2.createTrackbar("S Min", "Trackbars", 50, 255, nothing)
cv2.createTrackbar("S Max", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("V Min", "Trackbars", 50, 255, nothing)
cv2.createTrackbar("V Max", "Trackbars", 255, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get current positions of trackbars
    h_min = cv2.getTrackbarPos("H Min", "Trackbars")
    h_max = cv2.getTrackbarPos("H Max", "Trackbars")
    s_min = cv2.getTrackbarPos("S Min", "Trackbars")
    s_max = cv2.getTrackbarPos("S Max", "Trackbars")
    v_min = cv2.getTrackbarPos("V Min", "Trackbars")
    v_max = cv2.getTrackbarPos("V Max", "Trackbars")

    # ========================
    # SELECT COLOR (Uncomment one)
    # ========================

    # GREEN
    # lower = np.array([35, 50, 50])
    # upper = np.array([85, 255, 255])

    # BLUE
    lower = np.array([90, 50, 50])
    upper = np.array([130, 255, 255])

    # YELLOW
    # lower = np.array([20, 100, 100])
    # upper = np.array([35, 255, 255])

    # ORANGE
    # lower = np.array([10, 100, 100])
    # upper = np.array([25, 255, 255])

    # PURPLE
    # lower = np.array([130, 50, 50])
    # upper = np.array([160, 255, 255])

    # WHITE
    # lower = np.array([0, 0, 200])
    # upper = np.array([179, 40, 255])

    # BLACK
    # lower = np.array([0, 0, 0])
    # upper = np.array([179, 255, 40])


    mask = cv2.inRange(hsv, lower, upper)

    # Noise removal
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, "Detected", (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Green Detection", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
