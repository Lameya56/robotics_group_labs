import cv2
import numpy as np

def main():
    cap = cv2.VideoCapture("data/video.mp4")

    if not cap.isOpened():
        print("Error: Could not open video source.")
        return
    
    paused = False
    frame = None

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                print("End of stream or error.")
                break

        # HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 50, 80])
        upper_yellow = np.array([35, 255, 255])
        mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)

        mask = mask1

        # Noise Reduction
        kernel = np.ones((5, 5), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

        # Conntours
        contours, hierarchy = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        result = cv2.bitwise_and(frame, frame, mask=mask)

        tracked_frame = frame.copy()

        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = None, None

            # Create a copy for drawing
            cv2.drawContours(tracked_frame, [largest], -1, (0, 255, 0), 2)

            if cx and cy:
                cv2.circle(tracked_frame, (cx, cy), 5, (0, 0, 255), -1)

        # Display all frames
        cv2.imshow('Raw Frame - Press SPACE to pause, Q to quit', frame)
        cv2.imshow('HSV Frame', hsv)
        cv2.imshow("Mask", mask)
        cv2.imshow("Cleaned Mask", mask_clean)
        cv2.imshow("Segmented Object", result)
        cv2.imshow("Tracked Object", tracked_frame)

        key = cv2.waitKey(1) & 0xFF

        # Hit "Space" key to pause the video
        if key == ord(' '):
            paused = not paused
            if paused:
                print("Video paused - press SPACE to resume")
                # Optionally save the frame when paused
                print("Frame saved as 'captured_frame.jpg'")

        # Exit on 'q' key
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
