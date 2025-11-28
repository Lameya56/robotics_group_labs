import cv2
import numpy as np

def main():
    # 1. Open video capture (0 = default camera, or path to video file)
    cap = cv2.VideoCapture("data/video.mp4")  # change to "data/video.mp4" if using a file

    if not cap.isOpened():
        print("Error: Could not open video source.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("End of stream or error.")
            break

        # TODO: call your processing function here
        # processed_frame, control_command = process_frame(frame)

        # For now, just show the raw frame:
        cv2.imshow("Raw Frame", frame)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.imshow("HSV", hsv)

        lower1 = np.array([20, 100, 100])
        upper1 = np.array([30, 255, 255])
        mask1 = cv2.inRange(hsv, lower1, upper1)

        mask = mask1
        
        kernel = np.ones((5, 5), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)
        contours, hierarchy = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.imshow("Mask", mask)

        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow("Segmented Object", result)

         if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = None, None

            # Create a copy for drawing
            tracked_frame = frame.copy()
            cv2.drawContours(tracked_frame, [largest], -1, (0, 255, 0), 2)

            if cx and cy:
                cv2.circle(tracked_frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.imshow("Tracked Object", tracked_frame)

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
