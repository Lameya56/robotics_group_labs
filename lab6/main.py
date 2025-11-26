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

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
