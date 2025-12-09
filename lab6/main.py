import cv2
import numpy as np
import time
import csv
import matplotlib.pyplot as plt
import os

def main():
    # --- Model Configuration ---
    TARGET_CLASS_ID = 5  # class ID for 'bottle'
    CONF_THRESH = 0.5
    prototxt_path = "models/deploy.prototxt"
    model_path = "models/mobilenet_iter_73000.caffemodel"
    
    # Load the neural network
    net = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)
    # --- Configuration ---
    video_path = "data/orange_bright.mp4"
    test_name = "orange_bright"
    save_annotated_video = True   # set False if you don't want an output video
    annotated_filename = f"annotated_output_{test_name}.mp4"
    
    # Color tracking outputs
    results_csv_color = f"results_log_color_{test_name}.csv"
    cx_plot_png_color = f"cx_over_time_color_{test_name}.png"
    
    # MobileNet detection outputs
    results_csv_mobilenet = f"results_log_mobilenet_{test_name}.csv"
    cx_plot_png_mobilenet = f"cx_over_time_mobilenet_{test_name}.png"

    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("Error: Could not open video source.")
        return

    # Get video properties for writer
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0 or np.isnan(fps):
        fps = 30.0  # fallback
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Video writer (optional)
    if save_annotated_video:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(annotated_filename, fourcc, fps, (width, height))
    else:
        writer = None

    paused = False
    frame = None

    # Task 4 logging variables
    frame_idx = 0
    log_color = []  # Color tracking: (frame_idx, timestamp, cx, cy, area, command)
    log_mobilenet = []  # MobileNet detection: (frame_idx, timestamp, cx_det, cy_det, confidence, command_det)
    cx_history = []
    command_history = []

    # area threshold to consider detection valid (tune for your resolution)
    area_min = 200

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                print("End of stream or error.")
                break

        # compute image center & deadband (do this after we have a frame)
        height, width, _ = frame.shape
        center_x = width // 2
        deadband = int(0.10 * width)  # 10% of frame width; tune as needed

        # Detection: MobileNet-based object detection
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            frame,
            scalefactor=0.007843,    # 1/127.5
            size=(300, 300),
            mean=127.5
        )
        net.setInput(blob)
        detections = net.forward()
        
        cx_det, cy_det = None, None
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence < CONF_THRESH:
                continue

            class_id = int(detections[0, 0, i, 1])
            if class_id != TARGET_CLASS_ID:
                continue

            # Scale bounding box
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (x1, y1, x2, y2) = box.astype("int")

            cx_det = (x1 + x2) // 2
            cy_det = (y1 + y2) // 2

            # Draw box + centroid
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.circle(frame, (cx_det, cy_det), 5, (255, 0, 0), -1)
            cv2.putText(frame, "Detector", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            break  # keep only the first matching detection

        # Detector Command
        confidence_det = 0.0
        if cx_det is not None:
            # Get confidence from detection
            for i in range(detections.shape[2]):
                class_id = int(detections[0, 0, i, 1])
                if class_id == TARGET_CLASS_ID:
                    confidence_det = detections[0, 0, i, 2]
                    break
            
            if cx_det < center_x - width * 0.1:
                command_det = "TURN_LEFT"
            elif cx_det > center_x + width * 0.1:
                command_det = "TURN_RIGHT"
            else:
                command_det = "FORWARD"
        else:
            command_det = "SEARCH"

        # HSV conversion and mask (your existing values)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Used for Part 1-5: yellow object
        lower_yellow = np.array([20, 50, 80])
        upper_yellow = np.array([35, 255, 255])
        # Used for Part 6: orange object
        lower_orange = np.array([15, 100, 80])
        upper_orange = np.array([30, 220, 220])
        mask1 = cv2.inRange(hsv, lower_orange, upper_orange) # modify based on which object to test on

        mask = mask1

        # Noise Reduction
        kernel = np.ones((5, 5), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

        # Contours
        contours, hierarchy = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        result = cv2.bitwise_and(frame, frame, mask=mask)

        tracked_frame = frame.copy()

        cx = None
        cy = None
        area = 0

        if contours:
            # Optional: filter small contours and pick largest valid by area
            valid_contours = [c for c in contours if cv2.contourArea(c) >= 50]  # filter tiny noise
            if valid_contours:
                largest = max(valid_contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = None, None

                # Draw contour and centroid (if valid)
                cv2.drawContours(tracked_frame, [largest], -1, (0, 255, 0), 2)
                if cx is not None and cy is not None:
                    cv2.circle(tracked_frame, (cx, cy), 5, (0, 0, 255), -1)

        # ---------- Task 4: Decide command from cx ----------
        if cx is None:
            command = "SEARCH"
        else:
            # if the detected area is too small, treat as not found
            if area < area_min:
                command = "SEARCH"
            else:
                if cx < center_x - deadband:
                    command = "TURN_LEFT"
                elif cx > center_x + deadband:
                    command = "TURN_RIGHT"
                else:
                    command = "FORWARD"

        # Overlay command and debug info on tracked_frame
        cv2.putText(tracked_frame, f"Color Cmd: {command}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(tracked_frame, f"Det Cmd: {command_det}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cx_text = str(cx) if cx is not None else "-"
        cv2.putText(tracked_frame, f"cx: {cx_text} area: {int(area)}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        # draw center and deadband lines for visual aid
        cv2.line(tracked_frame, (center_x, 0), (center_x, height), (255, 255, 0), 1)
        cv2.line(tracked_frame, (center_x - deadband, 0), (center_x - deadband, height), (200, 200, 200), 1)
        cv2.line(tracked_frame, (center_x + deadband, 0), (center_x + deadband, height), (200, 200, 200), 1)

        # ---------- Logging ----------
        timestamp = time.time()
        # Log color tracking results
        log_color.append((frame_idx, timestamp, cx if cx is not None else -1, cy if cy is not None else -1, int(area), command))
        # Log MobileNet detection results
        log_mobilenet.append((frame_idx, timestamp, cx_det if cx_det is not None else -1, cy_det if cy_det is not None else -1, confidence_det, command_det))
        cx_history.append(cx if cx is not None else None)
        command_history.append(command)

        # Write annotated frame to video if enabled
        if writer is not None:
            # ensure tracked_frame is BGR and same size
            writer.write(tracked_frame)

        # Display all frames
        # cv2.imshow('Raw Frame - Press SPACE to pause, Q to quit', frame)
        # cv2.imshow('HSV Frame', hsv)
        # cv2.imshow("Mask", mask)
        # cv2.imshow("Cleaned Mask", mask_clean)
        # cv2.imshow("Segmented Object", result)
        cv2.imshow("Tracked Object", tracked_frame)

        key = cv2.waitKey(1) & 0xFF

        # Hit "Space" key to pause the video
        if key == ord(' '):
            paused = not paused
            if paused:
                print("Video paused - press SPACE to resume")
                # Optionally save the frame when paused
                cv2.imwrite("captured_frame.jpg", frame)
                print("Frame saved as 'captured_frame.jpg'")

        # Exit on 'q' key
        elif key == ord('q'):
            break

        frame_idx += 1

    # --- Cleanup ---
    cap.release()
    if writer is not None:
        writer.release()
        print(f"Saved annotated video as {annotated_filename}")
    cv2.destroyAllWindows()

    # --- Save CSV of results ---
    os.makedirs("results", exist_ok=True)
    
    # Save color tracking results
    csv_path_color = os.path.join("results", results_csv_color)
    with open(csv_path_color, 'w', newline='') as f:
        writer_csv = csv.writer(f)
        writer_csv.writerow(['frame_idx', 'timestamp', 'cx', 'cy', 'area', 'command'])
        writer_csv.writerows(log_color)
    print(f"Saved color tracking CSV: {csv_path_color}")
    
    # Save MobileNet detection results
    csv_path_mobilenet = os.path.join("results", results_csv_mobilenet)
    with open(csv_path_mobilenet, 'w', newline='') as f:
        writer_csv = csv.writer(f)
        writer_csv.writerow(['frame_idx', 'timestamp', 'cx_det', 'cy_det', 'confidence', 'command_det'])
        writer_csv.writerows(log_mobilenet)
    print(f"Saved MobileNet detection CSV: {csv_path_mobilenet}")

    # --- Plot cx over time for color tracking ---
    frames_color = [row[0] for row in log_color]
    cxs_color = [row[2] if row[2] != -1 else None for row in log_color]

    plt.figure(figsize=(10,4))
    # plot only numeric cx values; for None use nan so matplotlib skips them
    cxs_numeric = [float(x) if x is not None and x != -1 else np.nan for x in cxs_color]
    plt.plot(frames_color, cxs_numeric, marker='o', linestyle='-', label='cx (color)', color='green')
    plt.axhline(center_x, linestyle='--', label='center_x')
    plt.axhline(center_x - deadband, linestyle=':', label='left band')
    plt.axhline(center_x + deadband, linestyle=':', label='right band')
    plt.xlabel('frame')
    plt.ylabel('cx (pixels)')
    plt.title('Color Tracking: Object centroid x over time')
    plt.legend()
    plt.grid(True)
    plot_path_color = os.path.join("results", cx_plot_png_color)
    plt.savefig(plot_path_color, bbox_inches='tight')
    print(f"Saved color tracking plot: {plot_path_color}")
    
    # --- Plot cx over time for MobileNet detection ---
    frames_mobilenet = [row[0] for row in log_mobilenet]
    cxs_mobilenet = [row[2] if row[2] != -1 else None for row in log_mobilenet]
    confidences = [row[4] for row in log_mobilenet]

    plt.figure(figsize=(10,4))
    cxs_numeric_mobilenet = [float(x) if x is not None and x != -1 else np.nan for x in cxs_mobilenet]
    plt.plot(frames_mobilenet, cxs_numeric_mobilenet, marker='s', linestyle='-', label='cx_det (MobileNet)', color='blue')
    plt.axhline(center_x, linestyle='--', label='center_x')
    plt.axhline(center_x - width * 0.1, linestyle=':', label='left band')
    plt.axhline(center_x + width * 0.1, linestyle=':', label='right band')
    plt.xlabel('frame')
    plt.ylabel('cx_det (pixels)')
    plt.title('MobileNet Detection: Object centroid x over time')
    plt.legend()
    plt.grid(True)
    plot_path_mobilenet = os.path.join("results", cx_plot_png_mobilenet)
    plt.savefig(plot_path_mobilenet, bbox_inches='tight')
    print(f"Saved MobileNet detection plot: {plot_path_mobilenet}")
    
    plt.show()

  


if __name__ == "__main__":
    main()

