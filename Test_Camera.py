import cv2
from marker_detector import MarkerDetector

def main():
    # Open default camera    (0 = first webcam)
    
    cap = cv2.VideoCapture(0)

    # Optional: force resolution 
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("ERROR: Could not open camera")
        return
    
    detector = MarkerDetector()

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(
        "camera_test_recording.mp4",
        fourcc,
        cap.get(cv2.CAP_PROP_FPS),
        (
            int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        ),
    )
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        detections = detector.detect(frame)

        out.write(frame)
        cv2.imshow("Camera", frame)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC key
            break 

    out.release()
    cap.release()
    cv2.destroyAllWindows()
if __name__ == "__main__":
    main()