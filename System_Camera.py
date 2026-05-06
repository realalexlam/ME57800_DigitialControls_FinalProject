import cv2
import time
import numpy as np
from marker_detector import MarkerDetector

class CameraSystem:
    def __init__(self, camera_index=0):
        # FORCE the V4L2 driver and 640x480 resolution to prevent crashes
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.detector = MarkerDetector()    
        self.latest_tvec = None

    def update(self):
        ret, frame = self.cap.read()
        
        if not ret or frame is None or frame.shape[0] == 0:
            return None, None

        # Create a clean copy to prevent memory errors
        clean_frame = frame.copy()
        detections = self.detector.detect(clean_frame)

        if detections:
            self.latest_tvec = detections[0]["tvec"]
            
            # Draw the bounding box and ID on the image!
            for det in detections:
                cv2.aruco.drawDetectedMarkers(clean_frame, [det["corners"]], np.array([det["id"]]))

        # Return BOTH the math and the image so we can display it
        return self.latest_tvec, clean_frame

    def get_tvec(self):
        return self.latest_tvec

# --- Auto-Hunt execution loop ---
if __name__ == "__main__":
    print("Starting direct OpenCV camera connection...")
    print("Hunting for the correct RealSense RGB channel via V4L2...")
    
    working_index = -1
    
    for i in range(8):
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None and frame.shape[0] > 0:
                working_index = i
                cap.release()
                break
        cap.release()

    if working_index == -1:
        print("Error: Could not find a working RGB video stream.")
    else:
        print(f"Success! Locked onto the RGB camera at index {working_index}.")
        cam = CameraSystem(camera_index=working_index) 
        print("Camera is live. A video window should pop up shortly!")
        
        try:
            while True:
                tvec, frame = cam.update()
                
                # Show the live video feed!
                if frame is not None:
                    cv2.imshow("Alchemist Camera Feed", frame)

                if tvec is not None:
                    x, y, z = tvec
                    print(f"Marker Found! XYZ: [X: {x:.3f}, Y: {y:.3f}, Z: {z:.3f}] meters")
                
                # waitKey allows the video window to refresh. Press 'q' to quit cleanly.
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\n'q' pressed. Exiting...")
                    break
                    
        except KeyboardInterrupt:
            print("\nClosing camera...")
        finally:
            cam.cap.release()
            cv2.destroyAllWindows()
