import cv2
from marker_detector import MarkerDetector

class CameraSystem:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.detector = MarkerDetector()    
        self.latest_tvec = None

    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            return None

        detections = self.detector.detect(frame)

        if detections:
            self.latest_tvec = detections[0]["tvec"]

        return self.latest_tvec

    def get_tvec(self):
        return self.latest_tvec