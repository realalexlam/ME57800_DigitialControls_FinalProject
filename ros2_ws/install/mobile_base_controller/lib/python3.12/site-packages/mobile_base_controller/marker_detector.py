import cv2
import numpy as np

class MarkerDetector:
    def __init__(self, 
                 aruco_dict_type=cv2.aruco.DICT_4X4_50,
                 marker_length = 0.1, # Choose marker Size
                 camera_matrix=None,
                 dist_coeffs=None):
        
        self.image_width = 640
        self.image_height = 480
        self.marker_length = marker_length

        # OpenCV 4.7+ uses getPredefinedDictionary instead of Dictionary_get
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        except AttributeError:
            # Fallback for older OpenCV versions
            self.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)

        # OpenCV 4.7+ uses DetectorParameters() instead of DetectorParameters_create()
        try:
            self.detector_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            # Fallback for older OpenCV versions
            self.detector_params = cv2.aruco.DetectorParameters_create()

        if camera_matrix is None:
            camera_matrix = np.array([[600, 0, 320],
                                      [0, 600, 240],
                                      [0, 0, 1]], dtype=np.float32)
        if dist_coeffs is None:
            dist_coeffs = np.zeros((5, 1))
        
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def detect(self, image):

         # OpenCV 4.7+ uses ArucoDetector class instead of detectMarkers function
        try:
            # Try new API (OpenCV 4.7+)
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
            corners, ids, _ = detector.detectMarkers(image)
        except AttributeError:
            # Fallback to old API (OpenCV < 4.7)
            corners, ids, _ = cv2.aruco.detectMarkers(image, self.aruco_dict, parameters=self.detector_params)

        if ids is None or len(corners) == 0:
            return []

        detections = []

        # Estimate pose
        half_size = self.marker_length / 2
        obj_points = np.array([
            [-half_size,  half_size, 0],
            [ half_size,  half_size, 0],
            [ half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

        img_points = corners[0].reshape(-1, 2)


        retval, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    img_points,
                    self.camera_matrix,
                    self.dist_coeffs
                )
        tvec = tvec.flatten()
        rvec = rvec.flatten()
        
        detections.append({
            "id": int(ids[0]),
            "tvec": tvec,
            "rvec": rvec,
            "corners": corners[0]
        })

        return detections
