import cv2
import numpy as np

print(f"OpenCV Version: {cv2.__version__}")

try:
    # 1. Setup the Dictionary (Standard 6x6)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    
    # 2. Generate a Marker (ID=42)
    # Create a white background
    img = np.ones((400, 400), dtype=np.uint8) * 255
    # Draw marker ID 42 into the image
    img = cv2.aruco.drawMarker(aruco_dict, 42, 300, img, 1)
    
    # 3. Attempt Detection
    parameters = cv2.aruco.DetectorParameters()
    
    # Check for new API (OpenCV 4.7+) vs Old API
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejected = detector.detectMarkers(img)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=parameters)

    # 4. Verify Result
    if ids is not None and 42 in ids:
        print("\n✅ SUCCESS: ArUco is installed and working!")
        print(f"   Detected Marker ID: {ids[0][0]}")
        
        # Optional: Show the test image
        # cv2.imshow("Test Marker", img)
        # cv2.waitKey(2000)
        # cv2.destroyAllWindows()
    else:
        print("\n❌ FAILURE: ArUco functions loaded, but detection failed.")

except AttributeError:
    print("\n❌ FAILURE: 'cv2.aruco' not found.")
    print("   You likely installed 'opencv-python' (standard).")
    print("   Please run: pip uninstall opencv-python && pip install opencv-contrib-python")
except Exception as e:
    print(f"\n❌ ERROR: {e}")