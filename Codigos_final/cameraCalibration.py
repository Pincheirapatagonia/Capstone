import numpy as np
import cv2
import glob

# Define the size of the chessboard pattern
chessboard_size = (9, 6)  # Change this to the number of inner corners in your chessboard

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Get a list of calibration images
images = glob.glob('calibration_images/*.jpg')  # Change the path to the directory containing your calibration images

# Iterate through each calibration image
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    # If corners are found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)  # Adjust the delay as needed

cv2.destroyAllWindows()

# Calibrate the camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save the calibration results
np.savez('camera_calibration.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

def point_to_meter(x, y):
    # Load the camera calibration data
    calibration_data = np.load('camera_calibration.npz')
    mtx = calibration_data['mtx']
    dist = calibration_data['dist']

    # Pixel coordinates of a point in the image
    pixel_coords = np.array([[x, y]], dtype=np.float32)

    # Undistort pixel coordinates
    undistorted_coords = cv2.undistortPoints(pixel_coords, mtx, dist)

    # Convert undistorted pixel coordinates to meters
    # Assuming you have information about the real-world dimensions of the object
    object_width_meters = 0.2  # Adjust this based on your actual object size
    object_height_meters = 0.1

    # Scale the undistorted pixel coordinates to meters
    undistorted_coords_meters = undistorted_coords * np.array([[object_width_meters, object_height_meters]])

    print("Undistorted coordinates in meters:", undistorted_coords_meters)