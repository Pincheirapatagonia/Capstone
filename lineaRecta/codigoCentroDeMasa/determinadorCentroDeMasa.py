import cv2
import os
import time

local = os.getcwd()
target = os.path.join(local, "lineaRecta", "codigoCentroDeMasa")
os.chdir(target)

def calculate_center_of_rotation(video_path):
    # Load the video
    cap = cv2.VideoCapture(video_path)

    # Create a background subtractor
    bg_subtractor = cv2.createBackgroundSubtractorMOG2()

    # Read the first frame
    ret, frame = cap.read()

    # Get the height and width of the frame
    height, width, _ = frame.shape

    # Define the region of interest (ROI)
    roi_top = height // 2 - 65
    roi_bottom = height - 35
    roi_left = 0
    roi_right = width

    while True:
        # Read the next frame
        ret, frame = cap.read()
        
        
        if not ret:
            break

        # Apply ROI to the frame
        frame = frame[roi_top:roi_bottom, roi_left:roi_right]

        BWframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        BWframe[BWframe <60] = 0
        BWframe[BWframe >= 60] = 255

        
        # Find contours in the black and white frame
        contours, _ = cv2.findContours(BWframe, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the rectangle contour (rectangle)
        max_area = 0
        max_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area and area < 150000:
                max_area = area
                max_contour = contour

        if max_contour is not None:
            # Get the bounding rectangle coordinates
            x, y, w, h = cv2.boundingRect(max_contour)


            # Calculate the center of rotation
            center_x = x + w // 2
            center_y = y + h // 2

            # Draw the rectangle on the frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Draw a circle at the center of rotation
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

        # Display the 
        cv2.imshow("BWframe", BWframe)
        cv2.imshow("Video", frame)
        time.sleep(0.1);
        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture and close the window
    cap.release()
    cv2.destroyAllWindows()

# Calculate the center of rotation for "giroDerecha.mp4"
calculate_center_of_rotation("giroDerecha.mp4")

# Calculate the center of rotation for "giroIzquierda.mp4"
# calculate_center_of_rotation("giroIzquierda.mp4")
