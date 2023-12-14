import cv2
import os
import time
import numpy as np
import math

local = os.getcwd()
target = os.path.join(local, "lineaRecta", "codigoCentroDeMasa")
os.chdir(target)

kernel = np.ones((3, 3), np.uint8)


def crop_minAreaRect(img, cnt):
    rect = cv2.minAreaRect(cnt)

    box = cv2.boxPoints(rect)
    box = np.int0(box)
    width = int(rect[1][0])
    height = int(rect[1][1])

    src_pts = box.astype("float32")
    dst_pts = np.array([[0, height-1],
                        [0, 0],
                        [width-1, 0],
                        [width-1, height-1]], dtype="float32")
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped = cv2.warpPerspective(img, M, (width, height))
    return warped


def calculate_center_of_rotation(video_path):
    # Load the video
    cap = cv2.VideoCapture(video_path)


    # Read the first frame
    ret, frame = cap.read()

    # Get the height and width of the frame
    height, width, _ = frame.shape

    # Define the region of interest (ROI)
    roi_top = height // 2 - 65
    roi_bottom = height - 35
    roi_left = 0
    roi_right = width

    lenCrop = 0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    current_frame = 0

    def on_change_frame(value):
        nonlocal current_frame
        current_frame = value

    #cv2.namedWindow("Slider")
    #cv2.createTrackbar("Frame", "Slider", 0, total_frames - 1, on_change_frame)
    i = 0
    while True:
        # Set the video capture to the current frame
        #cap.set(cv2.CAP_PROP_POS_FRAMES, current_frame)

        # Read the next frame
        ret, frame = cap.read()

        if not ret:
            break

        # Apply ROI to the frame
        frame = frame[roi_top:roi_bottom, roi_left:roi_right]

        BWframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        BWframe[BWframe < 60] = 0
        BWframe[BWframe >= 60] = 255

        img_erosion = cv2.erode(BWframe, kernel, iterations=7)
        img_dilation = cv2.dilate(img_erosion, kernel, iterations=5)

        # Find contours in the black and white frame
        contours, _ = cv2.findContours(
            img_dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the rectangle contour (rectangle)
        max_area = 0
        max_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area and area < 150000:
                max_area = area
                max_contour = contour
        if max_contour is not None:
            # draw the bigger contour
            cv2.drawContours(frame, [max_contour], 0, (0, 0, 255), 3)
            # find the center of the bigger contour
            M = cv2.moments(max_contour)

            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            # draw a circle in the center of the bigger contour
            cv2.circle(BWframe, (center_x, center_y), 5, (0, 0, 255), -1)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

            rect = cv2.minAreaRect(max_contour)

            width = int(rect[1][0])
            height = int(rect[1][1])
            if lenCrop == 0:
                lenCrop = max(width, height) * math.sqrt(2) / 2

            startx = round(max(center_x - lenCrop, 0))
            starty = round(max(center_y - lenCrop, 0))
            endx = round(min(center_x + lenCrop, frame.shape[1]))
            endy = round(min(center_y + lenCrop, frame.shape[0]))
            img_croped = BWframe[starty:endy, startx:endx]

            # cv2.imshow("Erosion", img_erosion)
            # cv2.imshow("Dilatacion", img_dilation)
            # cv2.imshow("BWframe", img_dilation)
            cv2.imshow("BWframe", BWframe)
            cv2.imshow("IMGCut", img_croped)

            if i == 170:  # 150
                cv2.imwrite("IMGIzqCut.jpg", img_croped)
            elif i == 180:  # 170
                cv2.imwrite("IMGIzqCut2.jpg", img_croped)
            # Update the slider position
            
            i += 1
            
            
            #cv2.setTrackbarPos("Frame", "Slider", current_frame)
            #time.sleep(0.1)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture and close the window
    cap.release()
    cv2.destroyAllWindows()


# Calculate the center of rotation for "giroDerecha.mp4"
#calculate_center_of_rotation("giroDerecha.mp4")

# Calculate the center of rotation for "giroIzquierda.mp4"
calculate_center_of_rotation("giroIzquierda.mp4")
