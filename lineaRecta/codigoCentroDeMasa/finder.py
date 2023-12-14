from PIL import Image
import numpy as np
import cv2
import os





def dilate_erode_image(image_path):
    # Load the image
    image = cv2.imread(image_path, 0)

    # Invert colors
    inverted_image = cv2.bitwise_not(image)

    # Define the kernel
    kernel = np.ones((5, 5), np.uint8)

    # Apply dilation
    dilated_image = cv2.dilate(inverted_image, kernel, iterations=3)

    # Apply erosion
    eroded_image = cv2.erode(dilated_image, kernel, iterations=1)

    # Invert colors back
    final_image = cv2.bitwise_not(eroded_image)

    # Split the file path into directory, filename, and extension
    dir_path, filename = os.path.split(image_path)
    base, ext = os.path.splitext(filename)

    # Create a new filename by inserting "mod" before the extension
    new_filename = f"{base}_mod{ext}"

    # Join the directory path and the new filename
    path = os.path.join(dir_path, new_filename)
    print(path)

    # Save the image
    cv2.imwrite(path, final_image)

def combine_images(image1_path, image2_path):
    # Open the images and convert them to RGB
    image1 = Image.open(image1_path).convert("RGB")
    image2 = Image.open(image2_path).convert("RGB")

    # Resize images to the smallest size
    min_size = min(image1.size, image2.size)
    image1 = image1.resize(min_size)
    image2 = image2.resize(min_size)

    # Convert images to numpy arrays
    image1_data = np.array(image1)
    image2_data = np.array(image2)

    # Create a new image with the same size and RGB format
    new_image_data = np.zeros(image1_data.shape, dtype=np.uint8)

    # Set the green channel to the first image's black pixels
    new_image_data[(image1_data[:, :, 0] == 0) & (
        image1_data[:, :, 1] == 0) & (image1_data[:, :, 2] == 0)] = [0, 255, 0]

    # Set the red channel to the second image's black pixels
    new_image_data[(image2_data[:, :, 0] == 0) & (
        image2_data[:, :, 1] == 0) & (image2_data[:, :, 2] == 0)] = [255, 0, 0]

    # Create a new image from the data
    new_image = Image.fromarray(new_image_data)

    return new_image


# Global variables
points = []
image = None


def click_event(event, x, y, flags, params):
    # Check if left mouse button clicked
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))

        # Draw line if two points have been clicked
        if len(points) == 2:
            cv2.line(image, points[0], points[1], (255, 0, 0), 5)
            angle = np.arctan2(points[1][1] - points[0]
                               [1], points[1][0] - points[0][0])
            rotated = rotate_image(image, angle)
            cv2.imshow('image', rotated)


def rotate_image(image, angle):
    # Get image dimensions
    (h, w) = image.shape[:2]
    # Define the pivot point
    center = (w / 2, h / 2)
    # Perform the rotation
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(image, M, (w, h))
    return rotated


def process_image(image_path):
    global image
    image = cv2.imread(image_path)
    cv2.imshow('image', image)
    cv2.setMouseCallback('image', click_event)

    while True:
        key = cv2.waitKey(0) & 0xFF
        if key == ord('s'):
            cv2.imwrite('rotated_image.jpg', image)
            break
        elif key == ord('r'):
            image = cv2.imread(image_path)
            cv2.imshow('image', image)
            points.clear()

    cv2.destroyAllWindows()

list_images = ['lineaRecta\\codigoCentroDeMasa\\IMGCut.jpg', 'lineaRecta\\codigoCentroDeMasa\\IMGCut2.jpg', 'lineaRecta/codigoCentroDeMasa/IMGIzqCut.jpg', 'lineaRecta/codigoCentroDeMasa/IMGIzqCut2.jpg']
for img in list_images:
    dilate_erode_image(img)
    
# Use the function
new_image = combine_images(
    'lineaRecta\\codigoCentroDeMasa\\IMGCut_mod.jpg', 'lineaRecta\\codigoCentroDeMasa\\IMGCut2_mod.jpg')
new_image.save('lineaRecta\\codigoCentroDeMasa\combined_image1.png')

new_image = combine_images(
    'lineaRecta\\codigoCentroDeMasa\\IMGizqCut_mod.jpg', 'lineaRecta\\codigoCentroDeMasa\\IMGizqCut2_mod.jpg')
new_image.save('lineaRecta\\codigoCentroDeMasa\combined_image2.png')

# Use the function
process_image('lineaRecta\\codigoCentroDeMasa\combined_image1.png')
