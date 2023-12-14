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


list_images = ['lineaRecta\\codigoCentroDeMasa\\IMGCut.jpg', 'lineaRecta\\codigoCentroDeMasa\\IMGCut2.jpg', 'lineaRecta/codigoCentroDeMasa/IMGIzqCut.jpg', 'lineaRecta/codigoCentroDeMasa/IMGIzqCut2.jpg']
for img in list_images:
    dilate_erode_image(img)
    
# Use the function
#new_image = combine_images(
#    'lineaRecta\\codigoCentroDeMasa\\IMGCutmod.jpg', 'lineaRecta\\codigoCentroDeMasa\\IMGCut2mod.jpg')
#new_image.save('lineaRecta\\codigoCentroDeMasa\combined_image1.png')
