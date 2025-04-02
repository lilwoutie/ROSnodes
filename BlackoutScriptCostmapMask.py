import cv2
import numpy as np

def blackout_area(image_path, coordinates, grayscale_value=255):
    """
    Reads a .pgm image, blacks out the area within the given coordinates, and saves the result.

    :param image_path: Path to the .pgm image file.
    :param coordinates: A list of 4 tuples representing the vertices of the area to blackout (x, y).
    :param grayscale_value: The grayscale value to use for the blackout area (default is 255).
    """
    # Load the .pgm image
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"Image not found at {image_path}")

    # Create a mask with the same dimensions as the image
    mask = np.zeros_like(image)

    # Define the polygon using the coordinates
    points = np.array(coordinates, dtype=np.int32)

    # Fill the polygon on the mask with the specified grayscale value
    cv2.fillPoly(mask, [points], grayscale_value)

    # Apply the mask to blackout the area in the original image
    image[mask == grayscale_value] = 0

    # Save the modified image
    output_path = image_path.replace(".pgm", "_blackout.pgm")
    cv2.imwrite(output_path, image)
    print(f"Modified image saved to {output_path}")

# Example usage
if __name__ == "__main__":
    # Path to the .pgm image (relative path)
    image_path = "./images/laser.pgm"

    # Coordinates of the area to blackout (x, y)
    coordinates = [(50, 50), (150, 50), (150, 150), (50, 150)]

    # Call the function with the default grayscale value
    blackout_area(image_path, coordinates)

    # Example of calling the function with a custom grayscale value
    # blackout_area(image_path, coordinates, grayscale_value=128)