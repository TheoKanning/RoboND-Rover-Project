import numpy as np
import cv2


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
                   & (img[:, :, 1] > rgb_thresh[1]) \
                   & (img[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


def hsv_thresh(rgb_img, low=(15, 80, 130), high=(30, 255, 180)):
    hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
    color_select = np.zeros_like(rgb_img[:, :, 0])
    color_select[cv2.inRange(hsv_img, low, high) == 255] = 1
    return color_select


def get_navigable(img, rgb_thresh=(160, 160, 160)):
    return color_thresh(img, rgb_thresh)


# Identifies obstacles by taking the inverse of the navigable area
def get_obstacle(img):
    navigable = get_navigable(img)
    obstacle = np.zeros_like(navigable)

    # assume that anything equal to [0,0,0] is not visible to the camera
    not_visible = (img[:, :, 0] == 0) \
                  & (img[:, :, 1] == 0) \
                  & (img[:, :, 2] == 0)
    obstacle[navigable == 0] = 1
    obstacle[not_visible] = 0  # reset all not-visible pixels to 0
    return obstacle


# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1] / 2).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles


# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated


def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world


# Translates from image coords to world coords by calling rover_coords
# and pix_to_world in succession.
# Returns x_pixels, y_pixels
def image_to_world(binary_img, xpos, ypos, yaw, worldsize, scale):
    xpix, ypix = rover_coords(binary_img)
    return pix_to_world(xpix, ypix, xpos, ypos, yaw, worldsize, scale)


# Define a function to perform a perspective transform
def perspect_transform(image):
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[image.shape[1] / 2 - dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1] / 2 + dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1] / 2 + dst_size, image.shape[0] - 2 * dst_size - bottom_offset],
                              [image.shape[1] / 2 - dst_size, image.shape[0] - 2 * dst_size - bottom_offset],
                              ])
    M = cv2.getPerspectiveTransform(source, destination)
    warped = cv2.warpPerspective(image, M, (image.shape[1], image.shape[0]))  # keep same size as input image

    return warped


# Return true if rover is stable enough to record map data
def stable(rover):
    if 1 < rover.pitch < 359:
        return False
    elif 1 < rover.roll < 359:
        return False
    return True


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
    #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
    #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    # Rover.nav_dists = rover_centric_pixel_distances
    # Rover.nav_angles = rover_centric_angles
    warped = perspect_transform(rover.img)
    navigable = get_navigable(warped)
    obstacle = get_obstacle(warped)
    sample = hsv_thresh(warped)

    # update rover's vision for debugging
    rover.vision_image[:, :, 0] = obstacle * 255
    rover.vision_image[:, :, 1] = sample * 255
    rover.vision_image[:, :, 2] = navigable * 255

    xpos = rover.pos[0]
    ypos = rover.pos[1]
    yaw = rover.yaw
    nav_world_x, nav_world_y = image_to_world(navigable, xpos, ypos, yaw, rover.worldmap.shape[0], 10)
    obs_world_x, obs_world_y = image_to_world(obstacle, xpos, ypos, yaw, rover.worldmap.shape[0], 10)
    sam_world_x, sam_world_y = image_to_world(sample, xpos, ypos, yaw, rover.worldmap.shape[0], 10)

    rover.worldmap[:,:,0] = rover.unexplored

    # if stable(rover):
    #     rover.worldmap[obs_world_y, obs_world_x, 0] += 1
    #     rover.worldmap[sam_world_y, sam_world_x, 1] += 1
    #     rover.worldmap[nav_world_y, nav_world_x, 2] += 1
    #     rover.worldmap[nav_world_y, nav_world_x, 0] -= 0.5  # remove obstacle data if navigable

    nav_rover_x, nav_rover_y = rover_coords(navigable)
    dist, angles = to_polar_coords(nav_rover_x, nav_rover_y)
    rover.nav_angles = angles
    rover.nav_dists = dist

    return rover
