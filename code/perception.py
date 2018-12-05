import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped

def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    path = np.zeros_like(img[:,:,0])
    obstacles = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    below_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    path[above_thresh] = 1
    obstacles[below_thresh] = 1
    path3d = np.dstack((path*0, path*255, path*0)).astype(np.float)
    obstacles3d = np.dstack((obstacles*0, obstacles*0, obstacles*255)).astype(np.float)
    # Return the binary image
    return path, obstacles, path3d, obstacles3d

#Define a function to find the rocks
def find_rock(img, lower_thresh, upper_thresh):
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                  [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,lower_thresh,upper_thresh)
    res = cv2.bitwise_and(img,img, mask = mask)
    return mask, res


# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
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

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    bottom_offset = 6
    img = Rover.img
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                  [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped = perspect_transform(img,source,destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    low_yellow = np.array([90,100,100])
    up_yellow = np.array([100,255,255])
    path, obstacles, path3d, obstacles3d = color_thresh(warped)
    rocks,res =find_rock(warped, low_yellow, up_yellow)        # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,2]=(obstacles+rocks)*255
    Rover.vision_image[:,:,1]=(path+rocks)*255
    Rover.vision_image[:,:,0]=rocks

        # Example: Rover.vision_image[:,:,0] = obstacasle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    x_pixel,y_pixel = rover_coords(path)
    x_obspix,y_obspix = rover_coords(obstacles)
    x_rockpix,y_rockpix = rover_coords(rocks)
    # 6) Convert rover-centric pixel values to world coordinates
    worldmap = np.zeros((200, 200))
    world_size = worldmap.shape[0]
    scale = 2*dst_size
    x_worldpix,y_worldpix = pix_to_world(x_pixel,y_pixel,Rover.pos[0],Rover.pos[1],Rover.yaw,world_size,scale)
    x_worldobspix,y_worldobspix = pix_to_world(x_obspix,y_obspix,Rover.pos[0],Rover.pos[1],Rover.yaw,world_size,scale)
    x_worldrock,y_worldrock = pix_to_world(x_rockpix,y_rockpix,Rover.pos[0],Rover.pos[1],Rover.yaw,world_size,scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    Rover.worldmap[y_worldobspix,x_worldobspix,0] = 255
    Rover.worldmap[y_worldpix,x_worldpix,2] += 10
    nav_pix = Rover.worldmap[:,:,2] > 0
   # Rover.worldmap[nav_pix,0] = 0
    Rover.worldmap[y_worldrock,x_worldrock,:] = 255
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    
    dist,angle = to_polar_coords(x_pixel,y_pixel)
    rockdist,rockangle = to_polar_coords(x_rockpix, y_rockpix)
    Rover.rock_dists = rockdist
    Rover.mean_rockdists = np.mean(rockdist)
    Rover.rock_angle = rockangle
    # Update Rover pixel distances and angles
    Rover.nav_dists = dist

    Rover.nav_angles = angle       
        
        
        
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    
    return Rover