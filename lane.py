import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import glob
from PIL import Image, ImageDraw
%matplotlib qt

def CameraCalibration(images,objpoints,imgpoints):
    
    objp = np.zeros( (6*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2) # x, y coordinates
    
    for fname in images:
        
        img = mpimg.imread(fname)
        
        #Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
        
        # If found, draw corners 
        if ret == True:
            imgpoints.append(corners)
            objpoints.append(objp)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (9, 6), corners, ret)

def Cal_undistort(test_img, objpoints, imgpoints, mtx, dist):
    
    #Undistort test image using object points and image points
    a1 = cv2.undistort(test_img, mtx, dist, None, mtx)
    
    return a1

def WarpPerspective(test_img, mtx, dist):
    
    # Grab the image shape
    img_size = (test_img.shape[1], test_img.shape[0]) #test_img.shape[1], [0] each mean width and height of the image. 
    width = test_img.shape[1]
    height = test_img.shape[0]
    
    # For source points I'm grabbing the outer four detected corners
    src = np.float32(
        [[(img_size[0] / 2) - 55, img_size[1] / 2 + 100],
        [((img_size[0] / 6) - 10), img_size[1]],
        [(img_size[0] * 5 / 6) + 60, img_size[1]],
        [(img_size[0] / 2 + 55), img_size[1] / 2 + 100]])

    # For destination points, I'm arbitrarily choosing some points to be
    # a nice fit for displaying our warped result 
    # again, not exact, but close enough for our purposes
    dst = np.float32(
        [[(width / 4), 0],
        [(width / 4), img_size[1]],
        [(width * 3 / 4), img_size[1]],
        [(width * 3 / 4), 0]])
    
    # Given src and dst points, calculate the perspective transform matrix
    M = cv2.getPerspectiveTransform(src, dst)

    # Warp the image using OpenCV warpPerspective()
    warped = cv2.warpPerspective(test_img, M, img_size)
    
    return warped, M, src, dst

def luv_thresh(img, thresh=(225, 255)):
    
    #Convert RGB to HLS and threshold to binary image using S channel
    l_channel = cv2.cvtColor(img, cv2.COLOR_BGR2LUV)[:,:,0]
    sxbinary = np.zeros_like(l_channel)
    sxbinary[(l_channel > thresh[0]) & (l_channel <= thresh[1])] = 1
    return sxbinary

def hls_thresh(img, thresh=(180, 255)):
    
    #Convert RGB to HLS and threshold to binary image using S channel
    s_channel = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)[:,:,2]
    sxbinary = np.zeros_like(s_channel)
    sxbinary[(s_channel > thresh[0]) & (s_channel <= thresh[1])] = 1
    return sxbinary

def lab_bthresh(img, thresh=(155, 200)):
    # 1) Convert to LAB color space
    lab = cv2.cvtColor(img, cv2.COLOR_RGB2Lab)
    lab_b = lab[:,:,2]
    # 2) Apply a threshold to the L channel
    sxbinary = np.zeros_like(lab_b)
    sxbinary[((lab_b > thresh[0]) & (lab_b <= thresh[1]))] = 1
    # 3) Return a binary image of threshold result
    return sxbinary

def combined_thresh(img):
    
    #combined with luv l and lab b of the gradient
    hls_bin = hls_thresh(img, thresh=(180, 255))
    luv_bin = luv_thresh(img, thresh=(225, 255))
    lab_bin = lab_bthresh(img, thresh=(155, 200))
    combined = np.zeros_like(hls_bin)
    combined[(luv_bin == 1) | (lab_bin == 1) ] = 1
    
    return combined

def find_lane_pixels(binary_warped):
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
    # Create an output image to draw on and visualize the result
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0]//2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # HYPERPARAMETERS
    # Choose the number of sliding windows
    nwindows = 9
    # Set the width of the windows +/- margin
    margin = 100
    # Set minimum number of pixels found to recenter window
    minpix = 50

    # Set height of windows - based on nwindows above and image shape
    window_height = np.int(binary_warped.shape[0]//nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated later for each window in nwindows
    leftx_current = leftx_base
    rightx_current = rightx_base

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        
        # Find the four below boundaries of the window ###
        win_xleft_low = leftx_current - margin  # Update this
        win_xleft_high = leftx_current + margin  # Update this
        win_xright_low = rightx_current - margin  # Update this
        win_xright_high = rightx_current + margin  # Update this
        
        # Draw the windows on the visualization image
        cv2.rectangle(out_img,(win_xleft_low,win_y_low),
        (win_xleft_high,win_y_high),(0,255,0), 2) 
        cv2.rectangle(out_img,(win_xright_low,win_y_low),
        (win_xright_high,win_y_high),(0,255,0), 2) 
        
        # Identify the nonzero pixels in x and y within the window ###
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        
        ### If you found > minpix pixels, recenter next window ###
        ### (`right` or `leftx_current`) on their mean position ###
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    # Concatenate the arrays of indices (previously was a list of lists of pixels)
    try:
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
    except ValueError:
        # Avoids an error if the above is not implemented fully
        pass

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    
    return left_fit, right_fit, leftx, lefty, rightx, righty

def search_around_poly(binary_warped):
    
    left_fit, right_fit, leftx, lefty, rightx, righty = find_lane_pixels(binary_warped)
    
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
    
    ### Calc both polynomials using ploty, left_fit and right_fit ###
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
    car_position = binary_warped.shape[1]/2 # Car's position in any frame
    xm_per_pix = 3.7/700 # convert pixel size to real size.

    car_y = binary_warped.shape[0] - 1 # Y position of the car to calculate polynomial of the car.
    car_x_left = left_fit[0]*(car_y**2) + left_fit[1]*car_y + left_fit[2] # Finding the distance car turned left
    car_x_right = right_fit[0]*(car_y**2) + right_fit[1]*car_y + right_fit[2] # finding the distance car turned right
    
    lane_center_position = (car_x_left + car_x_right)/2 # Needed to be divided by 2 to find value of turns.
    center_dist = (car_position - lane_center_position) # To calculate the value of distance from the cneter
    center_dist *= xm_per_pix
    
    return left_fitx, right_fitx, ploty, leftx, lefty, rightx ,righty, result, center_dist

def generate_data(output,leftx,lefty,rightx,righty,ym_per_pix, xm_per_pix):
    
    # Generate x and y values for plotting
    ploty = np.linspace(0, output.shape[0]-1, output.shape[0])

    # Fit a second order polynomial to pixel positions in each fake lane line
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    
    left_fit_cr = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
    
    return ploty, left_fit_cr, right_fit_cr

def measure_curvature_real(out_img,leftx,lefty,rightx,righty):
    '''
    Calculates the curvature of polynomial functions in meters.
    '''
    # Define conversions in x and y from pixels space to meters
    ym_per_pix = 30/720 # meters per pixel in y dimension
    xm_per_pix = 3.7/700 # meters per pixel in x dimension
    
    # Start by generating our fake example data
    # Make sure to feed in your real data instead in your project!
    ploty, left_fit_cr, right_fit_cr = generate_data(out_img,leftx,lefty,rightx,righty,ym_per_pix,xm_per_pix)
    
    # Define y-value where we want radius of curvature
    # We'll choose the maximum y-value, corresponding to the bottom of the image
    y_eval = np.max(ploty)
    
    ##### Implement the calculation of R_curve (radius of curvature) #####
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    
    
    return left_curverad, right_curverad

"""images = glob.glob('camera_cal/calibration*.jpg')

# Arrays to store object points and image points from all the images.

objpoints = [] # 3D points in real world space
imgpoints = [] # 2D points in image plane

CameraCalibration(images,objpoints,imgpoints) # Function to find the corners

image = mpimg.imread('test_images/test1.jpg')

#Convert to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
#Calibrate camera using cv2.calibrate pre-defined library
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1] , None, None)"""

def process_image(image):

    result=Cal_undistort(image, objpoints, imgpoints, mtx, dist) # Funtion to calibrate camera and undistort the images.
    output = combined_thresh(result)
    top_down, perspective_M, src, dst= WarpPerspective(output, mtx, dist)
    Minv = np.linalg.inv(perspective_M)
    
    # Grapping left fitted x and right fitted x also to use measure_curvature_real function, 
    # extracting leftx, lefty, rightx, righty values.
    # Finally, in serach_around_poly I grabbed offset of the car from the center
    left_fitx, right_fitx, ploty, leftx,lefty,rightx,righty,out_img,center_dist = search_around_poly(top_down)
    
    # To grap the curvature of left and right
    left,right = measure_curvature_real(top_down,leftx,lefty,rightx,righty)
    curvature = (left+right)/2
    
    """Image visualization to plot back to original image"""
    warp_zero = np.zeros_like(top_down).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    # Draw the lane onto the warped blank image
    cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    newwarp = cv2.warpPerspective(color_warp, Minv, (image.shape[1], image.shape[0])) 
    # Combine the result with the original image
    FinalOutput = cv2.addWeighted(result, 1, newwarp, 0.3, 0)
    """Image visualization end"""
    
    if center_dist > 0:
        direction = 'right'
    else:
        direction = 'left' 
    abs_center_dist = abs(center_dist)
    text = '{:04.3f}'.format(abs_center_dist) + 'm ' + direction + ' of center'
    cv2.putText(FinalOutput, text, (100,150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), lineType=cv2.LINE_AA)
    cv2.putText(FinalOutput, 'Road Curvature:' +str(curvature) +' m', (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), lineType=cv2.LINE_AA) 
    
        
    
    return FinalOutput,direction


