# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import cv2 as cv
import imutils
import time

# def nothing(x):
#     pass

# cv.namedWindow("Trackbars")

# cv.createTrackbar("L - H", "Trackbars", 0, 255, nothing)
# cv.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
# cv.createTrackbar("L - V", "Trackbars", 200, 255, nothing)
# cv.createTrackbar("U - H", "Trackbars", 255, 255, nothing)
# cv.createTrackbar("U - S", "Trackbars", 50, 255, nothing)
# cv.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

# load the image, clone it for output, and then convert it to grayscale
image = VideoStream(src=0, usePiCamera=False, resolution=(640, 480), framerate=30).start()

lower_blue = np.array([90, 50, 50])
upper_blue = np.array([130, 255, 255])

# Set parameters for k-means clustering
num_clusters = 3  # Number of clusters (colors)
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.2)

def main():
	while True:
		# grab the current frame
		frame = image.read()
		# handle the frame from VideoCapture or VideoStream
		if frame is None:
			break
        # Convert frame to HSV
		hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Create a mask for blue color
		# l_h = cv.getTrackbarPos("L - H", "Trackbars")
		# l_s = cv.getTrackbarPos("L - S", "Trackbars")
		# l_v = cv.getTrackbarPos("L - V", "Trackbars")
		# u_h = cv.getTrackbarPos("U - H", "Trackbars")
		# u_s = cv.getTrackbarPos("U - S", "Trackbars")
		# u_v = cv.getTrackbarPos("U - V", "Trackbars")
		# lower = np.array([l_h,l_s,l_v])
		# upper = np.array([u_h,u_s,u_v])
		mask = cv.inRange(hsv, lower_blue, upper_blue)

        # Apply the mask to the original frame
		segmented_image = cv.bitwise_and(frame, frame, mask=mask)
		mask = cv.erode(mask, None, iterations=2)
		mask = cv.dilate(mask, None, iterations=2)
		scale_frame = cv.cvtColor(segmented_image, cv.COLOR_HSV2BGR)
		grayscale_frame = cv.cvtColor(scale_frame, cv.COLOR_BGR2GRAY)
		thresh = cv.adaptiveThreshold(grayscale_frame,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY,11,2)
		#blurFrame = cv.GaussianBlur(grayscale_frame, (17, 17), 0)

		contours, _ = cv.findContours(grayscale_frame.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		if len(contours) > 0:
			c = max(contours, key=cv.contourArea)
			((x, y), radius) = cv.minEnclosingCircle(c)
			if radius > 10 and radius < 300:
				cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

        # Display the original and segmented images
		cv.imshow('Original', frame)
		cv.imshow('Segmented', thresh)
		
		if cv.waitKey(1) & 0xFF == ord('q'):
			break
	image.stop()
	image.stream.release()
	cv.destroyAllWindows()

if __name__=='__main__':
	main()
	
    #KMEANS FILTER
	# # Resize frame for faster processing (optional)
		# frame = cv.resize(frame, None, fx=0.5, fy=0.5)
		# # Flatten the image to a list of pixels
		# pixels = frame.reshape(-1, 3).astype(np.float32)
		# # Apply k-means clustering
		# _, labels, centers = cv.kmeans(pixels, num_clusters, None, criteria, 10, cv.KMEANS_RANDOM_CENTERS)
	    # # Convert centers to 8-bit integers
		# centers = np.uint8(centers)
        # # Map each pixel to its corresponding cluster center
		# segmented_image = centers[labels.flatten()]
		# segmented_image = segmented_image.reshape(frame.shape)

        # # Display the original and segmented images
		# cv.imshow('Original', frame)
		# cv.imshow('Segmented', segmented_image)

