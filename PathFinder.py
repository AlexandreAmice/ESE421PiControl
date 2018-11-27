import numpy as np
import cv2
import timeit
import warnings
import time

import picamera
from picamera.array import PiRGBArray


warnings.filterwarnings('error')


camera = picamera.PiCamera()
photoHeight = 540
image_size = (960/2, 544/2)#(16*photoHeight/9, photoHeight)
camera.resolution =  image_size#(960, 540)#(16*photoHeight/9, photoHeight)
camera.framerate = 7
camera.vflip = False
camera.hflip = False
# camera.exposure_mode='off'
rawCapture = PiRGBArray(camera, size=image_size)

# allow the camera to warmup
time.sleep(0.1)


# class for Road detection
class PathFinder():
    def __init__(self, img, removePixels = None):
        # average x values of the fitted road edge
        self.bestx = None
        self.besty = None
        # polynomial coefficients averaged over the last iterations
        self.best_fit = None
        #polynomial coefficients for the most recent fit
        self.current_fit = None
        # radius of curvature of the road edge in meters
        self.curve_rad = None
        #distance in meters of vehicle center from the line
        self.offset = None

        # x values for detected line pixels
        self.allx = None
        # y values for detected line pixels
        self.ally = None

        # camera calibration parameters (if we get these)
        self.cam_mtx = None
        self.cam_dst = None

        # camera distortion parameters (if we do this)
        self.M = None
        self.Minv = None

        # image shape
        self.im_shape = (None,None)
        # distance to look ahead in meters
        self.look_ahead = 10 #currently arbitrary
        # cut off image from top to restrict view
        self.remove_pixels = removePixels

        #windowing parameters for edge detection.
        #WARNING THESE ARE REALLY SENSITIVE TAKE CARE WHEN CHANGING
        self.window_height = 10
        self.window_width = 10
        self.margin = 20

        # enlarge output image
        self.enlarge = 2.5
        # warning from numpy polyfit
        self.poly_warning = False

        #edge coordinates
        self.edgeCoord = None

        #what side to look for road
        self.look_left = False

        #found an edge on the correct side
        self.found_edge = False

        #current image we are finding edges on
        self.cur_image = img

        #desired offset
        self.desired_offset = 4

        #white washed image
        self.whiteImg = None

    # set camera calibration parameters
    def set_cam_calib_param(self, mtx, dst):
        self.cam_mtx = mtx
        self.cam_dst = dst

        # undistort image

    def undistort(self, img):
        return cv2.undistort(img, self.cam_mtx, self.cam_dst, None, self.cam_mtx)

    def whitewash(self, img):
        '''
        change image to black and white, where black values are kept black and all others
        become white. Median blur image to contrast as stark as possible
        :param img: image to whitewash
        :return: whitewashed image in gray scale
        '''
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV) #cast to hsv since edges more defined
        h,s,v = cv2.split(img) #split the channels so we can compute on individual channels
        low_h = 50
        high_h = 100
        low_s = 0
        high_s = 50
        h_mask = cv2.inRange(h, low_h, high_h)
        s_mask = 255-cv2.inRange(s, low_s, high_s)
        h_mask = cv2.medianBlur(h_mask, 19) #large median blur to ensure that patches are very rare
        s_mask = cv2.medianBlur(s_mask, 19)  #large median blur to ensure that patches are very rare
        tempS = np.zeros_like(s_mask)
        tempS[np.nonzero(s_mask)] = 1 #for binary computation
        tempH = np.zeros_like(s_mask)
        tempH[np.nonzero(h_mask)] = 1 #for binary computation
        res = tempS & tempH #take only the parts that thresholded white in both H and S
        res[np.nonzero(res)] = 255 #back to value of 255
        res = cv2.Sobel(res, cv2.CV_8U, 1, 0, ksize=3)
        res = cv2.Sobel(res, cv2.CV_8U, 0, 1, ksize=3)
        #cv2.imshow('gray.jpg', res)
        #cv2.imshow('gradX.jpg', res)
        #cv2.imshow('gradY.jpg', res)
        #cv2.imshow('hWash.jpg', h_mask)
        #cv2.imshow('sWash.jpg', s_mask)
        cv2.imshow('whiteWash.jpg', res)
        self.whiteImg = res

        return res

    def whitewash2(self):
        temp = cv2.GaussianBlur(self.cur_image, (13, 13), 3)
        r, g, b = cv2.split(temp)
        subtract = r-g-b
        thresh = cv2.inRange(subtract, 0, 80)
        thresh = cv2.merge((r, thresh, b))
        thresh = cv2.cvtColor(thresh, cv2.COLOR_RGB2GRAY)
        self.whiteImg = thresh
        return thresh


    def collect_edge(self, img, lowThresh, highThresh, window_height = 10, window_width = 10, margin = 40):
        '''
        collect edge
        :param img: Source image
        :param threshold: threshold above which potentially an edge
        :return: collected edge
        '''
        whitewash = self.whitewash(img)
        xs,ys = self.find_window_centroids(whitewash, window_width, window_height, margin)
        xs.reverse()
        #be sure that we found an edge

        if not self.found_edge:
            self.edge = None
            return None

        temp = np.zeros_like(whitewash)
        self.edge = temp
        for i in range(0,len(ys)):
            self.edge[ys[i]-margin:ys[i]+margin,xs[i]-margin:xs[i]+margin] = \
                whitewash[ys[i]-margin:ys[i]+margin,xs[i]-margin:xs[i]+margin]
        cv2.imshow('edge.jpg', self.edge)
        return xs, ys


        # find widow centroids of left and right lane

    def find_window_centroids(self, image, window_width, window_height, margin):
        '''
        iterate up along image to find where the road likely is
        :param image: image to find roads
        :param window_width: width of boxes to box road
        :param window_height:  width of boxes to box road
        :param margin: margin off from past center we can be
        :return: x,y pairs of the
        '''
        self.found_edge = False
        window_centroids = []  # Store the window centroid positions per level
        if self.look_left:
            window = np.array([float(i**2)/window_width for i in range(0, window_width)])
        else:
            window = np.array([float(i**2) / window_width for i in range(window_width, 0, -1)])
        #window = np.ones(window_width)  # Create our window template that we will use for convolutions

        # First find the two starting positions for the left and right lane by using np.sum to get the vertical image slice
        # and then np.convolve the vertical image slice with the window template

        # Sum bottom ratio of image
        ratio = 1 / 4
        cv2.imwrite('partialL.jpg', image[int(ratio * image.shape[0]):, :int(image.shape[1] / 2)])
        cv2.imwrite('partialR.jpg', image[int(image.shape[0] * ratio):, int(image.shape[1] / 2):])
        level = 0
        while not self.found_edge and level < (int)(image.shape[0] / window_height):
            if self.look_left:
                totSum = np.sum(image[int(ratio * (image.shape[0] - (level + 1)) * window_height):, :int(image.shape[1] / 2)], axis=0)
                center = np.argmax(np.convolve(window, totSum)) - window_width / 2
                if sum(totSum) > 20:
                    self.found_edge = False
            else:
                totSum = np.sum(image[int(image.shape[0] * ratio):, int(image.shape[1] / 2):], axis=0)
                center = np.argmax(np.convolve(window, totSum)) - window_width / 2 + int(image.shape[1] / 2)
                if sum(totSum) > 20:
                    self.found_edge = False
            level += 1



        # Add what we found for the first layer
        window_centroids.append(center)

        # Go through each layer looking for max pixel locations
        while level < (int)(image.shape[0] / window_height):
            # convolve the window into the vertical slice of the image
            image_layer = np.sum(image[int(image.shape[0] - (level + 1) * window_height):int(
                image.shape[0] - level * window_height), :], axis=0)
            conv_signal = np.convolve(window, image_layer)
            # Find the best centroid by using past center as a reference
            # Use window_width/2 as offset because convolution signal reference is at right side of window, not center of window
            offset = window_width / 2
            min_index = int(max(center + offset - margin, 0))
            max_index = int(min(center + offset + margin, image.shape[1]))
            center = np.argmax(conv_signal[min_index:max_index]) + min_index - offset
            center = center

            # Add what we found for that layer
            window_centroids.append(center)
        #make sure we actually found a road
        self.found_edge = sum(totSum) > 255 * 50
        ys = range(1, (int)(image.shape[0] / window_height)+1)
        ys = [i * window_height - 1 for i in ys]
        #need to reverse the x because of convolution doing things backward
        return window_centroids, ys



    def get_fit2(self, image):
        '''
        get the road edge fit
        :param image: image to find road in
        :return:
        '''
        temp = self.collect_edge(image, 180, 255,self.window_width, self.window_height, self.margin)
        if temp is None:
            return
        self.allx,self.ally = temp[0], temp[1]
        if self.bestx is not None and self.besty is not None:
            self.bestx, self.besty = (0.6*self.bestx + 0.4*np.array(self.allx)).astype(int), (0.6*self.besty + 0.4*np.array(self.ally)).astype(int)
        else:
            self.bestx, self.besty = np.array(self.allx), np.array(self.ally)

        if (len(self.allx) > 0):
            try:
                self.current_fit = np.polyfit(self.ally, self.allx, 1)
                self.best_fit = self.current_fit
            except np.RankWarning:
                self.poly_warning = True

    
    #TODO: FIX THIS
    def calculate_curvature_offset(self):
        '''
        Calculate the offset from the estimated middle of the curved line using a 2nd degree polynomial.
        Make sure this function assigns values to self.left_curverad, right.right_curverad, self.offset
        '''
        if self.found_edge:
            # define y value near the car
            y_eval = self.im_shape[0]

            # define conversions in x and y from pixels space to meters
            ym_per_pix = 50 / 250.0  # meters per pixel in y dimension
            xm_per_pix = 3.7 / 75.0  # meters per pixel in x dimension

            # create new polynomials to x,y in world space
            try:
                fit_cr = np.polyfit( self.besty * ym_per_pix,  self.bestx * xm_per_pix, 2)
            except np.RankWarning:
                self.poly_warning = True
                pass

            # if the poly fit is ok proceed
            if not self.poly_warning:
                try:
                # calculate the new radii of curvature
                    curve_rad= ((1 + (
                                2 * fit_cr[0] * y_eval * ym_per_pix + fit_cr[1]) ** 2) ** 1.5) / np.absolute(
                        2 * fit_cr[0])

                    # now our radius of curvature is in meters

                    # calculate the offset from the edge of the road
                    y_eval = y_eval * ym_per_pix
                    midpoint_car = self.im_shape[1] / 2.0
                    midpoint_lane = (fit_cr[0] * (y_eval ** 2) + fit_cr[1] * y_eval + fit_cr[2])

                    offset = midpoint_car * xm_per_pix - midpoint_lane / 2

                    # initialize the curvature and offset if this is the first detection
                    if self.curve_rad == None:
                        self.offset = offset

                    # average out the offset
                    else:
                        self.offset = self.offset * 0.9 + offset * 0.1
                except:
                    pass

    def project_on_road(self):
        image_input = self.cur_image
        image = image_input[self.remove_pixels:,:]
        #image = self.whitewash(image) REFACTOR PUT THIS IN COLLECT EDGE
        self.im_shape = image.shape
        self.get_fit2(image)
        #cv2.imwrite('edge.jpg', self.edge)
        # create fill image
        temp_filler = np.zeros((self.remove_pixels, self.im_shape[1])).astype(np.uint8)
        filler = np.dstack((temp_filler, temp_filler, temp_filler))
        if self.found_edge:

            # combine the result with the original image
            rEdge = np.zeros_like(self.edge)
            gEdge = np.zeros_like(self.edge)
            bEdge = np.zeros_like(self.edge)
            rEdge[np.nonzero(self.edge)] = 255
            rEdge[np.nonzero(self.edge)] = 180
            edgeStack = np.dstack((bEdge,gEdge,rEdge))
            edgeFill = np.vstack((filler, edgeStack))

            result = cv2.addWeighted(edgeFill, 1, image_input, 1, 0) #edgeFill = filled
            y1 = min(self.besty)
            y2 = max(self.besty)

            #draw the line onto the screen
            point1 = (int(self.best_fit[0] * y1 + self.best_fit[1]), y1+ self.remove_pixels)
            point2 = (int(self.best_fit[0] * y2 + self.best_fit[1]), y2 + self.remove_pixels)
            cv2.line(result, point1, point2, [255, 0, 0], thickness=5)

            # get curvature and offset
            self.calculate_curvature_offset()

            # plot text on resulting image

            if self.look_left:
                img_text = "vehicle is: " + str(round(np.abs(self.offset), 2)) + ' (m) right of edge'
            else:
                img_text = "vehicle is: " + str(round(np.abs(self.offset), 2)) + ' (m) left of edge'

            cv2.putText(result, img_text, (15, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            whiteDstack = np.dstack((self.whiteImg, self.whiteImg, self.whiteImg))
            fill = np.vstack((filler, whiteDstack))
            #result = np.hstack((result, fill))
            return result

            # if lanes were not detected output source image
        else:
            #print 'no edge detected'
            whiteDstack = np.dstack((self.whiteImg,self.whiteImg,self.whiteImg))
            fill = np.vstack((filler,whiteDstack))
            return_image =  np.hstack((image_input, fill))
            return return_image



    def calc_phi_d(self):
        image = self.cur_image[self.remove_pixels:, :]
        self.get_fit2(self.cur_image)
        if not self.found_edge:
            return 20*(-self.look_left*1 + (not self.look_left)*1)
        else:
            self.get_fit2(self.cur_image)
            self.calculate_curvature_offset()
            print self.offset
            print self.desired_offset
            print (self.offset - self.desired_offset)
            return 5 * (self.offset-self.desired_offset) * (-self.look_left*1 + (not self.look_left)*1)

    def setLookLeft(self, val):
        self.look_left = val



        #############################################################################

#while True:
        '''
image = cv2.imread('blackRoad.jpg')
roadEdge = PathFinder(image)
roadEdge.look_ahead = 10
roadEdge.remove_pixels = 100
roadEdge.enlarge = 0.5  # 2.25
image = cv2.GaussianBlur(image, (3,3),1)
roadEdge.look_left = False
result = roadEdge.project_on_road()
cv2.imwrite('result.jpg',result)
'''
if __name__ == "__main__":
    roadEdge = PathFinder(None, 90)

    time.sleep(0.1)

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        start  = timeit.timeit()
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        

        # show the frame
        # lines.project_on_road_debug(image)
        roadEdge.cur_image = image
        cv2.imshow("Rpi lane detection", roadEdge.project_on_road())
        key = cv2.waitKey(1) & 0xFF

        # clear the stream in preparation for the next frame
        rawCapture.truncate()
        rawCapture.seek(0)
        end = timeit.timeit()
 

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
