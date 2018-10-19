import numpy as np
import cv2
import time
import warnings


image_size=(320, 192)

# class for Road detection
class Road_Edge():
    def __init__(self, img):
        # were the RoadEdge detected at least once
        self.detected_first = False
        # were the RoadEdge detected in the last iteration?
        self.detected = False
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
        self.remove_pixels = 300
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

        self.cur_image = img

        #canny edge image
        self.canny_collect = None

        #desired offset
        self.desired_offset = 4

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
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        h,s,v = cv2.split(img)
        v = np.zeros_like(v)
        low_h = 50 #np.array([50, 0, 0])
        high_h = 100 #np.array([100, 255, 255])
        low_s = 25
        high_s = 70
        h_mask = cv2.inRange(h, low_h, high_h)
        s_mask = 255-cv2.inRange(s, low_s, high_s)
        h_mask = cv2.medianBlur(h_mask, 5) #large median blur to ensure that patches are very rare
        s_mask = cv2.medianBlur(s_mask, 5)  # large median blur to ensure that patches are very rare
        res = cv2.merge([h_mask,s_mask,v])
        res = cv2.Sobel(h_mask, cv2.CV_8U, 1, 0, ksize=3)
        res = cv2.Sobel(res, cv2.CV_8U, 0, 1, ksize=3)
        cv2.imwrite('gradX.jpg', res)
        cv2.imwrite('gradY.jpg', res)

        cv2.imwrite('hWash.jpg', h_mask)
        cv2.imwrite('sWash.jpg', s_mask)
        cv2.imwrite('whiteWash.jpg', res)

        return res

    def dir_threshold(self, sobelx, sobely, thresh=(0, np.pi / 2)):

        abs_sobelx = np.abs(sobelx)
        abs_sobely = np.abs(sobely)
        grad_sobel = np.arctan2(abs_sobely, abs_sobelx)

        sbinary = np.zeros_like(grad_sobel)
        sbinary[(grad_sobel >= thresh[0]) & (grad_sobel <= thresh[1])] = 1

        return sbinary

        # get binary image based on sobel gradient thresholding
    def abs_sobel_thresh(self, sobel, thresh=(0, 255)):
        abs_sobel = np.absolute(sobel)

        max_s = np.max(abs_sobel)
        if max_s == 0:
            max_s = 1

        scaled_sobel = np.uint8(255 * abs_sobel / max_s)

        sbinary = np.zeros_like(scaled_sobel)

        sbinary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1

        return sbinary

    def collect_edge(self, img, lowThresh, highThresh, window_height = 40, window_width = 40, margin = 10):
        '''
        collect edge
        :param img: source image
        :param threshold: threshold above which potentially an edge
        :return: collected edge
        '''

        canny = cv2.Canny(img, highThresh, lowThresh)
        self.canny_collect = canny
        cv2.imwrite('cannyCollect.jpg', img)
        xs = self.find_window_centroids(canny, window_width, window_height, margin)
        xs.reverse()
        ys = range(1, (int)(img.shape[0] / window_height)+1)
        ys = [i * window_height - 1 for i in ys]
        temp = np.zeros_like(img)
        self.edge = temp
        self.edge[ys,xs] = 255
        self.edgeCoord = np.transpose(np.array([xs, ys]))
        cv2.imwrite('edge.jpg', self.edge)
        return xs, ys


        # find widow centroids of left and right lane

    def find_window_centroids(self, image, window_width, window_height, margin):
        '''
        iterate up along image to find where the road likely is
        :param image: image to find roads
        :param window_width: width of boxes to box road
        :param window_height:  width of boxes to box road
        :param margin: margin off from past center we can be
        :return: a set of windows that contain the line
        '''
        window_centroids = []  # Store the (left,right) window centroid positions per level
        window = np.ones(window_width)  # Create our window template that we will use for convolutions

        # First find the two starting positions for the left and right lane by using np.sum to get the vertical image slice
        # and then np.convolve the vertical image slice with the window template

        # Sum bottom ratio of image
        ratio = 11 / 12.0
        if self.look_left:
            totSum = np.sum(image[int(ratio * image.shape[0]):, :int(image.shape[1] / 2)], axis=0)
            center = np.argmax(np.convolve(window, totSum)) - window_width / 2
            if sum(totSum) < 20:
                self.found_edge = False
        else:
            totSum = np.sum(image[int( image.shape[0] * ratio):, int(image.shape[1] / 2):], axis=0)
            center = np.argmax(np.convolve(window, totSum)) - window_width / 2 + int(image.shape[1] / 2)
            if sum(totSum) < 20:
                self.found_edge = False



        # Add what we found for the first layer
        window_centroids.append(center)

        # Go through each layer looking for max pixel locations
        for level in range(1, (int)(image.shape[0] / window_height)):
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

        return window_centroids

        # create window mask for lane detecion

    def window_mask(self, width, height, img_ref, center, level):
        output = np.zeros_like(img_ref)
        output[int(img_ref.shape[0] - (level + 1) * height):int(img_ref.shape[0] - level * height), \
        max(0, int(center - width / 2)):min(int(center + width / 2), img_ref.shape[1])] = 1
        return output


    def check_fit(self):
        # Generate x and y values of the fit
        ploty = np.linspace(0, self.im_shape[0]-1, self.im_shape[0])
        fitx = self.current_fit[0]*ploty**2 + self.current_fit[1]*ploty + self.current_fit[2]

        # find max, min and mean distance between the lanes
        max_dist  = np.amax(np.abs(fitx))
        min_dist  = np.amin(np.abs(fitx))
        mean_dist = np.mean(np.abs(fitx))
        return True #temporary always say fit is fine
        # check if the lanes don't have a big deviation from the mean
        if (max_dist > 250) |  (np.abs(max_dist - mean_dist)> 100) | (np.abs(mean_dist - min_dist) > 100) | (mean_dist<50) | self.poly_warning:
            return False
        else:
            return True

    def get_fit2(self, image):
        '''
        get the road edge fit
        :param image: image to find road in
        :return:
        '''
        # window setting
        wind_width = 5
        wind_height = 5
        margin = 25  # how much road can deviate ahead
        self.allx,self.ally = self.collect_edge(image,180, 255,wind_height, wind_width, margin)
        self.bestx, self.besty = np.array(self.allx), np.array(self.ally)

        if (len(self.allx) > 0):
            try:
                self.current_fit = np.polyfit(self.ally, self.allx, 2)
                self.best_fit = self.current_fit
            except np.RankWarning:
                self.poly_warning = True
        self.detected = True
        self.detected_first = True
        template = np.array(image, np.uint8)  # add left window pixel
        zero_channel = np.zeros_like(template)  # create a zero color channel
        self.edge = np.array(cv2.merge((template, zero_channel, template)), np.uint8)
    
    
    def calculate_curvature_offset(self):
        '''
        Calculate the offset from the estimated middle of the curved line using a 2nd degree polynomial.
        Make sure this function assigns values to self.left_curverad, right.right_curverad, self.offset
        '''
        if self.detected_first:
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
                    self.curve_rad = curve_rad
                    self.offset = offset

                # average out the curvature and offset
                else:
                    self.curve_rad = self.curve_rad * 0.8 + curve_rad * 0.2
                    self.offset = self.offset * 0.9 + offset * 0.1

    def project_on_road(self):
        image_input = self.cur_image
        image = image_input[self.remove_pixels:,:]
        image = self.whitewash(image)
        self.im_shape = image.shape
        self.get_fit2(image)
        #cv2.imwrite('edge.jpg', self.edge)
        if self.detected_first & self.detected:
            # create fill image
            temp_filler = np.zeros((self.remove_pixels, self.im_shape[1])).astype(np.uint8)
            filler = np.dstack((temp_filler, temp_filler, temp_filler))

            # create an image to draw the lines on
            warp_zero = np.zeros_like(image).astype(np.uint8)
            color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

            ploty = np.linspace(0, image_input.shape[0] - 1, image_input.shape[0])
            fit = self.best_fit[0] * ploty ** 2 + self.best_fit[1] * ploty + self.best_fit[2]


            # recast the x and y points into usable format for cv2.fillPoly()
            pts1 = np.array([np.transpose(np.vstack([fit, ploty]))])
            pts2 = pts1
            pts = np.hstack((pts1, pts2))


            # draw the lane onto the warped blank image
            cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))


            # combine the result with the original image
            edgeFill = np.vstack((filler, self.edge))
            #filled = cv2.cvtColor(np.vstack((filler, self.edge)), cv2.COLOR_GRAY2RGB)
            #cv2.imwrite('filled.jpg', filled)
            result = cv2.addWeighted(edgeFill, 1, image_input, 1, 0) #edgeFill = filled
            for idx1, point1 in enumerate(self.edgeCoord):
                for idx2, point2 in enumerate(self.edgeCoord):
                    point1 = self.edgeCoord[idx1]
                    if idx1 != idx2:
                        temp1 = (point1[0], point1[1] + self.remove_pixels)
                        temp2 = (point2[0], point2[1] + self.remove_pixels)
                        cv2.line(result, temp1, temp2, [255,0,0])
            # get curvature and offset
            self.calculate_curvature_offset()

            # plot text on resulting image

            if self.look_left:
                img_text = "vehicle is: " + str(round(np.abs(self.offset), 2)) + ' (m) right of edge'
            else:
                img_text = "vehicle is: " + str(round(np.abs(self.offset), 2)) + ' (m) left of edge'

            cv2.putText(result, img_text, (15, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            print 'plotted'
            return result

            # if lanes were not detected output source image
        else:
            print 'no edge detected'
            return_image = cv2.resize(
                np.hstack((image_input, cv2.resize(np.zeros_like(image_input), (0, 0), fx=0.5, fy=1.0))), (0, 0),
                fx=self.enlarge, fy=self.enlarge)
            return return_image


    def can_find_edge(self):
        window_width = 5
        window_height = 5
        margin = 25
        window = np.ones(window_width)  # Create our window template that we will use for convolutions

        # First find the two starting positions for the left and right lane by using np.sum to get the vertical image slice
        # and then np.convolve the vertical image slice with the window template

        # Sum bottom ratio of image
        ratio = 1/4
        if self.look_left:
            totSum = np.sum(self.canny_collect[int(ratio * self.canny_collect.shape[0]):, :int(self.canny_collect.shape[1] / 2)], axis=0)
            center = np.argmax(np.convolve(window, totSum)) - window_width / 2
            return sum(totSum) > 20000
        else:
            totSum = np.sum(self.canny_collect[int(self.canny_collect.shape[0] * ratio):, int(self.canny_collect.shape[1] / 2):], axis=0)
            center = np.argmax(np.convolve(window, totSum)) - window_width / 2 + int(self.canny_collect.shape[1] / 2)
            return sum(totSum) > 20000

    def calc_phi_d(self):
        self.found_edge = self.can_find_edge()
        if not self.found_edge:
            return 20*(-self.look_left*1 + (not self.look_left)*1)
        else:
            self.get_fit2(self.cur_image)
            self.calculate_curvature_offset()
            print self.offset
            print self.desired_offset
            print (self.offset - self.desired_offset)
            return 5 * (self.offset-self.desired_offset) * (-self.look_left*1 + (not self.look_left)*1)




        #############################################################################

#while True:
image = cv2.imread('blackRoad.jpg')
roadEdge = Road_Edge(image)
roadEdge.look_ahead = 10
roadEdge.remove_pixels = 300
roadEdge.enlarge = 0.5  # 2.25
image = cv2.GaussianBlur(image, (3,3),1)
roadEdge.look_left = False
result = roadEdge.project_on_road()
print roadEdge.calc_phi_d()
cv2.imwrite('Result.jpg', result)
# show the frame
#cv2.imwrite('Road.jpg', lines.project_on_road_debug(image))
time.sleep(0.1)
key = cv2.waitKey(1) & 0xF
    #if key == ord("q"):
    #    break