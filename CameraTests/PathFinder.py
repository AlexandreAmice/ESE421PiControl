import numpy as np
import cv2
import time
import warnings


image_size=(320, 192)

# class for Road detection
class Road_Edge():
    def __init__(self):
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
        low_black = np.array([80, 0, 0])
        high_black = np.array([120, 255, 255])
        mask = cv2.inRange(img, low_black, high_black)
        mask = cv2.medianBlur(mask, 33) #large median blur to ensure that patches are very rare
        cv2.imwrite('whiteWash.jpg', mask )
        return mask

    def collect_edge(self, img, lowThresh, highThresh, window_height = 40, window_width = 40, margin = 10):
        '''
        collect edge
        :param img: source image
        :param threshold: threshold above which potentially an edge
        :return: collected edge
        '''
        canny = cv2.Canny(img, highThresh, lowThresh)
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
        ratio = 11/12.0
        l_sum = np.sum(image[int(ratio * image.shape[0] ):, :int(image.shape[1] / 2)], axis=0)
        l_center = np.argmax(np.convolve(window, l_sum)) - window_width / 2
        r_sum = np.sum(image[int( image.shape[0] * ratio):, int(image.shape[1] / 2):], axis=0)
        r_center = np.argmax(np.convolve(window, r_sum)) - window_width / 2 + int(image.shape[1] / 2)
        max_sum_flag = max(sum(l_sum),sum(r_sum))
        #one with biggest sum likely has the road
        if max_sum_flag == sum(l_sum):
            max_center = l_center
            max_sum = l_sum
        else:
            max_center = r_center
            max_sum = r_sum

        # Add what we found for the first layer
        window_centroids.append(max_center)

        # Go through each layer looking for max pixel locations
        for level in range(1, (int)(image.shape[0] / window_height)):
            # convolve the window into the vertical slice of the image
            image_layer = np.sum(image[int(image.shape[0] - (level + 1) * window_height):int(
                image.shape[0] - level * window_height), :], axis=0)
            conv_signal = np.convolve(window, image_layer)
            # Find the best centroid by using past center as a reference
            # Use window_width/2 as offset because convolution signal reference is at right side of window, not center of window
            offset = window_width / 2
            min_index = int(max(max_center + offset - margin, 0))
            max_index = int(min(max_center + offset + margin, image.shape[1]))
            center = np.argmax(conv_signal[min_index:max_index]) + min_index - offset
            max_center = center

            # Add what we found for that layer
            window_centroids.append(center)

        return window_centroids

        # create window mask for lane detecion
    def window_mask(self, width, height, img_ref, center, level):
        output = np.zeros_like(img_ref)
        output[int(img_ref.shape[0] - (level + 1) * height):int(img_ref.shape[0] - level * height), \
        max(0, int(center - width / 2)):min(int(center + width / 2), img_ref.shape[1])] = 1
        return output

    def get_fit(self, image):
        '''
        get the road edge fit
        :param image: image to find road in
        :return:
        '''
        if not self.detected:
            #window setting
            wind_width = 5
            wind_height = 5
            margin = 5 #how much road can deviate ahead
            window_centroids = self.find_window_centroids(image, wind_width, wind_height, margin)

            if len(window_centroids) > 0: #make sure we found a line
                window_points = np.zeros_like(image) #for storing window points

                #iterate through all the bounding boxes of the edge
                for level in range(0, len(window_centroids)):
                    # Window_mask is a function to draw window areas
                    mask = self.window_mask(wind_width, wind_height, image, window_centroids[level], level)
                    # r_mask = self.window_mask(window_width,window_height,image,window_centroids[level][1],level)
                    # Add graphic points from window mask here to total pixels found

                    window_points[(image == 255) & (mask == 1)] = 1

                # construct images of the results
                template = np.array(window_points * 255, np.uint8)  # add left window pixel
                zero_channel = np.zeros_like(template)  # create a zero color channel
                edge = np.array(cv2.merge((template, zero_channel,template)), np.uint8)  # make the lane edge
                # get points for polynomial fit
                self.ally, self.allx = window_points.nonzero()

                if len(self.allx) > 0:
                    try:
                        self.current_fit = np.polyfit(self.ally, self.allx, 2)
                        self.poly_warning = False
                    except:
                        self.poly_warning = True

                    # check if lanes are detected correctly
                    if self.check_fit():
                        self.detected = True

                        # if this is the first detection initialize the best values
                        if not self.detected_first:
                            self.best_fit = self.current_fit
                        # if not then average with new
                        else:
                            self.best_fit = self.best_fit * 0.6 + self.current_fit * 0.4 

                        # assign new best values based on this iteration
                        self.detected_first = True
                        self.edge = edge
                        self.bestx = self.allx
                        self.besty = self.ally

                    # set flag if lanes are not detected correctly
                    else:
                        self.detected = False

                 # if lanes were detected in the last frame, search area for current frame
                else:

                    non_zero_y, non_zero_x = image.nonzero()

                    margin = 10  # search area margin
                    lane_points_idx = ((non_zero_x > (
                                self.best_fit[0] * (non_zero_y ** 2) + self.best_fit[1] * non_zero_y +
                                self.best_fit[2] - margin)) & (non_zero_x < (
                                self.best_fit[0] * (non_zero_y ** 2) + self.best_fit[1] * non_zero_y +
                                self.best_fit[2] + margin)))

                    # extracted lane pixels
                    self.allx = non_zero_x[lane_points_idx]
                    self.ally = non_zero_y[lane_points_idx]


                    # if lines were found
                    if (len(self.allx) > 0):
                        try:
                            self.current_fit = np.polyfit(self.ally, self.allx, 2)
                        except np.RankWarning:
                            self.poly_warning = True

                        # check if lanes are detected correctly
                        if self.check_fit():
                            # average out the best fit with new values
                            self.best_fit = self.best_fit * 0.6 + self.current_fit * 0.4
                            # self.best_fit_r = self.best_fit_r*0.6 + self.current_fit_r * 0.4

                            # assign new best values based on this iteration
                            self.bestx = self.allx
                            self.besty = self.ally


                            # construct images of the results
                            template = np.copy(image).astype(np.uint8)

                            template[non_zero_y[lane_points_idx], non_zero_x[
                                lane_points_idx]] = 255  # add left window pixels
                            zero_channel = np.zeros_like(template)  # create a zero color channel
                             # make color image left and right lane
                            self.edge = np.array(cv2.merge((template, zero_channel,template)), np.uint8)
                            #cv2.imwrite('edge.jpg', self.edge)
                            # set flag if lanes are not detected correctly
                        else:
                            self.detected = False



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
        self.allx,self.ally = self.collect_edge(image,20, 80,wind_height, wind_width, margin)

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
        #self.edge = np.array(cv2.merge((template, zero_channel, template)), np.uint8)


    def project_on_road(self, image_input):
        image = image_input[self.remove_pixels:,:]
        image = self.whitewash(image)
        self.im_shape = image.shape
        self.get_fit2(image)
        #cv2.imwrite('edge.jpg', self.edge)
        if self.detected_first & self.detected:
            # create fill image
            temp_filler = np.zeros((self.remove_pixels, self.im_shape[1])).astype(np.uint8)
            filler = temp_filler

            # create an image to draw the lines on
            warp_zero = np.zeros_like(image).astype(np.uint8)
            color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

            ploty = np.linspace(0, image_input.shape[0] - 1, image_input.shape[0])
            fit = self.best_fit[0] * ploty ** 2 + self.best_fit[1] * ploty + self.best_fit[2]


            # recast the x and y points into usable format for cv2.fillPoly()
            pts_left = np.array([np.transpose(np.vstack([fit, ploty]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([fit, ploty])))])
            pts = np.hstack((pts_left, pts_right))

            # draw the lane onto the warped blank image
            cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))


            # combine the result with the original image
            filled = cv2.cvtColor(np.vstack((filler, self.edge)), cv2.COLOR_GRAY2RGB)
            cv2.imwrite('filled.jpg', filled)
            result = cv2.addWeighted(filled, 1, image_input, 1, 0)
            for idx1, point1 in enumerate(self.edgeCoord):
                for idx2, point2 in enumerate(self.edgeCoord):
                    point1 = self.edgeCoord[idx1]
                    if idx1 != idx2:
                        temp1 = (point1[0], point1[1] + self.remove_pixels)
                        temp2 = (point2[0], point2[1] + self.remove_pixels)
                        cv2.line(result, temp1, temp2, [255,0,0])
            '''
            # get curvature and offset
            self.calculate_curvature_offset()

            # plot text on resulting image
            img_text = 'None'  # "radius of curvature: " + str(round((self.left_curverad + self.right_curverad)/2,2)) + ' (m)'

            if self.offset < 0:
                img_text2 = "vehicle is: " + str(round(np.abs(self.offset), 2)) + ' (m) left of center'
            else:
                img_text2 = "vehicle is: " + str(round(np.abs(self.offset), 2)) + ' (m) right of center'

            small = cv2.resize(left_right_fill, (0, 0), fx=0.5, fy=0.5)
            small2 = cv2.resize(np.vstack((filler, self.left_right)), (0, 0), fx=0.5, fy=0.5)

            result2 = cv2.resize(np.hstack((result, np.vstack((small2, small)))), (0, 0), fx=self.enlarge,
                                 fy=self.enlarge)
            # result2 = cv2.resize(np.hstack((np.vstack((filler,np.dstack((self.binary_image*255,self.binary_image*255,self.binary_image*255)))), np.vstack((small2,small)))), (0,0), fx=self.enlarge, fy=self.enlarge)

            cv2.putText(result2, img_text, (15, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(result2, img_text2, (15, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            '''
            print 'plotted'
            return result

            # if lanes were not detected output source image
        else:
            print 'no edge detected'
            return_image = cv2.resize(
                np.hstack((image_input, cv2.resize(np.zeros_like(image_input), (0, 0), fx=0.5, fy=1.0))), (0, 0),
                fx=self.enlarge, fy=self.enlarge)
            return return_image



        #############################################################################
roadEdge = Road_Edge()
roadEdge.look_ahead = 10
roadEdge.remove_pixels = 300
roadEdge.enlarge = 0.5  # 2.25
while True:
    image = cv2.imread('blackRoad.jpg')
    image = cv2.GaussianBlur(image, (7,7),5)
    result = roadEdge.project_on_road(image)
    cv2.imwrite('Result.jpg', result)
    # show the frame
    #cv2.imwrite('Road.jpg', lines.project_on_road_debug(image))
    time.sleep(0.1)
    key = cv2.waitKey(1) & 0xF
    if key == ord("q"):
        break