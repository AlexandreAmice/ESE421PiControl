import numpy as np
import math
import cv2
import time
import warnings
from scipy import stats

#import picamera
#from picamera.array import PiRGBArray
warnings.filterwarnings('error')

class FindEdge():
    def __init__(self, img, removePixels = None):
        #Best fit points for left and right edges
        self.bestxL = None
        self.bestyL = None
        self.bestxR = None
        self.bestyR = None
        self.bestLineL = None
        self.bestLineR = None

        #polynomial coefficients of best guess for desired edge
        self.best_fit = None

        # camera calibration parameters (if we get these)
        self.cam_mtx = None
        self.cam_dst = None

        #current image
        self.cur_image = img
        self.x_dim = img.shape[0]
        self.y_dim = img.shape[1]
        self.trim_top = removePixels
        self.trim_left = 0
        self.trim_right = self.cur_image
        self.cropped_image = img[self.trim_top:, :]


        # windowing parameters for edge detection.
        # WARNING THESE ARE REALLY SENSITIVE TAKE CARE WHEN CHANGING
        self.window_height = 10
        self.window_width = 10
        self.margin = 20

        #what side of the road we want to look for
        self.look_left = False

        #what side we can see the edge
        self.found_left = False
        self.found_right = False

        #processed image for edge detection
        self.proc_img = None
        self.edges = None

        #green saturation
        self.green_L_sat = 0
        self.green_R_sat = 0


    def set_new_image(self, image):
        self.cur_image = image
        self.cropped_image = self.cur_image[self.trim_top:, :]
        self.x_dim = image.shape[0]
        self.y_dim = image.shape[1]


    def set_proc_img_left(self):
        '''
        process the cropped image in the hope of finding an edge
        :param self:
        :return:
        '''
        temp = cv2.medianBlur(self.cur_image[self.trim_top:, self.trim_left:self.cur_image.shape[0] / 2], 13)
        r, g, b = cv2.split(temp)
        subtract = r - g - b
        thresh = cv2.inRange(subtract, 0, 80)
        thresh = cv2.merge((r, thresh, b))
        thresh = cv2.medianBlur(thresh, 21)
        self.edges = cv2.Canny(thresh, 100, 200)
        self.proc_img = thresh




        #write images to file for debugging
        cv2.imwrite('r.jpg', r)
        cv2.imwrite('g.jpg', g)
        cv2.imwrite('b.jpg', g)
        cv2.imwrite('subtract.jpg', subtract)

    def get_green_L_sat(self):
        pass



    def get_green_R_sat(self):
        pass


    def collect_edge_R(self):
        curTheta = 0
        # origin of picture is top left corner. Shift line parameters by these coordinates in order to have the scanning line
        #  and the image lines in the same coordinate system.
        anchor = np.array([self.x_dim/2, self.y_dim])
        #tolerance for determining if a point is on a line
        epsilon = 5

        edge_points_x = []
        edge_points_y = []

        while curTheta < math.pi/2:
            m = math.tan(curTheta)
            on_line = lambda point: m*(point[0]-anchor[0]) <= (point[1]-anchor[1])+epsilon and m*(point[0]-anchor[0]) >= (point[1]-anchor[1])-epsilon
            min_dist = (np.inf, np.inf)
            best_point = np.array[-1,-1]
            for point in np.nonzero(self.proc_img):
                if on_line(point) and (np.linalg.norm(point-anchor) < min_dist):
                    best_point = point
                    min_dist = np.linalg.norm(point-anchor)

            edge_points_x.append(best_point[0])
            edge_points_y.append(best_point[1])
            curTheta += 0.1

        slope, intercept, r_value, p_value, std_err = stats.linregress(edge_points_x, edge_points_y)
        #this parameter needs tuning but as the loop continues we want the accepted thresh to drop
        acceptedThresh = 20
        minPoints = 10
        while r_value < 0.99:
            new_points_x = []
            new_points_y = []
            for i, x in enumerate(edge_points_x):
                curY = edge_points_y[i]
                yhat = slope*x - intercept
                residual = abs(curY - yhat)
                if residual < acceptedThresh:
                    new_points_x.append(x)
                    new_points_y.append(curY)
            edge_points_x = new_points_x
            edge_points_y = new_points_y
            acceptedThresh -= 0.5
            if len(edge_points_x > minPoints):
                slope, intercept, r_value, p_value, std_err = stats.linregress(edge_points_x, edge_points_y)


        self.bestLineR = (slope, intercept)
        self.bestxR = edge_points_x
        self.bestyR = edge_points_y
        #Algorithm:
        #create a ray starting at angle theta = 0
        #create list of points
        #while theta < pi/2
            #rotate ray by dtheta amount
            #find point on ray that intersects nearest edge
            #store point

        #do while loop of the following
        # find a linear fit of points
        #calculate residual of each points
            #if residual > threshold
                #discard point
        #calculate new fit

        #return the accepted points and the fit
        pass

    def collect_edge_L(self):
        pass

    def draw_r_edge(self):
        x1 = min(self.bestxR)
        x2 = max(self.bestxR)
        (slope, intercept)= self.bestLineR

