#!/usr/bin/env python3

from audioop import avg
import cv2
import numpy as np
from matplotlib import pyplot as plt
import math
import glob
import os
import rospy
import time
import ros_numpy
from sensor_msgs.msg import Image

from sklearn.cluster import KMeans


# from: https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html

class input_params(object):
    pass

class CannyEdgeDetector:
    def __init__(self):
        
        params = input_params()
        #MUST have the following paramterss
        try:
            self.image_topic = rospy.get_param('/pipe_detector/image_topic')
            self.canny_edge_threshold_1 = rospy.get_param("/pipe_detector/canny_edge_threshold_1")
            self.morp_kernel_val = rospy.get_param("/pipe_detector/morp_kernel_val")
            self.canny_edge_threshold_1 = rospy.get_param("/pipe_detector/canny_edge_threshold_1")
            self.canny_edge_threshold_2 = rospy.get_param("/pipe_detector/canny_edge_threshold_2")
            self.use_auto_canny_threshold = rospy.get_param("/pipe_detector/use_auto_canny_threshold")
            self.hough_transform_threshold = rospy.get_param("/pipe_detector/hough_transform_threshold")
            self.use_simple_hough = rospy.get_param("/pipe_detector/use_simple_hough")
            self.image_width = rospy.get_param("/pipe_detector/image_width")
            self.image_height = rospy.get_param("/pipe_detector/image_height")
            self.calibrate_homography = rospy.get_param('/pipe_detector/calibrate_homography')
            self.debug_mode = rospy.get_param('/pipe_detector/debug_mode')

        except rospy.ROSException:
            print("could not get param from server")


        #used for homography calibration
        
        self.refPt = [] #TODO set twice?
        self.calibrate_homography = False
        self.OUT_H_FACTOR = 2
        self.output_width = 0

        self.src_pt1 = 0
        self.src_pt2 = 0
        self.bottom_offset = 0
        self.hough_min_line = 50
        self.hough_max_gap = 10
        self.hough_line_width_resolution = 2
        self.hough_angle_resolution = 180
        self.create_trackbars = False
        self.linemodel = KMeans(n_clusters=2, n_init=1, tol=10, max_iter=1000)
        self.refPt = np.array([(148, 275), (441, 275), (self.image_width, self.image_height), (0, self.image_height)])
        self.image_resize_size = (600, 700)

        self.use_auto_canny_threshold = False

        self.PIPERAIL_WIDTH_MIN = 250
        self.PIPERAIL_WIDTH_MAX = 500

        self.use_simple_hough = True
        self.use_image_topic = False
        
        self.debug_mode = False
        if self.debug_mode:
            cv2.namedWindow("homography_params", cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow("detection_params", cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow("hsv_threshold", cv2.WINDOW_AUTOSIZE)

            cv2.createTrackbar("src_pt1", "homography_params" , 100, self.image_width, self.on_trackbar)
            cv2.createTrackbar("src_pt2", "homography_params" , 310, self.image_height, self.on_trackbar)
            # cv2.createTrackbar("bottom_offset", "homography_params" , 0, self.image_height, self.on_trackbar)
            cv2.createTrackbar("out_h_factor", "homography_params" , 1, 20, self.on_trackbar)
            if self.use_simple_hough:
                cv2.createTrackbar("hough_transform_threshold", "detection_params" , 220, 400, self.on_trackbar)
            else:
                cv2.createTrackbar("hough_transform_threshold", "detection_params" , 210, 300, self.on_trackbar)
            cv2.createTrackbar("blur_threshold", "detection_params" , 11, 20, self.on_trackbar)
            cv2.createTrackbar("morp_kernel_val", "detection_params" , 5, 20, self.on_trackbar)
            # cv2.createTrackbar("output_width", "homography_params" , 0, 640, self.on_trackbar)
            cv2.createTrackbar('hough_min_line_length','detection_params',100, 200, self.on_trackbar)
            cv2.createTrackbar('hough_max_gap','detection_params',5, 50, self.on_trackbar)
            cv2.createTrackbar('hough_line_width_resolution','detection_params',2, 20, self.on_trackbar)
            cv2.createTrackbar('hough_angle_resolution','detection_params',180, 360, self.on_trackbar)
            cv2.createTrackbar('canny_upper_threshold','detection_params',200, 255, self.on_trackbar)
            cv2.createTrackbar('canny_lower_threshold','detection_params',50, 255, self.on_trackbar)
            

            low_H_name = 'Low H'
            low_S_name = 'Low S'
            low_V_name = 'Low V'
            high_H_name = 'High H'
            high_S_name = 'High S'
            high_V_name = 'High V'

            cv2.createTrackbar('low_hue', 'hsv_threshold' , 0, 180, self.on_trackbar)
            cv2.createTrackbar('high_hue', 'hsv_threshold' , 180, 180, self.on_trackbar)
            cv2.createTrackbar('low_sat', 'hsv_threshold' , 60, 255, self.on_trackbar)
            cv2.createTrackbar('high_sat', 'hsv_threshold' , 120, 255, self.on_trackbar)
            cv2.createTrackbar('low_val', 'hsv_threshold' , 0, 255, self.on_trackbar)
            cv2.createTrackbar('high_val', 'hsv_threshold' , 255, 255, self.on_trackbar)
        
            self.on_trackbar(1)

        else:
            self.set_default_params()

        if self.use_image_topic:
            rospy.logerr("Subscribing to image topic: " + self.image_topic)
            rospy.Subscriber(self.image_topic, Image, self.image_callback)

        self.warp_publisher = rospy.Publisher("/pipe_detector/warped_image", Image, queue_size=10)
        self.points_publisher = rospy.Publisher("/pipe_detector/pts_src", Image, queue_size=10)
        self.unwarp_publisher = rospy.Publisher("/pipe_detector/piperail_detection_image", Image, queue_size=10)


    def set_default_params(self):

        self.refPt[0,0] = self.image_width/2 - 100
        self.refPt[0,1] = 310 #326
        self.refPt[1,0] = self.image_width/2 + 100
        self.refPt[1,1] = 310 #326

        self.OUT_H_FACTOR = 1
        self.output_width =  100

        self.blur_threshold = 13
        self.hough_transform_threshold = 170
        self.morp_kernel_val = 3

        self.hough_min_line = 350
        self.hough_max_gap = 10

        self.hough_line_width_resolution =  2
        self.hough_angle_resolution =  180

        self.low_hue =  0
        self.high_hue =  255

        self.low_sat =  60
        self.high_sat =  120

        self.low_val =  0
        self.high_val =  255

    def on_trackbar(self, val):
        self.refPt[0,0] = self.image_width/2 - int(cv2.getTrackbarPos('src_pt1','homography_params'))
        self.refPt[0,1] = int(cv2.getTrackbarPos('src_pt2','homography_params'))
        self.refPt[1,0] = self.image_width/2 + int(cv2.getTrackbarPos('src_pt1','homography_params'))
        # self.refPt[1,0] = int(cv2.getTrackbarPos('src_pt3','homography_params'))
        self.refPt[1,1] = int(cv2.getTrackbarPos('src_pt2','homography_params'))

        # self.bottom_offset = int(cv2.getTrackbarPos('bottom_offset','homography_params'))
        self.OUT_H_FACTOR = int(cv2.getTrackbarPos('out_h_factor','homography_params'))
        self.output_width =  int(cv2.getTrackbarPos('src_pt1','homography_params'))

        self.blur_threshold = int(cv2.getTrackbarPos('blur_threshold','detection_params'))

        if (self.blur_threshold % 2) == 0:
            self.blur_threshold += 1

        self.hough_transform_threshold = int(cv2.getTrackbarPos('hough_transform_threshold','detection_params'))
        self.morp_kernel_val = int(cv2.getTrackbarPos('morp_kernel_val','detection_params'))

        self.hough_min_line = int(cv2.getTrackbarPos('hough_min_line_length','detection_params'))
        self.hough_max_gap = int(cv2.getTrackbarPos('hough_max_gap','detection_params'))

        self.hough_line_width_resolution =  int(cv2.getTrackbarPos('hough_line_width_resolution','detection_params'))
        self.hough_angle_resolution =  int(cv2.getTrackbarPos('hough_angle_resolution','detection_params'))

        self.canny_edge_threshold_1 =  int(cv2.getTrackbarPos('canny_lower_threshold','detection_params'))
        self.canny_edge_threshold_2 =  int(cv2.getTrackbarPos('canny_upper_threshold','detection_params'))

        self.low_hue =  int(cv2.getTrackbarPos('low_hue','hsv_threshold'))
        self.high_hue =  int(cv2.getTrackbarPos('high_hue','hsv_threshold'))

        self.low_sat =  int(cv2.getTrackbarPos('low_sat','hsv_threshold'))
        self.high_sat =  int(cv2.getTrackbarPos('high_sat','hsv_threshold'))

        self.low_val =  int(cv2.getTrackbarPos('low_val','hsv_threshold'))
        self.high_val =  int(cv2.getTrackbarPos('high_val','hsv_threshold'))



    def resize_images(self, img):
        resized_image = cv2.resize(img, self.image_resize_size)
        return resized_image

    def detect_piperail_from_img_msg(self, msg):
        
        if self.debug_mode:
            cv2.waitKey(1) 

        # rospy.loginfo("Got Image msg")
        np_img = ros_numpy.numpify(msg)
        img = cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)

        if self.calibrate_homography:
            self.calibrate_homography_fxn(img)
            
        if(len(img)>0):
            birdseye_img, homography = self.warpImage(img, self.refPt)
            
            start = time.time()
            ret_img, target, fail_target = self.detect_pipe_rail(birdseye_img)
            end = time.time()
            # rospy.loginfo("total piperail detectiong working at %.3f Hz", 1/(end - start))

            self.warp_publisher.publish(ros_numpy.msgify(Image, ret_img, encoding='bgr8'))

            if self.debug_mode:
                pts = self.refPt.reshape((-1,1,2))
                cv2.polylines(img,[pts],True,(0,255,255))
                self.points_publisher.publish(ros_numpy.msgify(Image, img, encoding='bgr8'))


        return ret_img, target, fail_target
        

    def image_callback(self, msg):
        
        if self.debug_mode:
            cv2.waitKey(1) 


        self.detect_piperail_from_img_msg(msg)
        # rospy.loginfo("Got Image msg")
        # np_img = ros_numpy.numpify(msg)
        # img = cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)

        # if self.calibrate_homography:
        #     self.calibrate_homography_fxn(img)
            
        # if(len(img)>0):
        #     birdseye_img, homography = self.warpImage(img, self.refPt)

        #     start = time.time()
        #     ret_img, target = self.detect_pipe_rail(birdseye_img)
        #     end = time.time()
        #     rospy.loginfo("total piperail detectiong working at %.3f Hz", 1/(end - start))

        #     self.warp_publisher.publish(ros_numpy.msgify(Image, ret_img, encoding='bgr8'))
            
        #     # unwarped_img = self.unwarpImage(ret_img, homography)
        #     # self.unwarp_publisher.publish(ros_numpy.msgify(Image, unwarped_img, encoding='bgr8'))

        # if self.debug_mode:
        #     pts = self.refPt.reshape((-1,1,2))
        #     cv2.polylines(img,[pts],True,(0,255,255))
        #     self.points_publisher.publish(ros_numpy.msgify(Image, img, encoding='bgr8'))


    def warpImage(self, image, pts_src):

        width = image.shape[1]
        height = image.shape[0]
        
        pts_dst = np.array([[self.output_width, 0], [width - self.output_width,0], [width - self.output_width , height*self.OUT_H_FACTOR], [self.output_width, height*self.OUT_H_FACTOR]])

        #compute homography from image to birds eye view coordinates
        h, status = cv2.findHomography(pts_src, pts_dst)

        image = image[self.bottom_offset:(self.bottom_offset+height), 0:width]

        #using max width and height of world coordinates for final image size
        im_out = cv2.warpPerspective(image, h, (width, height))

        return im_out, h

    def unwarpImage(self, image, h):

        width = image.shape[1]
        height = image.shape[0]

        #using max width and height of world coordinates for final image size
        ret_img = cv2.warpPerspective(image, np.linalg.inv(h), (width, height))

        return ret_img

    def rotate_bound(self, image, angle):
        # grab the dimensions of the image and then determine the
        # center
        (h, w) = image.shape[:2]
        (cX, cY) = (w // 2, h // 2)
        # grab the rotation matrix (applying the negative of the
        # angle to rotate clockwise), then grab the sine and cosine
        # (i.e., the rotation components of the matrix)
        M = cv2.getRotationMatrix2D((cX, cY), -angle, 1.0)
        cos = np.abs(M[0, 0])
        sin = np.abs(M[0, 1])
        # compute the new bounding dimensions of the image
        nW = int((h * sin) + (w * cos))
        nH = int((h * cos) + (w * sin))
        # adjust the rotation matrix to take into account translation
        M[0, 2] += (nW / 2) - cX
        M[1, 2] += (nH / 2) - cY
        # perform the actual rotation and return the image
        return cv2.warpAffine(image, M, (nW, nH))


    def calibrate_homography_fxn(self, image):

        cv2.namedWindow("image",cv2.WINDOW_KEEPRATIO)

        # use this to find new points in image
        cv2.setMouseCallback("image", self.click_point)

        while len(self.refPt) < 4:
            pts_src = np.array(self.refPt)
            pts = pts_src.reshape((-1,1,2))
            cv2.polylines(image,[pts],True,(0,255,255))
            # cv2.rectangle(resized, self.refPt[0], self.refPt[1], (0, 255, 0), 2)
            rospy.loginfo(self.refPt)
            
            cv2.imshow("image",image)
            k = cv2.waitKey(1)
            rospy.loginfo("click on screen to calibrate homography then close the window")

        self.calibrate_homography = False
        rospy.loginfo(self.refPt)


    def click_point(self, event, x, y, flags, param):
        # grab references to the global variables
        # if the left mouse button was clicked, record the starting
        # (x, y) coordinates and indicate that cropping is being
        # performed
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.refPt) == 4:
            #clear points if clicked more than 4 times
                self.refPt = []
            
            self.refPt.append((x, y))
        # check to see if the left mouse button was released


    def image_preprocessing(self, input_frame):

        kernel_1 = np.ones((self.blur_threshold, self.blur_threshold), np.float32) / 25
        
        # load the image, convert it to grayscale, and blur it slightly
        gray = cv2.cvtColor(input_frame, cv2.COLOR_BGR2GRAY)
        blur_1 = cv2.GaussianBlur(gray, (self.blur_threshold, self.blur_threshold), 0)

        binary = cv2.adaptiveThreshold(blur_1, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

        if self.debug_mode:
            cv2.imshow("binary image", binary)
            cv2.waitKey(1)

        binary = cv2.bitwise_not(binary)

        morph_kernel = np.ones((self.morp_kernel_val, self.morp_kernel_val), np.uint8)
        morph = cv2.morphologyEx(binary, cv2.MORPH_OPEN, morph_kernel)
        morph = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, morph_kernel)

        image_to_return = morph

        return image_to_return

    def find_canny_edges(self, processed_image):
        # compute the median of the single channel pixel intensities
        if self.use_auto_canny_threshold:
            v = np.median(processed_image)
            # apply automatic Canny edge detection using the computed median
            sigma = 0.33
            lower = int(max(0, (1.0 - sigma) * v))
            upper = int(min(255, (1.0 + sigma) * v))
        else:
            lower = self.canny_edge_threshold_1
            upper = self.canny_edge_threshold_2
        edges = cv2.Canny(processed_image, lower, upper)

        return edges


    def find_hough_lines(self, edges, image_to_draw=[]):
        lines = []

        if self.use_simple_hough:
            lines = cv2.HoughLines(edges, self.hough_line_width_resolution, np.pi / self.hough_angle_resolution, self.hough_transform_threshold, None, 0, 0)
            if self.debug_mode:
                if lines is not None:
                    for l in lines:
                        rho = l[0][0]
                        theta = l[0][1]
                        a = math.cos(theta)
                        b = math.sin(theta)
                        x0 = a * rho
                        y0 = b * rho
                        pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                        pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                        cv2.line(image_to_draw, pt1, pt2, (0, 0, 255), 1, cv2.LINE_AA)

        else:
            linesP = cv2.HoughLinesP(edges, self.hough_line_width_resolution, np.pi / self.hough_angle_resolution, self.hough_transform_threshold, None, self.hough_min_line, self.hough_max_gap)
            if linesP is not None:
                for i in range(0, len(linesP)):
                    # for x1, y1, x2, y2 in linesP[i]:
                        # cv2.line(image_to_draw, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    l = linesP[i][0]
                    cv2.line(image_to_draw, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)
        
        return lines, image_to_draw

    def draw_lines(self, lines, colour, image_to_draw):

        if lines is not None:
            for l in lines:
                rho = l[0][0]
                theta = l[0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                cv2.line(image_to_draw, pt1, pt2, colour, 1, cv2.LINE_AA)

        
        return image_to_draw

    def result_to_display(self, original_img, canny_edges, morph_image):

        original_img_resized = self.resize_images(original_img)
        morph_image_resized = self.resize_images(morph_image)
        numpy_horizontal = np.hstack((original_img_resized, morph_image_resized))
        
        return numpy_horizontal

    def detect_pipe_rail(self, input_frame):

        #Pre process the image: grayscale, blur, threhsold, invert, morphology 
        hsv = cv2.cvtColor(input_frame, cv2.COLOR_BGR2HSV)
        blur_1 = cv2.GaussianBlur(hsv, (self.blur_threshold, self.blur_threshold), 0)
        img = cv2.inRange(blur_1, (self.low_hue, self.low_sat, self.low_val), (self.high_hue, self.high_sat, self.high_val))
        # cv2.imshow("hsv threshold", img)

        # cv2.waitKey(1)
        

        # start = time.time()
        # processed_image = self.image_preprocessing(input_frame)  

        # end = time.time()
        # rospy.loginfo("preprocessing working at %.3f Hz", 1/(end - start))

        # start = time.time()
        edges = self.find_canny_edges(img) #We are now using this
        edges_cv2 = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        # morph_cv2 = cv2.cvtColor(processed_image, cv2.COLOR_GRAY2BGR)
        # end = time.time()
        # rospy.loginfo("edge detection working at %.3f Hz", 1/(end - start))

        if self.debug_mode:
            # cv2.imshow("preprocessed", processed_image)
            # cv2.waitKey(1)

            cv2.imshow("edge", edges)
            cv2.waitKey(1)

        # start = time.time()
        lines, input_frame_with_lines = self.find_hough_lines(edges, input_frame)
        fail_target = None
        if lines is not None:
            if (len(lines) >=2):

                #Find a pipe rail target
                #image_with_lines, target = self.mid_vector_from_lines(lines)

                # numpy_horizontal = self.result_to_display(input_frame, edges_cv2, morph_cv2_out)
                # cv2.imshow('Thresholder_App', numpy_horizontal)

                #cluster lines using Kmeans clustering N=2
                temp = np.zeros(lines.shape)
                temp[:,0,0] = abs(lines[:,0,0])
                temp[:,0,1] = np.deg2rad(np.mod(np.rad2deg(lines[:,0,1])+90,180)-90) 

            
                y = self.linemodel.fit_predict(temp.reshape(-1,2))
                # rospy.loginfo("kmeans clustering working at %.3f Hz", 1/(end - start))
                first_pipe_set = temp[y==0]
                second_pipe_set = temp[y==1]

                first_pipe = np.average(first_pipe_set, axis=0)
                second_pipe = np.average(second_pipe_set, axis=0)

                #could start doing smart checks here
                dist_first_pipe = first_pipe[0,0]
                dist_second_pipe = second_pipe[0,0]

                middle_pipe =np.zeros(first_pipe.shape)
                middle_pipe[0,0] = (first_pipe[0,0] + second_pipe[0,0])/2
                middle_pipe[0,1] = (first_pipe[0,1] + second_pipe[0,1])/2

                camera_offset = 25
                target_pipe = np.zeros(first_pipe.shape)
                target_pipe[0,0] = self.image_width/2  - camera_offset
                target_pipe[0,1] = 0

                # Draw the lines
                image_with_lines = self.draw_lines([first_pipe], (0, 255, 0), input_frame_with_lines)
                image_with_lines = self.draw_lines([second_pipe], (0, 255, 0), image_with_lines)
                image_with_lines = self.draw_lines([middle_pipe], (255, 0, 0), image_with_lines)
                image_with_lines = self.draw_lines([target_pipe], (0, 0, 255), image_with_lines)

                #check for structured distance between pipes
                delta_dist = abs(dist_first_pipe - dist_second_pipe)
                rospy.logdebug("Distance between pipes: %.3f", delta_dist)
                if self.PIPERAIL_WIDTH_MIN < delta_dist < self.PIPERAIL_WIDTH_MAX:

                    #get target
                    target = middle_pipe
                else:
                    rospy.logerr("Bad pipe detection")
                    target = None
                    fail_target = middle_pipe
            else:
                image_with_lines = input_frame
                target = None
        else:
            image_with_lines = input_frame
            target = None

        return image_with_lines, target, fail_target



    def read_data(self):
        cv2.namedWindow('Thresholder_App')

        #If we are reading data pick video or image folder
        if self.use_video:
            if self.input_video_file is not None:
                # Check if video opened successfully
                video_file = cv2.VideoCapture(self.input_video_file)
                if not video_file.isOpened():
                    print("Error opening video stream or file")
                # Read until video is completed
                while video_file.isOpened():
                    # Capture frame-by-frame
                    ret, frame = video_file.read()
                    xmargin = 100
                    ymargin = 750
                    if ret == True:
                        frame_crop = frame[xmargin:-xmargin, ymargin:-ymargin].copy()
                        self.detect_pipe_rail(frame_crop)
                        k = cv2.waitKey(1) & 0xFF
                        # exit if q or esc are pressed
                        if (k == ord('q') or k == 27):
                            break
                    else:
                        print('no video')
                        video_file.set(cv2.CAP_PROP_POS_FRAMES, 0)
                # When everything done, release the video capture object
                video_file.release()
            else:
                raise FileExistsError
        else:
            if self.input_image_file is not None:
                xmargin = 100
                ymargin = 750
                for each_img in os.listdir(self.input_image_file):
                    img = cv2.imread(os.path.join(self.input_image_file, each_img))
                    self.detect_pipe_rail(img)
                    # k = cv2.waitKey(1) & 0xFF
                    k = cv2.waitKey(2000) & 0xFF

                    # exit if q or esc are pressed
                    if (k == ord('q') or k == 27):
                        break
            else:
                print("Using video as no image directory available")
        # Closes all the frames
        cv2.destroyAllWindows()




def main():
    rospy.init_node('canny_edge_detector')
    # Get parameters from config file
    
    canny_edge_detector = CannyEdgeDetector()

    while not rospy.is_shutdown():
        
        # if params.use_video:
        # canny_edge_detector.read_data()
        rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException as e:
        print("Exception caught: {}".format(e))
