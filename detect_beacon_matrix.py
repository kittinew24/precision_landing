#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import pickle
from imutils import contours
from skimage import measure
import imutils
import tf

class BeaconDetection(object):

    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("BeconDetection", anonymous=True)
        self.tranformation_brodcaster = tf.TransformBroadcaster()
        rospy.Subscriber("camera2/usb_cam2/image_raw", Image, self.update_frame_callback)
        self.gps_position_subscribe = rospy.Subscriber('/mavros/global_position/rel_alt',Float64,self.Get_Position)
        self.rangefinder_positiom_subscribe = rospy.Subscriber('/mavros/distance_sensor/rangefinder_pub',Range,self.range_altitude)
        self.heading = rospy.Subscriber('/mavros/global_position/compass_hdg',Float64,self.Get_heading)
        rospy.wait_for_message("/camera2/usb_cam2/image_raw", Image)
        rospy.wait_for_message('/mavros/distance_sensor/rangefinder_pub',Range)
        self.fx = 729.6825441284694
        self.fy = 730.0559741727269
        self.cX_img = 300.88215320824696
        self.cY_img = 211.83079401098433
        
    def update_frame_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8") 
    
    #define get_position function 
    def Get_Position(self,data):
        self.pose_z = data.data

    def Get_heading(self, data):
    	self.heading = data.data
    	
    #define range_altitude dunction
    def range_altitude(self,data):
        self.pose_z_range = data.range

    def position_calculate(self,beacon_x,beacon_y):
        x_m = self.pose_z_range*(beacon_x-self.cX_img)/self.fx
        y_m = self.pose_z_range*(beacon_y-self.cY_img)/self.fy
        return x_m, y_m 

    def main(self):
        #open matrix file of camera calibration
        #with open('CameraCalibration/cameraMatrix.pkl', 'rb') as f:
        #    cameraMatrix = pickle.load(f)
        #with open('CameraCalibration/dist.pkl', 'rb') as f:
        #    dist = pickle.load(f)
        #n=1
        while not rospy.is_shutdown():
            frame = self.image
            # h,  w = frame.shape[:2]
            # newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))
            # dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)

            # # crop the image
            # x, y, w, h = roi
            # dst = dst[y:y+h, x:x+w]

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (11, 11), 0)

            ### find edge and draw rectangle on max area contour ###
            box = []
            if self.pose_z_range > 2 :
                edged = cv2.Canny(blurred, 20, 50)
                edged  = cv2.dilate( edged, None, iterations=2)
                contour, hierarchy = cv2.findContours(edged, 
                    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                points = []    
                sizes = []

                for cnt in contour:
                    rect = cv2.boundingRect(cnt)
                    points.append(rect)
                    sizes.append([rect[2], rect[3]])

                area = [ i[1]*i[0] for i in sizes]
                if area != [] :
                    area_max = max(area)
                    # cv2.imshow('Canny Edges After Contouring', edged)

                    for cnt in contour:
                        rect = cv2.boundingRect(cnt)
                        if rect[2]*rect[3] == area_max:
                            cv2.rectangle(frame, rect, (0,255,0), 1)
                            box = rect
                            # print(f"box position: {box}")
                            #cv2.imshow('Contours',image)

            # threshold the image to reveal light regions in the
            # blurred image
            thresh = cv2.threshold(blurred, 235, 255, cv2.THRESH_BINARY)[1]
            # perform a series of erosions and dilations to remove
            # any small blobs of noise from the thresholded image
            thresh = cv2.erode(thresh, None, iterations=2)
            thresh = cv2.dilate(thresh, None, iterations=4)
            cv2.imshow('thresh', thresh)
            # perform a connected component analysis on the thresholded image
            labels = measure.label(thresh, background=0)
            mask = np.zeros(thresh.shape, dtype="uint8")
            # loop over the unique components
            sizes = []
            # loop over the unique components
            for label in np.unique(labels):
                # if this is the background label, ignore it
                if label == 0:
                    continue
                # otherwise, construct the label mask and count the
                # number of pixels 
                labelMask = np.zeros(thresh.shape, dtype="uint8")
                labelMask[labels == label] = 255
                numPixels = cv2.countNonZero(labelMask)
                sizes.append(numPixels)
                # if the number of pixels in the component is sufficiently
                # large, then add it to our mask of "large blobs"
                mask = cv2.add(mask, labelMask)
            # cv2.imshow("mask",mask)
            # find the contours in the mask, then sort them from left to
            # right
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            # print(len(cnts))
            if len(cnts) != 0:
                cnts = contours.sort_contours(cnts)[0]
                # loop over the contours
                for (i, c) in enumerate(cnts):
                    # draw the bright spot on the image
                    (x, y, w, h) = cv2.boundingRect(c)
                    ((cX, cY), radius) = cv2.minEnclosingCircle(c)
              
                    cv2.circle(frame, (int(cX), int(cY)), int(radius),
                            (0, 0, 255), 3)
                    x_m, y_m = self.position_calculate(beacon_x=cX, beacon_y=cY)
                        #cv2.putText(frame, "x: {:.2f} y{:.2f}".format(x_m, y_m) , (0, 15),
                                				#cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
                        #cv2.putText(frame, "heading: {:.2f}".format(self.heading) , (0, 30),
                                                         				##cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2) 
                    self.tranformation_brodcaster.sendTransform((x_m, y_m, self.pose_z_range), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(), "beacon","usb_cam1" )

                        #print(x_m, y_m)
            # show the output image
            cv2.circle(frame, (int(self.cX_img), int(self.cY_img)), 0,
                                        (0, 0, 255), 3)
            #cv2.imwrite("matrix_test/matrix"+str(n)+'.jpg',frame)
            #n+=1
            #cv2.imshow("Video", frame)
            self.position_subscribe = rospy.Subscriber('/mavros/distance_sensor/rangefinder_pub',Range,self.range_altitude)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
#main
if __name__ == '__main__':
    obj = BeaconDetection()
    obj.main()