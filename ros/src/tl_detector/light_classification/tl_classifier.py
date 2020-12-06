from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = np.array([170,150,220])
        upper = np.array([179,255,255])
        mask = cv2.inRange(image, lower, upper)
        pixels = cv2.countNonZero(mask)
        if pixels > 100 :
            return TrafficLight.RED
        
        return TrafficLight.UNKNOWN
