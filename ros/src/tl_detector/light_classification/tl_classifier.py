from collections import namedtuple
import cv2
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    # region of interest:
    ROI = namedtuple('ROI', ['top', 'left', 'bottom', 'right'], verbose=True)
    # image size:
    ImageSize = namedtuple('ImageSize', ['height', 'width'], verbose=True)

    def __init__(
        self,
        # ROI(top, left, bottom, right):
        ROI = (30, 0, 530, 800),
        downscale_ratio = 5.0
    ):
        # init ROI:
        (top, left, bottom, right) = ROI
        self.ROI = TLClassifier.ROI(
            top = top, left = left, 
            bottom = bottom, right = right
        )
        # init input image size:
        (height, width) = (self.ROI.bottom - self.ROI.top, self.ROI.right - self.ROI.left)
        self.input_size = TLClassifier.ImageSize(
            height = int(height / downscale_ratio),
            width = int(width / downscale_ratio)
        )
        # format for OpenCV:
        self.input_size_ = tuple(
            (self.input_size.width, self.input_size.height)
        )

    def preprocess(self, image):
        """ pre-process camera image for traffic light classification

        Args:

        Returns:

        """
        # step 1 -- crop to ROI:
        ROI = image[self.ROI.top:self.ROI.bottom, self.ROI.left:self.ROI.right, :]

        # step 2 -- resize:
        resized = cv2.resize(
            ROI,
            self.input_size_,
            interpolation = cv2.INTER_AREA
        )

        # step 3 -- histogram equalization:
        YUV = cv2.cvtColor(resized, cv2.COLOR_BGR2YUV)

        Y = cv2.split(YUV)[0]
        Y_equalized = cv2.equalizeHist(Y)

        YUV[:, :, 0] = Y_equalized

        return cv2.cvtColor(YUV, cv2.COLOR_YUV2BGR)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
