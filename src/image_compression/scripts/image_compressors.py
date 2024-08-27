#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridgeError, CvBridge
import numpy as np
import cv2

class ImageCompressor:
    def __init__(self, input_topic, output_topic, encoding):
        self.bridge = CvBridge()
        self.encoding = encoding
        self.pub = rospy.Publisher(output_topic, CompressedImage, queue_size=1)
        self.sub = rospy.Subscriber(input_topic, Image, self.callback)

    def callback(self, data):
        try:
            if self.encoding == 'bgr8': # RGB 
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # Convert the RGB image.

                # Compress the image using JPG format.
                jpg_img = cv2.imencode('.jpg', cv_image)[1]

                # Create a CompressedImage message.
                compressed_image = CompressedImage()
                compressed_image.header.stamp = data.header.stamp
                compressed_image.header.frame_id = data.header.frame_id
                compressed_image.format = "jpeg"
                compressed_image.data = np.array(jpg_img).tostring()
            else:
                pass
            
            # publish compressed topic 
            self.pub.publish(compressed_image)
            
        except CvBridgeError as e:
            rospy.logerr(e)


if __name__ == "__main__":
    rospy.init_node('image_compressor')
    topics_rgb = ['/carter1/rgb_left', '/carter1/rgb_right', '/carter2/rgb_left', '/carter2/rgb_right', '/carter3/rgb_left', '/carter3/rgb_right']
    compressors_rgb = [ImageCompressor(topic, topic + "/compressed", "bgr8") for topic in topics_rgb]
    rospy.spin()
