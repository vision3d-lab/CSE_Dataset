#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import CompressedImage, Image

## Stereo RGB 
carter1_rgb_left = None
carter1_rgb_right = None
carter2_rgb_left = None
carter2_rgb_right = None
carter3_rgb_left = None
carter3_rgb_right = None
## Stereo Depth 
carter1_depth_left = None
carter1_depth_right = None
carter2_depth_left = None
carter2_depth_right = None
carter3_depth_left = None
carter3_depth_right = None


def callback(msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8, msg9, msg10, msg11, msg12):
    global carter1_rgb_left, carter1_rgb_right, carter2_rgb_left, carter2_rgb_right, carter3_rgb_left, carter3_rgb_right, \
            carter1_depth_left, carter1_depth_right, carter2_depth_left, carter2_depth_right, carter3_depth_left, carter3_depth_right
    # Save recent message 
    carter1_rgb_left = msg1
    carter2_rgb_left = msg2
    carter3_rgb_left = msg3
    carter1_rgb_right = msg4
    carter2_rgb_right = msg5
    carter3_rgb_right = msg6
    carter1_depth_left = msg7
    carter2_depth_left = msg8
    carter3_depth_left = msg9
    carter1_depth_right = msg10
    carter2_depth_right = msg11
    carter3_depth_right = msg12


def timer_callback(event):
    global carter1_rgb_left, carter1_rgb_right, carter2_rgb_left, carter2_rgb_right, carter3_rgb_left, carter3_rgb_right, \
            carter1_depth_left, carter1_depth_right, carter2_depth_left, carter2_depth_right, carter3_depth_left, carter3_depth_right
    
    if carter1_rgb_left and carter1_rgb_right and carter2_rgb_left and carter2_rgb_right and carter3_rgb_left and carter3_rgb_right and \
        carter1_depth_left and carter1_depth_right and carter2_depth_left and carter2_depth_right and carter3_depth_left and carter3_depth_right:
        
        carter1_rgb_left_pub.publish(carter1_rgb_left)
        carter2_rgb_left_pub.publish(carter2_rgb_left)
        carter3_rgb_left_pub.publish(carter3_rgb_left)
        carter1_rgb_right_pub.publish(carter1_rgb_right)
        carter2_rgb_right_pub.publish(carter2_rgb_right)
        carter3_rgb_right_pub.publish(carter3_rgb_right)

        carter1_depth_left_pub.publish(carter1_depth_left)
        carter2_depth_left_pub.publish(carter2_depth_left)
        carter3_depth_left_pub.publish(carter3_depth_left)
        carter1_depth_right_pub.publish(carter1_depth_right)
        carter2_depth_right_pub.publish(carter2_depth_right)
        carter3_depth_right_pub.publish(carter3_depth_right)
        

# Initialize ROS node
rospy.init_node('camera_sync')


## Stereo RGB 
carter1_rgb_left_pub = rospy.Publisher('synced/carter1/rgb_left/compressed', CompressedImage, queue_size=10)
carter2_rgb_left_pub = rospy.Publisher('synced/carter2/rgb_left/compressed', CompressedImage, queue_size=10)
carter3_rgb_left_pub = rospy.Publisher('synced/carter3/rgb_left/compressed', CompressedImage, queue_size=10)
carter1_rgb_right_pub = rospy.Publisher('synced/carter1/rgb_right/compressed', CompressedImage, queue_size=10)
carter2_rgb_right_pub = rospy.Publisher('synced/carter2/rgb_right/compressed', CompressedImage, queue_size=10)
carter3_rgb_right_pub = rospy.Publisher('synced/carter3/rgb_right/compressed', CompressedImage, queue_size=10)
# ## Stereo depth 
carter1_depth_left_pub = rospy.Publisher('synced/carter1/depth_left', Image, queue_size=10)
carter2_depth_left_pub = rospy.Publisher('synced/carter2/depth_left', Image, queue_size=10)
carter3_depth_left_pub = rospy.Publisher('synced/carter3/depth_left', Image, queue_size=10)
carter1_depth_right_pub = rospy.Publisher('synced/carter1/depth_right', Image, queue_size=10)
carter2_depth_right_pub = rospy.Publisher('synced/carter2/depth_right', Image, queue_size=10)
carter3_depth_right_pub = rospy.Publisher('synced/carter3/depth_right', Image, queue_size=10)


## Stereo RGB 
rgb_left_1 = message_filters.Subscriber('carter1/rgb_left/compressed', CompressedImage)
rgb_left_2 = message_filters.Subscriber('carter2/rgb_left/compressed', CompressedImage)
rgb_left_3 = message_filters.Subscriber('carter3/rgb_left/compressed', CompressedImage)
rgb_right_1 = message_filters.Subscriber('carter1/rgb_right/compressed', CompressedImage)
rgb_right_2 = message_filters.Subscriber('carter2/rgb_right/compressed', CompressedImage)
rgb_right_3 = message_filters.Subscriber('carter3/rgb_right/compressed', CompressedImage)
## Stereo depth 
depth_left_1 = message_filters.Subscriber('carter1/depth_left', Image)
depth_left_2 = message_filters.Subscriber('carter2/depth_left', Image)
depth_left_3 = message_filters.Subscriber('carter3/depth_left', Image)
depth_right_1 = message_filters.Subscriber('carter1/depth_right', Image)
depth_right_2 = message_filters.Subscriber('carter2/depth_right', Image)
depth_right_3 = message_filters.Subscriber('carter3/depth_right', Image)


# ApproximateTimeSynchronizer 
ts = message_filters.ApproximateTimeSynchronizer([rgb_left_1, rgb_left_2, rgb_left_3, rgb_right_1, rgb_right_2, rgb_right_3, \
                                                    depth_left_1, depth_left_2, depth_left_3, depth_right_1, depth_right_2, depth_right_3], 10, 0.1)
ts.registerCallback(callback)

# Set timer to 30hz 
rospy.Timer(rospy.Duration(1.0/30.0), timer_callback)

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass