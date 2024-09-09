import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import rosbag
from tools import add_gaussian_shifts, filterDisp

bridge = CvBridge()
dot_pattern_ = cv2.imread(cv2.imread("./cse_kinect_pattern.png", 0))

# CSE Dataset parameters
focal_length = 610.0  
baseline_m = 0.12  # baseline in meters 
invalid_disp_ = 99999999.9
scale_factor = 100  # converting depth from m to cm
max_depth = 5 # user define 

# List of topics to process
topics_to_process = [
    "/carter1/depth_left",
    "/carter1/depth_right",
    "/carter2/depth_left",
    "/carter2/depth_right",
    "/carter3/depth_left",
    "/carter3/depth_right"
]

# Topic map for processed topics
processed_topic_map = {
    "/carter1/depth_left": "/carter1/noised_depth_left",
    "/carter1/depth_right": "/carter1/noised_depth_right",
    "/carter2/depth_left": "/carter2/noised_depth_left",
    "/carter2/depth_right": "/carter2/noised_depth_right",
    "/carter3/depth_left": "/carter3/noised_depth_left",
    "/carter3/depth_right": "/carter3/noised_depth_right",
}

def process_depth_image(depth_image):
    try:
        depth = depth_image.astype('float32')

        # Add noise Gaussian Shift
        depth_interp = add_gaussian_shifts(depth)

        # Compute disparity
        disp_ = focal_length * baseline_m / (depth_interp + 1e-5)
        depth_f = np.round(disp_ * 8.0) / 8.0

        # Filtering disparity
        out_disp = filterDisp(depth_f, dot_pattern_, invalid_disp_)

        # Re-compute depth
        depth = focal_length * baseline_m / (out_disp + 1e-10)
        depth[out_disp == invalid_disp_] = 0  # Handle invalid values

        # Filtering > max_depth
        depth[depth > max_depth] = 0

        # Add depth noise 
        # h, w = depth.shape
        # noisy_depth = (35130 / np.round((35130 / np.round(depth * scale_factor)) + 
        #                                 np.random.normal(size=(h, w)) * (1.0 / 6.0) + 0.5)) / scale_factor

        return depth

    except Exception as e:
        rospy.logerr("Error processing depth image: %s", str(e))
        return None

def process_bag_file(input_bag_file, output_bag_file):
    try:
        # Open the input and output bags
        with rosbag.Bag(output_bag_file, 'w') as outbag:
            for topic, msg, t in rosbag.Bag(input_bag_file).read_messages():
                if topic in topics_to_process:
                    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

                    # Process the depth image
                    processed_depth = process_depth_image(depth_image)

                    if processed_depth is not None:
                        depth_msg = bridge.cv2_to_imgmsg(processed_depth, encoding="passthrough")
                        depth_msg.header = msg.header  # Keep the original header information
                        outbag.write(processed_topic_map[topic], depth_msg, t)
                else:
                    outbag.write(topic, msg, t)

        rospy.loginfo("Processing completed. Output saved to %s", output_bag_file)

    except Exception as e:
        rospy.logerr("Error processing bag file: %s", str(e))

if __name__ == "__main__":
    rospy.init_node('depth_image_processor', anonymous=True)

    input_bag = "/path/to/your/input.bag"
    output_bag = "/path/to/your/output.bag"

    process_bag_file(input_bag, output_bag)