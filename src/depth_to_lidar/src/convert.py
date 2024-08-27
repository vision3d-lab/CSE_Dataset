import rospy
import rosbag
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Header

def depth_image_to_point_cloud(depth_image, camera_intrinsics, depth_scale=0.001):

    fx = camera_intrinsics['fx']
    fy = camera_intrinsics['fy']
    cx = camera_intrinsics['cx']
    cy = camera_intrinsics['cy']

    point_cloud = []

    height, width = depth_image.shape
    for v in range(height):
        for u in range(width):
            z = depth_image[v, u] * depth_scale
            if z == 0:
                continue
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            point_cloud.append((x, z, -y))

    return np.array(point_cloud)

def assign_to_lidar_lines(point_cloud, num_lines=32, vertical_fov=(-60, 60)):
    lower, upper = vertical_fov
    total_fov = np.abs(upper - lower)
    delta_angle = total_fov / num_lines

    lidar_angles = np.linspace(lower, upper, num=num_lines, endpoint=False)

    lidar_lines = []

    for point in point_cloud:
        x, y, z = point
        angle = np.arctan2(z, np.sqrt(x**2 + y**2)) * 180 / np.pi

        line_index = np.searchsorted(lidar_angles, angle, side='right') - 1

        if 0 <= line_index < num_lines:
            angle_diff = np.abs(lidar_angles[line_index] - angle)
            if angle_diff <= (delta_angle / 55.0):
                if lidar_lines and len(lidar_lines) > line_index:
                    lidar_lines[line_index] = np.vstack((lidar_lines[line_index], np.array([point])))
                else:
                    while len(lidar_lines) <= line_index:
                        lidar_lines.append(np.empty((0, 3)))
                    lidar_lines[line_index] = np.array([point])

    return [line for line in lidar_lines if line.size > 0]


def create_pointcloud2_msg(point_cloud, frame_id="map"):

    header = Header(frame_id=frame_id)
    header.stamp = rospy.Time.now()
    fields = [point_cloud2.PointField(name=n, offset=i*4, datatype=point_cloud2.PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
    return point_cloud2.create_cloud(header, fields, point_cloud)


if __name__ == "__main__":
    
    input_bag = rospy.get_param('/lidar_data_saver/input_bag_filename')
    input_depth = rospy.get_param('/lidar_data_saver/input_depth_topic')
    output_bag = rospy.get_param('/lidar_data_saver/output_bag_filename')
    num_channel = rospy.get_param('/lidar_data_saver/number_of_lidar_channels')
    fov_low = rospy.get_param('/lidar_data_saver/vertical_fov_lowest')
    fov_high = rospy.get_param('/lidar_data_saver/vertical_fov_highest')

    rospy.init_node('depth_to_lidar_converter', anonymous=True)

    camera_intrinsics = {
        'fx': 610.832763671875,
        'fy': 611.7766723632812,
        'cx': 640.0,
        'cy': 360.0,
    }


    bridge = CvBridge()


    bag = rosbag.Bag(input_bag)
    output_bag = rosbag.Bag(output_bag, 'w')

    try:
        for topic, msg, t in bag.read_messages(topics=[input_depth]):
            if rospy.is_shutdown():
                break

            if topic == input_depth:
                # Depth img msg -> OpenCV formatpor
                cv_depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                # Depth Image -> Point Cloud
                point_cloud = depth_image_to_point_cloud(cv_depth_image, camera_intrinsics)

                print("point cloud shape " , point_cloud.shape)

                original_msg = create_pointcloud2_msg(point_cloud)
                
                # # Point Cloud -> 32-line LiDAR
                lidar_data = assign_to_lidar_lines(point_cloud, num_lines=num_channel, vertical_fov=(fov_low, fov_high))
                all_points = np.concatenate(lidar_data, axis=0)

                print("lidar data shape " , all_points.shape)

                cloud_msg = create_pointcloud2_msg(all_points)
                
                output_bag.write('/pseudo_lidar_pcd', cloud_msg, t)

                
                rospy.sleep(0.1)
            
    finally:
        output_bag.close()
        
    rospy.spin()

