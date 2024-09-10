# Dynamic feature removal module for SWARM-SLAM-D & COVINS-D
We developed Swarm-SLAM-D by applying the “Dynamic feature removal module” to Swarm-SLAM algorithm to evaluate the influence of dynamic objects. 
Our “Dynamic feature removal module” is modified based on the “Moving consistency check module” of DS-SLAM(https://github.com/ivipsourcecode/DS-SLAM/blob/c7577c8300f73c2e15cb540a963492659d972c73/src/Frame.cc#L339). 
Unlike DS-SLAM, we utilize reverse optical flow to determine dynamic keypoints from the current keyframes. 
This approach reduces dependence on key points from previous frames and enables the classification of dynamic key points without additional processing. 
As a result, we could handle the occlusion of dynamic objects (\eg, objects coming in or out of the field of view). 
The overall prodecure is as follows:

i. The inputs to the algorithm are the current frame \( I_{\text{curr}} \), the set of keypoints \( P_{\text{curr}} \) extracted from \( I_{\text{curr}} \), and the previous frame \( I_{\text{prev}} \). Our dynamic feature removal algorithm assumes that the current frame has moved backward to align with the previous frame, and all subsequent steps are processed based on this assumption.

ii. Using the Lucas-Kanade Optical Flow algorithm~\cite{lkopt}, calculate the optical flow from \( I_{\text{curr}} \) to \( I_{\text{prev}} \). The predicted result is the corresponding point \( P^{\prime}_{\text{curr}} \) in \( I_{\text{prev}} \) for each point in \( P_{\text{curr}} \).

iii. Filter out inappropriate correspondences based on the image's intensity.

iv. Using the RANSAC method, estimate the Fundamental matrix and utilize it to classify keypoints as inliers (static) or outliers (dynamic).

v. Remove outliers (\ie, dynamic keypoints) from the original set \( P_{\text{curr}} \).

```c++

// Check for moving keypoints between frames
std::vector<cv::KeyPoint> RGBDHandler::check_moving(cv::Mat &pre_img, cv::Mat &current_img, std::vector<cv::KeyPoint> &pre_kpts, std::vector<cv::KeyPoint> &current_kpts)
{
    // Initialize output vector for inliers (static keypoints)
    std::vector<cv::KeyPoint> inliers;
    outliers.clear();  // Clear previous outliers
    std::vector<cv::Point2f> prepoint;  // Points in the previous image
    std::vector<cv::Point2f> nextpoint; // Points in the current image

    // Convert keypoints to Point2f format for optical flow calculation
    cv::KeyPoint::convert(pre_kpts, nextpoint); 
    cv::KeyPoint::convert(current_kpts, prepoint); 

    // Copy current and previous images to grayscale matrices
    cv::Mat imGrayPre = current_img;
    cv::Mat imgray = pre_img;

    std::vector<uchar> state; // Status vector to check if a point has a correspondence
    std::vector<float> err;   // Error vector for optical flow
    // Compute optical flow between previous and current images using the Lucas-Kanade method
    cv::calcOpticalFlowPyrLK(imGrayPre, imgray, prepoint, nextpoint, state, err, cv::Size(22, 22), 5, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.01));
    
    std::vector<cv::Point2f> F_prepoint, F_nextpoint;  // Points that will be used for fundamental matrix estimation

    // Loop through all the points to filter out outliers
    for (int i = 0; i < state.size(); i++)
    {
        if (state[i] != 0)  // Only consider points that have a valid correspondence
        {
            // Define a 3x3 grid around the keypoint for pixel comparison
            int dx[10] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
            int dy[10] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };
            int x1 = prepoint[i].x, y1 = prepoint[i].y;
            int x2 = nextpoint[i].x, y2 = nextpoint[i].y;
            // Check if the points are near the edges of the image
            if ((x1 < limit_edge_corner || x1 >= imgray.cols - limit_edge_corner || x2 < limit_edge_corner || x2 >= imgray.cols - limit_edge_corner
                || y1 < limit_edge_corner || y1 >= imgray.rows - limit_edge_corner || y2 < limit_edge_corner || y2 >= imgray.rows - limit_edge_corner))
            {
                state[i] = 0;  // Mark as invalid if near the edges
                continue;
            }
            double sum_check = 0;  // Initialize the sum for intensity difference check
            // Compare intensity values around the point to further filter outliers
            for (int j = 0; j < 9; j++)
                sum_check += abs(imGrayPre.at<uchar>(y1 + dy[j], x1 + dx[j]) - imgray.at<uchar>(y2 + dy[j], x2 + dx[j]));
            // If the intensity difference is too large, mark as invalid
            if (sum_check > limit_of_check) state[i] = 0;
            // If still valid, add to the list for fundamental matrix estimation
            if (state[i])
            {
                F_prepoint.push_back(prepoint[i]);
                F_nextpoint.push_back(nextpoint[i]);
            }
        }
    }
    
    // Estimate the Fundamental matrix using RANSAC to classify points as inliers or outliers
    cv::Mat mask = cv::Mat(cv::Size(1, 300), CV_8UC1);  // Mask to hold inliers from RANSAC
    cv::Mat F = cv::findFundamentalMat(F_prepoint, F_nextpoint, mask, cv::FM_RANSAC, 0.1, 0.99);
    double sum_outlier = 0.0;  // Sum of distances of outliers from epipolar lines
    double n_out = 0.0000000001;  // Initialize the count of outliers

    // Loop through all points to determine inliers and outliers
    for (int i = 0; i < prepoint.size(); i++)
    {
        if (state[i] != 0)  // Only consider points that passed the earlier checks
        {
            // Compute the epipolar distance using the Fundamental matrix
            double A = F.at<double>(0, 0)*prepoint[i].x + F.at<double>(0, 1)*prepoint[i].y + F.at<double>(0, 2);
            double B = F.at<double>(1, 0)*prepoint[i].x + F.at<double>(1, 1)*prepoint[i].y + F.at<double>(1, 2);
            double C = F.at<double>(2, 0)*prepoint[i].x + F.at<double>(2, 1)*prepoint[i].y + F.at<double>(2, 2);
            double dd = fabs(A*nextpoint[i].x + B*nextpoint[i].y + C) / sqrt(A*A + B*B);

            // Identify inliers (static points) based on the epipolar distance threshold
            if (dd <= limit_dis_epi) // Threshold should be defined based on your requirements
            {
                total_n_in++;  // Increment count of inliers
                inliers.push_back(cv::KeyPoint(prepoint[i], 1.f));  // Add to inliers
            }
            else
            {
                outliers.push_back(cv::KeyPoint(prepoint[i], 1.f));  // Add to outliers

                sum_outlier += dd;  // Update outlier distance sum
                n_out++;  // Increment count of outliers
                total_sum += dd;  // Update total distance sum
                total_n_out++;  // Increment total count of outliers
            }
        }
    }

    // Log information about the results
    RCLCPP_INFO(node_->get_logger(), "check_moving %lu", inliers.size());
    RCLCPP_INFO(node_->get_logger(), "AVG  %lf", sum_outlier / n_out);
    RCLCPP_INFO(node_->get_logger(), "Total AVG  %lf", total_sum / total_n_out);
    RCLCPP_INFO(node_->get_logger(), "Total Keypoint number  %lf", total_n_in);
    RCLCPP_INFO(node_->get_logger(), "Total dynamic kp number  %lf", total_n_out);
    RCLCPP_INFO(node_->get_logger(), "Total KF number  %lf", total_keyframe);

    return inliers;  // Return the inliers (static keypoints)
}


```