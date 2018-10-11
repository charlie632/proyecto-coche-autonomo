#include "Follower.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>

static const std::string OPENCV_WINDOW = "Image window";

Follower::Follower() : imageTransport(nh) {
    imageSubscriber = imageTransport.subscribe("/camera/rgb/image_raw", 1,
            &Follower::imageCallback, this);
    cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

    // Create a display window
    cv::namedWindow(OPENCV_WINDOW);
}

Follower::~Follower() {
    // Close the display window
    cv::destroyWindow(OPENCV_WINDOW);
}

void Follower::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // convert the ROS image message to a CvImage
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Update the GUI window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
}
