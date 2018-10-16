#ifndef FIND_ROAD_H
#define FIND_ROAD_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW_THR = "thr";
static const std::string OPENCV_WINDOW_EDGES = "edges";
static const std::string OPENCV_WINDOW_SKE = "Skeleton";

const float ROI_Y = 320.0;
const float LINE_HEIGHT = 370.0;
const float PI = 3.14159;

const double halfC = M_PI / 180;

const float IMAGE_HEIGHT = 480;
const float IMAGE_WIDTH = 640;


/**
   Class that subscribes to the image stream from the camera on the ackermann vehicle,
   and implements some basic computer vision methods to find the road ahead. 
   The class is header-file only. You only need to include the header file in your code.
 */
class RoadFinder
{
  cv::Point2f middle_of_road_;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  float right_angle_average;
  float left_angle_average;
  float angle_error_degrees;
  cv::Mat thr, gray, hsv, edge;

  std::vector<cv::Vec4i> lines_; // To hold lines found by Hough transform

public:
  RoadFinder()
      : it_(nh_)
  {
    // Temporary values
    middle_of_road_.x = 400;
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image", 1,
                               &RoadFinder::imageCallback, this);
    image_pub_ = it_.advertise("/road_finder/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  float get_right_angle()
  {
    return right_angle_average;
  }

  float get_left_angle()
  {
    return left_angle_average;
  }

  RoadFinder(const char *topic_name)
      : it_(nh_)
  {
    // Temporary values
    middle_of_road_.x = 2;
    middle_of_road_.y = 2;

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe(topic_name, 1,
                               &RoadFinder::imageCallback, this);
    image_pub_ = it_.advertise("/road_finder/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~RoadFinder()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  int imageWidth()
  {
    if (thr.empty())
    {
      return 800;
    } // Temporary value
    else
    {
      return thr.cols;
    }
  }

  int imageHeight()
  {
    if (thr.empty())
    {
      return 600;
    } // Temporary value
    else
    {
      return thr.rows;
    }
  }

  float getAngleError()
  {
    return angle_error_degrees;
  }

  cv::Point2f &getMidpoint()
  {
    return middle_of_road_;
  }

  /**
     Returns the number of continuous white pixels along the middle column of the thresholded image
     starting from the bottom of the image. 
  */
  int freeRoadAhead()
  {
    if (thr.empty())
    {
      return 0;
    }
    else
    {
      // Get mid column of thresholded image
      int horizon = imageHeight() / 2;
      int midcol = imageWidth() / 2;
      cv::Mat mid_line = thr.col(midcol);

      int k = imageHeight() - 1;

      while (mid_line.at<uchar>(k) == 255 and k > horizon)
      {
        k--;
      }
      return imageHeight() - k;
    }
  }

  /**
     Returns the lines found in the image 
   */
  std::vector<cv::Vec4i> &getLines()
  {
    return lines_;
  }

  /**
     Callback function analyzing the incoming images
  */
  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Find white part of image. Determine centroid and draw circle at centroid
    // Make sure to ignore the marks in the middle of the road. Since the white contains also
    // the orange of the mid stripes, convert to color space
    // convert image to grayscale
    cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 66), thr);

    // find moments of the image
    cv::Moments m = cv::moments(thr, true);
    middle_of_road_.x = m.m10 / m.m00;
    middle_of_road_.y = m.m01 / m.m00;

    cv::Canny(thr, edge, 50, 200, 3);                          // detect edges
    cv::HoughLinesP(edge, lines_, 1, CV_PI / 180, 20, 50, 10); // detect lines

    cv::imshow(OPENCV_WINDOW_THR, thr);
    cv::imshow(OPENCV_WINDOW_EDGES, edge);

    cv::Mat skel(thr.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp(thr.size(), CV_8UC1);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool done;
    do
    {
      cv::morphologyEx(thr, temp, cv::MORPH_OPEN, element);
      cv::bitwise_not(temp, temp);
      cv::bitwise_and(thr, temp, temp);
      cv::bitwise_or(skel, temp, skel);
      cv::erode(thr, thr, element);

      double max;
      cv::minMaxLoc(thr, 0, &max);
      done = (max == 0);
    } while (!done);

    cv::imshow(OPENCV_WINDOW_SKE, skel);

     cv::Canny(skel, edge, 50, 200, 3);                          // detect edges
    cv::HoughLinesP(edge, lines_, 1, CV_PI / 180, 10, 50, 10); // detect lines

    cv::imshow(OPENCV_WINDOW_THR, thr);
    cv::imshow(OPENCV_WINDOW_EDGES, edge);

    // cv::imshow('edges', edge);

    // Find free road ahead
    int free_road = freeRoadAhead();

    // show the image with a point mark at the centroid, and detected lines

    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, middle_of_road_, 10, CV_RGB(255, 0, 0));

    // Draw the lines
    int midcol = imageWidth() / 2;
    cv::line(cv_ptr->image, cv::Point(midcol, imageHeight()),
             cv::Point(midcol, imageHeight() - free_road),
             cv::Scalar(0, 200, 0), 2, cv::LINE_AA);

    float right_angles = 0;
    float left_angles = 0;
    float left_lines_count = 0;
    float right_lines_count = 0;
    float angle_diff = 0;

    for (int i = 0; i < lines_.size(); i++)
    {
      cv::Vec4i l = lines_[i];

      float x_ = l[2] - l[0];
      float y_ = l[3] - l[1];
      float m = 0;

      if (x_ != 0)
      {
        m = -y_ / x_;
      }

      float b = l[3] - (m * l[2]);

      if (l[3] < ROI_Y || l[1] < ROI_Y)
      {
        continue;
      }

      if (m > 0)
      {
        // ROS_INFO("B_IZQ: %f", b);
      }
      else
      {
        // ROS_INFO("B_DER: %f", b);
      }

      if (x_ == 0)
      {
        continue;
      }

      float degrees = atan2(y_, x_) * (180 / PI);


      if (fabs(degrees) < 10 || fabs(degrees) > 90)
      {
        continue;
      }

      if (degrees < 0)
      {
        //Lineas izq
        degrees = degrees*-1;
        degrees = -degrees + 180;
        left_angles = left_angles + degrees;
        left_lines_count++;
        cv::line(cv_ptr->image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
               cv::Scalar(120, 0, 200), 2, cv::LINE_AA);
      }
      else if (degrees > 0)
      {
        //lineas derechas
        right_angles = right_angles + degrees;
        right_lines_count++;
        cv::line(cv_ptr->image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
               cv::Scalar(0, 0, 200), 2, cv::LINE_AA);
      }

      // ROS_INFO("Angulo der: %f Ángulo izq: %f", right_angle_average, left_angle_average);

      
    }

    if (right_lines_count != 0)
    {
      right_angle_average = right_angles / right_lines_count;
    }
    else
    {
      right_angle_average = 0;
    }

    if (left_lines_count != 0)
    {
      left_angle_average = left_angles / left_lines_count;
    }
    else
    {
      left_angle_average = 0;
    }

    angle_diff = right_angle_average - (left_angle_average * (-1));
     ROS_INFO("left: %f, right: %f", left_angle_average, right_angle_average);
    float angle_average = (right_angle_average + left_angle_average)/2;

    // ROS_INFO("AVE %f", angle_average);


    cv::Point first_point = cv::Point(IMAGE_WIDTH/2, IMAGE_HEIGHT);
    float last_x = IMAGE_WIDTH/2 - (IMAGE_HEIGHT/tanf(angle_average*halfC));
    cv::Point last_point = cv::Point(last_x, 0);

    cv::line(cv_ptr->image, first_point, last_point,
               cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

    // ROS_INFO("ANGLE_DIFF: %f", angle_diff);linear extrapolation
    angle_error_degrees = angle_diff;
    // float angle_diff_rad = (angle_diff * PI/180);

    // ROS_INFO("IZQ: %f, DER: %f", left_angle_average, right_angle_average);

    // float y_final = IMAGE_HEIGHT - LINE_HEIGHT*sin(angle_diff_rad);
    // float x_final = IMAGE_WIDTH - LINE_HEIGHT*cos(angle_diff_rad);
    // cv::line( cv_ptr->image, cv::Point(IMAGE_WIDTH/2, IMAGE_HEIGHT), cv::Point(x_final, y_final),
    // cv::Scalar(0,255,0), 2, cv::LINE_AA);

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());
  }
};

#endif
