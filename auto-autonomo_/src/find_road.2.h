#ifndef FIND_ROAD_H
#define FIND_ROAD_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

using namespace cv;
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

Rect ROI(0, 0, IMAGE_WIDTH, ROI_Y); //(start from 50,50 and has size 100x100)
Mat mask_ = Mat::zeros(640, 480, CV_8UC3);
Point rook_points[1][20];

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
  float angle_average_total;
  cv::Mat thr, gray, hsv, edge;

  std::vector<cv::Vec4i> lines_; // To hold lines found by Hough transform
  std::vector<cv::Vec4i> lines_right;
	std::vector<cv::Vec4i> lines_left;

public:
  RoadFinder()
      : it_(nh_)
  {
    // Temporary values
    middle_of_road_.x = 400;
    // Subscribe to input video feed and publish output video feed
    // image_sub_ = it_.subscribe("/camera/image", 1,
    //                            &RoadFinder::imageCallback, this);
    // image_pub_ = it_.advertise("/road_finder/output_video", 1);

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
    // image_sub_ = it_.subscribe(topic_name, 1,
    //                            &RoadFinder::imageCallback, this);
    // image_pub_ = it_.advertise("/road_finder/output_video", 1);

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


 
  float getAngleAverage(){
    return angle_average_total;
  }
  void imageProcessing(const cv::Mat &img)
  {
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    // smooth the image in the "src" and save it to "dst"
    // blur(src, dst, Size(i,i));

    // Gaussian smoothing
    Mat dst;

    cv::GaussianBlur(hsv, dst, cv::Size(5, 5), 0, 0);                     //Convert to HSV;
    cv::inRange(dst, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 66), thr); //Mask

    // cv::imshow(OPENCV_WINDOW_THR, thr);
    // cv::imshow("blur", dst);

    Mat mask(thr.size(), CV_8UC1, Scalar::all(255));
    mask(ROI).setTo(Scalar::all(0));

    cv::Mat img_roi;
    bitwise_and(thr, mask, img_roi);

    rook_points[0][0] = Point(0, 480);
    rook_points[0][1] = Point(120, 320);
    rook_points[0][2] = Point(470, 320);
    rook_points[0][3] = Point(640, 480);

    const Point *ppt[1] = {rook_points[0]};
    int npt[] = {4};

    fillPoly(mask_, ppt, npt, 1, Scalar(255, 255, 255), 8);

    // bitwise_and(mask_, thr, img_roi);
    // imshow("Image", mask_);
    // imshow("roi", img_roi);

    cv::Mat thr_ = img_roi.clone();

    //Skelenotization

    // cv::Mat skel(thr.size(), CV_8UC1, cv::Scalar(0));
    // cv::Mat temp(thr.size(), CV_8UC1);

    // cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    // bool done;
    // do
    // {
    //   cv::morphologyEx(thr, temp, cv::MORPH_OPEN, element);
    //   cv::bitwise_not(temp, temp);
    //   cv::bitwise_and(thr, temp, temp);
    //   cv::bitwise_or(skel, temp, skel);
    //   cv::erode(thr, thr, element);

    //   double max;
    //   cv::minMaxLoc(thr, 0, &max);
    //   done = (max == 0);
    // } while (!done);

    //End Skelenotization

    cv::Canny(thr_, edge, 50, 200, 3);
    cv::HoughLinesP(edge, lines_, 1, CV_PI / 180, 10, 70, 40);
    cv::Mat img_copy = img.clone();

    float right_angles = 0;
    float left_angles = 0;
    float left_lines_count = 0;
    float right_lines_count = 0;
    float angle_diff = 0;

    //   p3 p4
    // p1    p2

    float p1_x = IMAGE_WIDTH;
    float p1_y = 0;
    float p2_x = 0;
    float p2_y = 0;

    float p3_x = IMAGE_WIDTH;
    float p3_y = IMAGE_HEIGHT;

    float p4_x = 0;
    float p4_y = IMAGE_HEIGHT;


    Point p1 = Point(0,0);
    Point p2 = Point(0,0);
    Point p3 = Point(0,0);
    Point p4 = Point(0,0);

    float point_right_x_i = 0;
    float point_right_x_f = 0;
    float point_right_y_i = 0;
    float point_right_y_f = 0;

    float point_left_x_i = 0;
    float point_left_x_f = 0;
    float point_left_y_i = 0;
    float point_left_y_f = 0;


    cv::Mat img_dummy = img.clone();
    // for (int i = 0; i < lines_.size(); i++)
    // {
    //   cv::Vec4i l = lines_[i];

    //   float x_ = l[2] - l[0];
    //   float y_ = l[3] - l[1];

    //   float degrees = atan(y_/x_) * (180 / PI);

    //   if(degrees > 10 && degrees < 60){
    //     lines_right.push_back(l);
    //     point_right_x_i += l[0];
    //     point_right_y_i += l[1];
    //     point_right_x_f += l[2];
    //     point_right_y_f += l[3];
    //   }
    //   else if( degrees > -10 && degrees < -60){
    //     lines_left.push_back(l);
    //     point_left_x_i += l[0];
    //     point_left_y_i += l[1];
    //     point_left_x_f += l[2];
    //     point_left_y_f += l[3];
    //   }
    // }

    // if(lines_left.size() > 0){
    //   int size = lines_left.size();
    //   point_left_x_i = point_left_x_i/size;
    //   point_left_y_i = point_left_y_i/size;
    //   point_left_x_f = point_left_x_f/size;
    //   point_left_y_f = point_left_y_f/size;
    //   cv::line( img_dummy, cv::Point(point_left_x_i,point_left_y_i), cv::Point(point_left_x_f,point_left_y_f),cv::Scalar(0,0,200), 2, cv::LINE_AA);
    // }

    // if(lines_right.size() > 0){
    //   int size = lines_right.size();
    //   point_right_x_i = point_right_x_i/size;
    //   point_right_y_i = point_right_y_i/size;
    //   point_right_x_f = point_right_x_f/size;
    //   point_right_y_f = point_right_y_f/size;

    //   cv::line( img_dummy, cv::Point(point_right_x_i,point_right_y_i), cv::Point(point_right_x_f,point_right_y_f),cv::Scalar(200,0,0), 2, cv::LINE_AA);
    // }


    imshow("kk", img);

  }

};

#endif
