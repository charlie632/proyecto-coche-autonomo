#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv){
    ros::init(argc, argv, "showimage");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);

    Rect ROI(0, 0, 640, 320.0); //(start from 50,50 and has size 100x100)
    Mat mask_ = Mat::zeros(90, 640, CV_8UC1);
    Rect ROI_MASK(0, 360, 640, 90);
    float IMAGE_CENTER = 330;
    float center_point_x;
    int number_of_lines = 0;
    Point rook_points[1][4];
    std::vector<cv::Vec4i> lines_;
    Mat thr_;
    Mat thresholdImage;
    Mat edge;
    Mat medianBlurImage;
    Mat gaussianBlurImage;
    float error_in_pixels;


    Mat image;
    image = imread(argv[1]); 
    if (!image.data) {
        cout << "Error al cargar imagen: " << argv[1] << endl;
        exit(1);
    }

    //A HSV
    Mat hsv;
    cvtColor(image, hsv, cv::COLOR_BGR2HSV);

 // imshow("Cropped_img", cropped_image);

    Mat cropped_image_;
    Mat cropped_image;
    Mat cropped_image_all_lines;
    cropped_image_ = hsv(ROI_MASK);
    cvtColor(cropped_image_, cropped_image, COLOR_HSV2BGR);
    cropped_image_all_lines = cropped_image.clone();

    GaussianBlur(cropped_image_, gaussianBlurImage, cv::Size(5, 5), 0, 0); //Convert to HSV;
    medianBlur(gaussianBlurImage, medianBlurImage, 5);
    cv::inRange(medianBlurImage, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 90), thresholdImage); //Mask

    
    Mat x;
    cvtColor(medianBlurImage, x, COLOR_HSV2BGR);

    cv::Mat img_roi;
    

    rook_points[0][0] = Point(0, 90);
    rook_points[0][1] = Point(90, 0);
    rook_points[0][2] = Point(550, 0);
    rook_points[0][3] = Point(640, 90);

    const Point *ppt[1] = {rook_points[0]};
    int npt[] = {4};

    fillPoly(mask_, ppt, npt, 1, Scalar(255, 255, 255), 8);

    bitwise_and(thresholdImage, mask_, img_roi);

    thr_ = thresholdImage.clone();


    Canny(thr_, edge, 50, 200, 3);
    HoughLinesP(edge, lines_, 1, CV_PI / 180, 10, 70, 40);

    float right_angles = 0;
    float left_angles = 0;
    float left_lines_count = 0;
    float right_lines_count = 0;
    float angle_diff = 0;

    float point_right_x_i = 0;
    float point_right_x_f = 0;
    float point_right_y_i = 0;
    float point_right_y_f = 0;

    float point_left_x_i = 0;
    float point_left_x_f = 0;
    float point_left_y_i = 0;
    float point_left_y_f = 0;

    int count_right = 0;
    int count_left = 0;

    for (int i = 0; i < lines_.size(); i++)
    {
      cv::Vec4i l = lines_[i];

      float x_ = l[2] - l[0];
      float y_ = l[3] - l[1];

      cv::line(cropped_image_all_lines, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 200, 0), 2, cv::LINE_AA);

      if (x_ == 0)
      {
        continue;
      }
      float degrees = atan(y_ / x_) * (180 / CV_PI);

      if (degrees > 5 && degrees < 80)
      {
        count_right++;
        point_right_x_i += l[0];
        point_right_y_i += l[1];
        point_right_x_f += l[2];
        point_right_y_f += l[3];
      }
      else if (degrees < -5 && degrees > -80)
      {
        count_left++;
        point_left_x_i += l[0];
        point_left_y_i += l[1];
        point_left_x_f += l[2];
        point_left_y_f += l[3];
      }
    }

    if (count_left > 0)
    {
      number_of_lines++;
      point_left_x_i = point_left_x_i / count_left;
      point_left_y_i = point_left_y_i / count_left;
      point_left_x_f = point_left_x_f / count_left;
      point_left_y_f = point_left_y_f / count_left;
      cv::line(cropped_image, cv::Point(point_left_x_i, point_left_y_i), cv::Point(point_left_x_f, point_left_y_f), cv::Scalar(0, 0, 200), 2, cv::LINE_AA);
    }

    if (count_right > 0)
    {

      number_of_lines++;
      point_right_x_i = point_right_x_i / count_right;
      point_right_y_i = point_right_y_i / count_right;
      point_right_x_f = point_right_x_f / count_right;
      point_right_y_f = point_right_y_f / count_right;

      cv::line(cropped_image, cv::Point(point_right_x_i, point_right_y_i), cv::Point(point_right_x_f, point_right_y_f), cv::Scalar(200, 0, 0), 2, cv::LINE_AA);
    }

   
    
    float center_point_y;

    if (count_right != 0 && count_left != 0)
    {
      center_point_x = (point_right_x_i + point_right_x_f + point_left_x_f + point_left_x_i) / 4;
      center_point_y = (point_right_y_i + point_right_y_f + point_left_y_f + point_left_y_i) / 4;
      float d_x = point_left_x_f - point_left_x_i;
      float d_y = point_left_y_f - point_left_y_i;
      float degrees_curve_L =  atan(d_y / d_x) * (180 / CV_PI);

      d_x = point_right_x_f - point_right_x_i;
      d_y = point_right_y_f - point_right_y_i;
      float degrees_curve_R =  atan(d_y / d_x) * (180 / CV_PI);
      

    }

    else if( count_right == 0 && count_left > 0){
      float d_x = point_left_x_f - point_left_x_i;
      float d_y = point_left_y_f - point_left_y_i;
      float degrees_curve =  atan(d_y / d_x) * (180 / CV_PI);
      float desplazamiento_l = (-53 - degrees_curve) * 10;

    //   center_point_x = (point_left_x_f+point_left_x_i)/2 - desplazamiento_l;
      center_point_x = IMAGE_CENTER - desplazamiento_l;
      ROS_INFO("L: %f, D: %f, C: %f", degrees_curve, desplazamiento_l, center_point_x);
      center_point_y = IMAGE_CENTER;
      // ROS_INFO("CI: %f", degrees_curve);
    }
    else if( count_right > 0 && count_left == 0 ) {
      float d_x = point_right_x_f - point_right_x_i;
      float d_y = point_right_y_f - point_right_y_i;
      float degrees_curve =  atan(d_y / d_x) * (180 / CV_PI);
      
      float desplazamiento_r = (57 - degrees_curve)*6;

      center_point_x = (point_right_x_f+point_right_x_i)/2 -desplazamiento_r;
      center_point_y = IMAGE_CENTER;
    }

    circle(cropped_image, Point(center_point_x, 60), 10, Scalar(0, 255, 0), 1, 8, 0);

    line(cropped_image, Point(IMAGE_CENTER, 0), Point(IMAGE_CENTER, 120), Scalar(244, 66, 200), 1, LINE_AA);

    error_in_pixels = IMAGE_CENTER - center_point_x;

    count_right = 0;
    count_left = 0;


    point_right_x_i = 0;
    point_right_x_f = 0;
    point_right_y_i = 0;
    point_right_y_f = 0;

    point_left_x_i = 0;
    point_left_x_f = 0;
    point_left_y_i = 0;
    point_left_y_f = 0;


    imshow("Original", image);
    imshow("edge", edge);
    imshow("all lines", cropped_image_all_lines);
    imshow("Right and left", cropped_image);

    waitKey(0);
    ros::spinOnce();
    return 0;
}
