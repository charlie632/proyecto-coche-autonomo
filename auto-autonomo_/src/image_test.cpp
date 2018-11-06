#include <opencv2/opencv.hpp>
#include "ros/ros.h"

using namespace std;
using namespace cv;

const float ROI_Y = 320.0;
const float LINE_HEIGHT = 370.0;
const float PI = 3.14159;

const double halfC = M_PI / 180;

const float IMAGE_HEIGHT = 480;
const float IMAGE_WIDTH = 640;

Rect ROI(0, 0, IMAGE_WIDTH, ROI_Y); //(start from 50,50 and has size 100x100)

int main(int argc, char **argv){
    ros::init(argc, argv, "showimage");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);

    Mat image;
    image = imread(argv[1]); 
    if (!image.data) {
        cout << "Error al cargar imagen: " << argv[1] << endl;
        exit(1);
    }

    Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    // smooth the image in the "src" and save it to "dst"
    // blur(src, dst, Size(i,i));

    // Gaussian smoothing
    Mat dst;
    Mat thr;

    Mat mask(thr.size(), CV_8UC1, Scalar::all(255));
    mask(ROI).setTo(Scalar::all(0))

    cv::GaussianBlur(hsv, dst, cv::Size(5, 5), 0, 0);                     //Convert to HSV;
    cv::inRange(dst, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 66), thr); //Mask

    cv::imshow(OPENCV_WINDOW_THR, thr);
    cv::imshow("blur", dst);


    imshow("Original", image);

    waitKey(0);
    ros::spinOnce();
    return 0;
}
