#include "opencv2/opencv.hpp"
#include "ros/ros.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv){
    ros::init(argc, argv, "showcam");
    ros::NodeHandle n;
    ros::Rate loop_rate(25);

    VideoCapture cap(1); // Can also use CV_CAP_ANY instead of 0
    if(!cap.isOpened()){
        cout << "Error al abrir el dispositivo" << endl;
        return -1;
    }
    cout << "Listo" << endl;
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    cout << "Frame size : " << dWidth << " x " << dHeight << endl;

    Mat image, image_hsv, image_bw;
    int low_H = 16, low_S = 134, low_V = 89;
    int high_H = 26, high_S = 230, high_V = 187;

    while(ros::ok())
    {
        cap >> image; // You can also use:  cap.read(image);
        imshow("Source", image);

	cvtColor(image, image_hsv, COLOR_BGR2HSV);
	inRange(image_hsv, Scalar(low_H, low_S, low_V),
			   Scalar(high_H, high_S, high_V), image_bw);
	imshow("Final", image_bw);


//
//
// Aqui pegaremos más código después
//
//

        if(waitKey(1) == 27){
	    break;		 // Draw image and wait 1 milisecond
	}
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
