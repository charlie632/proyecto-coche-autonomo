#include <opencv2/opencv.hpp>
#include "ros/ros.h"

using namespace std;
using namespace cv;

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
    imshow("Original", image);

    waitKey(0);
    ros::spinOnce();
    return 0;
}
