#include "find_road.h"
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <cstdlib>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>

int main(int argc, char **argv)
{

  float speed = 1;
  float Ka = 1.0;
  // float Ka = 1
  float max_speed = 1.2;
  float min_speed = 0.02;
  float Ks = 0.2;
  std::vector<cv::Vec4i> lines_right;
  std::vector<cv::Vec4i> lines_left;

  ros::init(argc, argv, "auto_autonomo");
  ros::NodeHandle n;

  std_msgs::Float64 velocity;
  std_msgs::Float64 radius;
  geometry_msgs::Pose2D msg_movement;

  float gain = 1;
  float gain2 = 1;
  int erosion_size = 10;

  //Define el publicador
  ros::Publisher movement_pub = n.advertise<geometry_msgs::Pose2D>("sub_movement", 1);

  int videoSource = 0;
  //índice de la cámara
  if (argc > 1)
  {
    videoSource = atoi(argv[1]);
  }
  //Ganancia 1
  if (argc > 2)
  {
    gain = atof(argv[2]);
  }
  //Ganancia 2
  if (argc > 3)
  {
    gain2 = atof(argv[3]);
  }
  if ( argc > 4)
  {
    erosion_size = atoi(argv[4]);
  }

  //Capture image
  cv::VideoCapture cap(videoSource);
  if (!cap.isOpened())
    return 1;
  cv::Mat frame;

  ros::init(argc, argv, "find_road");
  RoadFinder rf;
  //Definición del ros rate
  ros::Rate rate(20);

  while (ros::ok())
  {

    cap >> frame;

    if (!frame.empty())
    {
      //Llama la función si hay imagen
      driveStruct x = rf.imageProcessing(frame, gain, gain2, erosion_size);
      msg_movement.x = x.speed;
      msg_movement.y = x.steering_angle;

      // cout << x.speed << " " << x.steering_angle << endl;
    }
    // //Llama la función para obtener el error en pizeles y el número de imágenes
    // float error_in_pixels = 0;
    // float number_of_lines = 0;
    // float pixel_to_angle = error_in_pixels * 0.1; //Primera atenuaciíon
    // float steering_angle = 0;

    // //Si solo hay una línea, le aplica una ganancias
    // if (number_of_lines == 1)
    // {
    //   steering_angle = 0 + pixel_to_angle * gain2;
    // }
    // //Si hay dos líneas, se le aplica otra.
    // else if (number_of_lines == 2)
    // {
    //   steering_angle = 0 + pixel_to_angle * gain;
    // }
    // else
    // {
    //   steering_angle = 0 + pixel_to_angle * gain;
    // }
    // // ROS_INFO("E: %f", error_in_pixels);

    // msg_movement.x = 0; //Este valor se maneja desde el Arduino
    // msg_movement.y = steering_angle;

    //Publica el mensaje
    movement_pub.publish(msg_movement);

    ros::spinOnce();

    rate.sleep();
  }
  return 0;
}
