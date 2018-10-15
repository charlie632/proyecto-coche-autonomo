#include "find_road.h"
#include "move_car.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>  

int main(int argc, char** argv)
{
  float speed = 1;  
  float Ka = 1.0;
  // float Ka = 1
  float max_speed = 1.2;
  float min_speed = 0.02;
  float Ks = 0.2;
  std::vector<cv::Vec4i> lines_right;
  std::vector<cv::Vec4i> lines_left;
  
  if (argc > 1) { speed = atof(argv[1]); }
  if (argc > 2) { Ka = atof(argv[2]); }
  if (argc > 3) { max_speed = atof(argv[3]); }
  if (argc > 4) { min_speed = atof(argv[4]); }


  ros::init(argc, argv, "find_road");
  RoadFinder rf;
  AckermannMover am;
  ros::Rate rate(20);

  image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  image_sub_ = it_.subscribe("/camera/image", 1, &Edge_Detector::imageCb, this);
	image_pub_ = it_.advertise("/auto-autonomo/raw_image", 1);
  
  while(ros::ok()) {

 
    ros::spinOnce();
    
    float angle_error = rf.getAngleError();

    float speed_command = speed - fabs(angle_error)*Ks;
    
    float steer = angle_error*Ka;
    if(steer > 20 ){
      steer = 20;
    }
    if(steer < -20){
      steer = -20;
    }
    
    float steering_angle = (steer*3.1416/180);

    ROS_INFO("RAD: %f | steer: %f", steering_angle, steer);


    // speed_command = std::max(min_speed, std::min(speed_command, max_speed));
    if(speed_command > max_speed){
      speed_command = max_speed;
    }

    if(speed_command < min_speed){
      speed_command = min_speed;
    }
  
    am.go(speed_command, steering_angle);
    // am.go(0, 0);
    rate.sleep();
  }
  return 0;
}
