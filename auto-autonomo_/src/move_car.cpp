#include "move_car.h"

int main(int argc, char** argv)
{
  float speed = 6.0;  
  float st_a = .2;  

  if (argc < 3) {
    ROS_ERROR("Wrong number of arguments. Usage:\n rosrun ackermann_controller move_car_node speed steering_angle");
    return 1;
  } else {
    speed = atof(argv[1]);
    st_a = atof(argv[2]);
  } 

  ros::init(argc, argv, "move_car");
  AckermannMover am;

  ros::Rate rate(10);

  while(ros::ok()) {
    am.go(speed, st_a);
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}
