#include "find_road.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_road");
  RoadFinder rf;
  ros::spin();
  return 0;
}
