#ifndef FIND_OBSTACLES_H
#define FIND_OBSTACLES_H

#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>

using namespace std;

/**
   Class that subscribes to the laser scan data from the ackermann vehicle,
   and implements some basic algorithms to find targets/obstacles
   The class is header-file only. You only need to include the header file in your code.
 */
class ObstacleFinder
{
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher obstacle_pub_;
  ros::Publisher steering_angle_pub;

public:
  ObstacleFinder()
  {
    scan_sub_ = nh_.subscribe("/scan", 1,
                              &ObstacleFinder::laserCallback, this);
    obstacle_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/obstacle", 10);
    steering_angle_pub = nh_.advertise<std_msgs::Float32>("/follow_the_leader", 10);
    

  }
  ~ObstacleFinder() {}

  /**
     Callback function analyzing the incoming laser scans
  */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
  {

    float near_lim = 0.2;
    float far_lim = scan->range_max;

    // Find the minimim range reading that is > near_lim
    int range_index;
    float range = far_lim;
    int k = 0;
    for (std::vector<float>::const_iterator it = scan->ranges.begin();
         it != scan->ranges.end(); ++it)
    {
      if ((*it > near_lim) and (*it < far_lim))
      {
        if (*it < range)
        {
          range = *it;
          range_index = k;
          //ROS_INFO("Found range %f, at index %d", *it, k);
        }
      }
      k++;
    }

    // The angle corresponding to the closest
    float angle = range_index * scan->angle_increment;
    // This angle is CLOCKWISE from the direction of the first sample which is
    // FORWARD. This corresponds to the default of the rplidar,
    // which is supposed to be mounted with  cable towards the back.

    // Publish the position of the closest obstacle as a point in local tf with x pointing forward
    // and y to the left.
    geometry_msgs::PointStamped obstacle_msg;
    std_msgs::Float32 steering_msg;
    float x = range*cos(angle);
    float y = range*sin(angle); //Because Z is pointing towards the floor

    float real_angle = atan2(y, x) * 180 / 3.1416;
    if(real_angle < 90 && real_angle > -90){
      steering_msg.data = real_angle;
      steering_angle_pub.publish(steering_msg);


      cout << real_angle <<std::endl;
    }
    else{
       steering_msg.data = 1000;
       steering_angle_pub.publish(steering_msg);
    }
    obstacle_msg.point.x = range * cos(angle);
    obstacle_msg.point.y = -(-range * sin(angle));
    obstacle_msg.point.z = 0;
    obstacle_msg.header.frame_id = "laser";

    obstacle_pub_.publish(obstacle_msg);
    
  }
};

#endif
