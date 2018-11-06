#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float64.h>
#include <ackermann_msgs/AckermannDriveStamped.h>


using namespace cv;
using namespace std;

int main(int argc, char** argv){
   ros::init(argc, argv, "car_teleop");
   ros::NodeHandle n;
   ros::Rate loop_rate(10);

   ros::Publisher cmd_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_vehicle/ackermann_cmd",1);

   ros::Publisher velocity_pub = n.advertise<std_msgs::Float64>("sub_speed", 1);
   ros::Publisher steering_pub = n.advertise<std_msgs::Float64>("sub_steering_angle", 1);
   ackermann_msgs::AckermannDriveStamped driveStamped;
  

  std_msgs::Float64 velocity;
  std_msgs::Float64 radius;

   namedWindow("Car Teleop");
//   Mat image;
  // image = imread("~/catkin_ws/src/control_coche/src/control_keys.jpg");
//   imshow("Car Teleop",image);

   int key;
   float pi = 3.1416;
   float speed = 0.1;
   float steering_angle = 0.0;

   cout << " Este nodo controla el coche (tipo ackermann). Se publica un mensaje al tópico /ackermann_vehicle/ackermann_cmd" << endl;
   cout << " Los controles son: " << endl;
   cout << "                         arriba-abajo  = speed control" << endl;
   cout << "                         derecha-izq   = steering_angle" << endl;
   cout << " " << endl;
   cout << "                         Q para salir." << endl;



   while (n.ok()){
      key = waitKey(1);

      if (key == 81){
         steering_angle = max(-11.0,min(steering_angle + 1.0 , 11.0)); 
      }
      if (key == 82){
         speed = max(0.0,min(speed+0.05,1.5));
      }
      if (key == 83){
         steering_angle = max(-11.0,min(steering_angle - 1.0 , 11.0)); 
      }
      if (key == 84){
        speed = max(0.0,min(speed-0.1,1.5));
      }

      velocity.data = speed;
      radius.data = steering_angle;

      velocity_pub.publish(velocity);
      steering_pub.publish(radius);

      driveStamped.drive.speed = speed;
      driveStamped.drive.steering_angle = steering_angle*CV_PI/180;
		
      cmd_pub.publish(driveStamped);

      cout<<"Steering_angle = "<<steering_angle<<", speed = "<<speed<<endl;


      if (key == 113){
        velocity.data = 0.0;
        radius.data = 0.0;
        driveStamped.drive.speed = 0;
        driveStamped.drive.steering_angle = 0;
        cmd_pub.publish(driveStamped);

         cout << " Bye, bye." << endl;
         break;
      }

      ros::spinOnce();
      loop_rate.sleep();
   }

   destroyWindow("Car Teleop");
}