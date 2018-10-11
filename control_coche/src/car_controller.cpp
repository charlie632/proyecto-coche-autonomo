#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

using namespace cv;
using namespace std;


void imageCallback(const sensor_msgs::ImageConstPtr& msg){
   try{
      imshow("Camera", cv_bridge::toCvShare(msg, "bgr8")->image);
      waitKey(1);
   }
   catch (cv_bridge::Exception& e){
      ROS_ERROR("No pude convertir de '%s' a 'bgr8'.", msg->encoding.c_str());
   }
}


int main(int argc, char** argv){
   ros::init(argc, argv, "car_controller");
   ros::NodeHandle n;
   ros::Rate loop_rate(10);

   namedWindow("Camera");
   startWindowThread();
   image_transport::ImageTransport it(n);
   image_transport::Subscriber sub=it.subscribe("/ackermann_vehicle/camera1/image_raw",1,imageCallback);

   ros::Publisher cmd_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_vehicle/ackermann_cmd",1);
   ackermann_msgs::AckermannDriveStamped driveStamped;


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
      //cout << key << endl;
      if (key == 81){
         steering_angle = steering_angle+2.01*pi/180.0;
      }
      if (key == 82){
         speed = max(0.0,min(speed+0.1,1.0));
      }
      if (key == 83){
         steering_angle = steering_angle-1.99*pi/180.0;
      }
      if (key == 84){
        speed = max(0.0,min(speed-0.1,1.0));
      }
      driveStamped.drive.speed = speed;
      driveStamped.drive.steering_angle = steering_angle;
      cmd_pub.publish(driveStamped);
      cout<<"Steering_angle = "<<steering_angle<<", speed = "<<speed<<endl;


      if (key == 113){
        driveStamped.drive.speed = 0.0;
        driveStamped.drive.steering_angle = 0.0;
         cmd_pub.publish(driveStamped);
         cout << " Bye, bye." << endl;
         break;
      }

      ros::spinOnce();
      loop_rate.sleep();
   }

   destroyWindow("Camera");
}
