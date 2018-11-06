#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <stdio.h>

using namespace cv;
using namespace std;

int width = 800, heigth = 600;
Mat image_bgr = Mat(heigth, width, CV_8UC3, Scalar(0,0,0));
int cut = 370;

Mat image_crop = Mat(heigth - cut, width, CV_8UC3, Scalar(0,0,0));
Mat image_bw   = Mat(heigth - cut, width, CV_8UC1, Scalar(0));

Mat element, labels, stats, centroids;
int nLabels, the_max, index_max;
int midWidth = width / 2.0;
Mat mask_leftL  = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
Mat mask_rightL = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
Mat mask_leftG  = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
Mat mask_rightG = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
double steering_angle=0.0, speed=0.0;
bool tracking_left = false, tracking_right = false;
float PuntoI, PuntoD, AnguloI, AnguloD;

ros::Publisher cmd_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){

      image_bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
      // line(image_bgr, Point(0,cut), Point(width, cut), Scalar(255,0,0), 3, LINE_AA);

      //imshow("Camera", image_bgr);
      //waitKey(1);



      // Basic image pre-traitment
      Rect myROI(0, cut, width, heigth - cut); // myROI(x,y,w,h)
      image_crop = image_bgr(myROI);
      //cvtColor(image_crop, image_gray, COLOR_BGR2GRAY);
      //blur(image_crop, image_crop, Size(3,3));

      //imshow("Camera", image_crop);
      //waitKey(1);

      

      inRange(image_crop, Scalar(0, 0, 0), Scalar(60, 60, 60), image_bw);
      
      // imshow("Camera", image_bw);
      // waitKey(1);




      int erosion_type = MORPH_ELLIPSE; //  Other options: MORPH_RECT; MORPH_CROSS;
      int erosion_size = 10;
      element = getStructuringElement( erosion_type,
                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       Point( erosion_size, erosion_size ) );


      Mat mask_leftL(image_crop.rows, image_crop.cols, CV_8UC1, Scalar(0));
      Mat mask_rightL(image_crop.rows, image_crop.cols, CV_8UC1, Scalar(0));
      if (tracking_left == false){
         mask_leftL(Range(image_crop.rows / 2, image_crop.rows), Range(0, 190)) = 255;
         mask_leftG = mask_leftL.clone();
         //cout << "Inicializo MASCARA izq" << endl;
      }
      if (tracking_right == false){
         mask_rightL(Range(image_crop.rows / 2, image_crop.rows), Range(image_crop.cols - 190, image_crop.cols)) = 255;
         mask_rightG = mask_rightL.clone();
         //cout << "Inicializo MASCARA der" << endl;
      }

      Mat image_lineLeft, image_lineRight;
      bitwise_and(image_bw, mask_leftG , image_lineLeft );
      bitwise_and(image_bw, mask_rightG, image_lineRight);

      // imshow("Camera", image_lineRight);
      // imshow("Camera2", mask_rightG);
      // waitKey(1);



      // Busquemos el carril IZQUIERDO
      nLabels = connectedComponentsWithStats(image_lineLeft, labels, stats, centroids, 8, CV_32S);
      //cout << endl << "Number of connected components = " << nLabels << endl << "All stats = " << stats << endl;
      // Find the biggest area
      the_max=0, index_max=-1;
      if (nLabels > 1){
         for (int i=1; i<nLabels; i++){
            if (stats.at<int>(i,CC_STAT_AREA) > the_max){
               the_max = stats.at<int>(i,CC_STAT_AREA);
               index_max = i;
            }
         }
         //cout << " Bigest (area, index) = (" << the_max << ", " << index_max << ")" << endl;

         Mat image_label;
         compare(labels, index_max, image_label, CMP_EQ);
         // Better way to get the skeleton of an binary image
         Mat skel(image_crop.rows, image_crop.cols, CV_8UC1, Scalar(0));
         int inicio, num_pixels = 0;
         for (int row = 0; row < skel.rows; row++){
            bool flag = false;
            for (int col = 0; col < skel.cols; col++){
               if (flag == false && image_label.at<unsigned char>(row,col,0) > 0){
                  inicio = col;
                  flag = true;
               }
               if (flag == true && image_label.at<unsigned char>(row,col,0) == 0){
                  skel.at<unsigned char>(row, (inicio + col) / 2 ) = 255;
                  flag = false;
                  num_pixels++;
               }
            }
         }
      //    imshow("Camera", skel);
      //       waitKey(1);
         //cout << num_pixels << endl;
         if (num_pixels > 5){
            Mat FI(num_pixels, 2, CV_32F, Scalar(0));
            Mat  X(num_pixels, 1, CV_32F, Scalar(0));
            int index = 0;
            for(float y = 0; y < skel.rows; y++){
               for(float x = 0; x < skel.cols; x++){
                  if (skel.at<unsigned char>(y,x) == 255){
                     FI.at<float>(index,0) = y;
                     FI.at<float>(index,1) = 1.0;
                      X.at<float>(index,0) = x;
                     index++;               
                  }
               }
            }
            Mat FIT = FI.t();
            Mat Par = (FIT * FI).inv() * FIT *X;
            //cout << " Param = " << Par.at<float>(0,1) << endl;
            vector<Point2f> curvePoints;
            float x;
            Point2f new_point;
            for (float y = 0; y < heigth; y++){
               x = Par.at<float>(0,0) * y + 
                   Par.at<float>(0,1);
               new_point = Point2f(max(0.0, min(double(x), 800.0)), y);
               curvePoints.push_back(new_point); //add point to vector or list
            }
            PuntoI = Par.at<float>(0,0) * image_crop.rows + Par.at<float>(0,1);
            //cout << "PuntoI =" << PuntoI << endl;
            AnguloI = atan(-1.0/Par.at<float>(0,0))*180.0/CV_PI;
            //cout << "AnguloI = " << AnguloI << endl;
            for (int i = 0; i < curvePoints.size() - 1; i++){
               line(image_crop, curvePoints[i], curvePoints[i + 1], Scalar(0,255,0), 2, CV_AA);
            }
            //cout << "ENCONTRE linea IZQ" << endl;
            tracking_left = true;
            dilate(image_lineLeft, mask_leftG, element);
         }
         else{
            //cout << "Perdí la linea IZQ" << endl;
            tracking_left = false;
         }
      }
      else{
         //cout << "Perdí la linea IZQ" << endl;
         tracking_left = false;
      }

      // imshow("Camerssa", image_lineLeft);
      //       waitKey(1);


      // imshow("Cdijeidee", image_crop);
      // waitKey(1);

      
      



      // Busquemos el carril DERECHO
      nLabels = connectedComponentsWithStats(image_lineRight, labels, stats, centroids, 8, CV_32S);
      //cout << endl << "Number of connected components = " << nLabels << endl << "All stats = " << stats << endl;
      // Find the biggest area
      the_max=0, index_max=-1;
      if (nLabels > 1){
         for (int i=1; i<nLabels; i++){
            if (stats.at<int>(i,CC_STAT_AREA) > the_max){
               the_max = stats.at<int>(i,CC_STAT_AREA);
               index_max = i;
            }
         }
         //cout << " Bigest (area, index) = (" << the_max << ", " << index_max << ")" << endl;

         Mat image_label;
         compare(labels, index_max, image_label, CMP_EQ);
         // Better way to get the skeleton of an binary image
         Mat skel(image_crop.rows, image_crop.cols, CV_8UC1, Scalar(0));
         int inicio, num_pixels = 0;
         for (int row = 0; row < skel.rows; row++){
            bool flag = false;
            for (int col = 0; col < skel.cols; col++){
               if (flag == false && image_label.at<unsigned char>(row,col,0) > 0){
                  inicio = col;
                  flag = true;
               }
               if (flag == true && image_label.at<unsigned char>(row,col,0) == 0){
                  skel.at<unsigned char>(row, (inicio + col) / 2 ) = 255;
                  flag = false;
                  num_pixels++;
               }
            }
         }
         //cout << num_pixels << endl;
         if (num_pixels > 5){
            Mat FI(num_pixels, 2, CV_32F, Scalar(0));
            Mat  X(num_pixels, 1, CV_32F, Scalar(0));
            int index = 0;
            for(float y = 0; y < skel.rows; y++){
               for(float x = 0; x < skel.cols; x++){
                  if (skel.at<unsigned char>(y,x) == 255){
                     FI.at<float>(index,0) = y;
                     FI.at<float>(index,1) = 1.0;
                      X.at<float>(index,0) = x;
                     index++;               
                  }
               }
            }
            Mat FIT = FI.t();
            Mat Par = (FIT * FI).inv() * FIT *X;
            //cout << " Param = " << Par.at<float>(0,1) << endl;
            vector<Point2f> curvePoints;
            float x;
            Point2f new_point;
            for (float y = 0; y < heigth; y++){
               x = Par.at<float>(0,0) * y + 
                   Par.at<float>(0,1);
               new_point = Point2f(max(0.0, min(double(x), 800.0)), y);
               curvePoints.push_back(new_point); //add point to vector or list
            }
            PuntoD = Par.at<float>(0,0) * image_crop.rows + Par.at<float>(0,1);
            //cout << "PuntoD =" << PuntoD << endl;
            AnguloD = atan(-1.0/Par.at<float>(0,0))*180.0/CV_PI;
            //cout << "AnguloD = " << AnguloD << endl;
            for (int i = 0; i < curvePoints.size() - 1; i++){
               line(image_crop, curvePoints[i], curvePoints[i + 1], Scalar(255,0,255), 2, CV_AA);
            }
            //cout << "ENCONTRE linea DER" << endl;
            tracking_right = true;
            dilate(image_lineRight, mask_rightG, element);
         }
         else{
            //cout << "Perdí la linea DER" << endl;
            tracking_right = false;
         }
      }
      else{
         //cout << "Perdí la linea DER" << endl;
         tracking_right = false;
      }

      // imshow("ambos carriles", image_crop);
      // waitKey(1);


      if (tracking_left && tracking_right){
         cout << "GoAhead" << endl;
         steering_angle = -((PuntoD+PuntoI)/2.0 - midWidth)/10.0;
         //steering_angle = -(AnguloI+AnguloD)/2.0;
         steering_angle = max(-35.0, min(steering_angle, 35.0));
         speed = min(speed + 0.01, 1.6);
      }
      if (tracking_left && !tracking_right){
         cout << "TurnRight" << endl;
         speed = max(speed - 0.1, 0.3);
         steering_angle = max(steering_angle - 2.0, -30.0); 
         //steering_angle = -AnguloI;
       }
      if (!tracking_left && tracking_right){
         cout << "TurnLeft" << endl;
         speed = max(speed - 0.1, 0.3);
         steering_angle = min(steering_angle + 2.0, 30.0);
         //steering_angle = -AnguloD;
      }
      if (!tracking_left && !tracking_right){
         cout << "IamLost" << endl;
         speed = 0.0;
         steering_angle = 0.0;
      }

      cout << "Speed = " << speed << ", Steering_angle[deg] = " << steering_angle << endl << endl;


      ackermann_msgs::AckermannDriveStamped driveStamped;
      driveStamped.drive.speed = speed;
      driveStamped.drive.steering_angle = steering_angle*(CV_PI/180.0);
      cmd_pub.publish(driveStamped);



}


int main(int argc, char** argv){
   ros::init(argc, argv, "car_controller");
   ros::NodeHandle n;
   ros::Rate loop_rate(10);

   namedWindow("Camera");
   startWindowThread();
   image_transport::ImageTransport it(n);
   image_transport::Subscriber sub=it.subscribe("/ackermann_vehicle/camera1/image_raw",1,imageCallback);

   cmd_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_vehicle/ackermann_cmd",1);
   ackermann_msgs::AckermannDriveStamped driveStamped;

   int key;
   while(n.ok()){
      key = waitKey(1);
      if (key == 113){   // If key == 'Q'  then exit
         break;
      }
      ros::spinOnce();
   }

   destroyWindow("Camera");
   driveStamped.drive.speed = 0.0;
   driveStamped.drive.steering_angle = 0.0;
   cmd_pub.publish(driveStamped);
}
