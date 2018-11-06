#ifndef FIND_ROAD_H
#define FIND_ROAD_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

int width = 640, heigth = 480;
Mat image_bgr = Mat(heigth, width, CV_8UC3, Scalar(0,0,0));
int cut = 370;
int h_cut = 100;

Mat image_crop = Mat(heigth - cut, width, CV_8UC3, Scalar(0,0,0));
Mat image_bw   = Mat(heigth - cut, width, CV_8UC1, Scalar(0));

Mat element, labels, stats, centroids;
int nLabels, the_max, index_max;
int midWidth = width / 2.0;
int cut_number = 220;
Mat mask_leftL  = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
Mat mask_rightL = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
Mat mask_leftG  = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
Mat mask_rightG = Mat(heigth - cut, width, CV_8UC1, Scalar(0));


Mat and_masks;
double min_and_masks, max_and_masks;

bool encimadas = false;


double steering_angle=0.0, speed=0.0;
bool tracking_left = false, tracking_right = false;
float PuntoI, PuntoD, AnguloI, AnguloD;



ros::Publisher cmd_pub;



struct driveStruct {
  float steering_angle;
  float speed;
};
/**
   Class that subscribes to the image stream from the camera on the ackermann vehicle,
   and implements some basic computer vision methods to find the road ahead. 
   The class is header-file only. You only need to include the header file in your code.
 */
class RoadFinder
{

public:
  RoadFinder()
  {

  }

  driveStruct imageProcessing(const cv::Mat &image_bgr, float k1, float k2, int erosion_size = 10)
  {
      // line(image_bgr, Point(0,cut), Point(width, cut), Scalar(255,0,0), 3, LINE_AA);

      // imshow("Camera", image_bgr);
      // waitKey(1);

      encimadas  = false;

      // Basic image pre-traitment
      Rect myROI(h_cut, cut, width - 2*h_cut, heigth - cut); // myROI(x,y,w,h)
      image_crop = image_bgr(myROI);
      //cvtColor(image_crop, image_gray, COLOR_BGR2GRAY);
      blur(image_crop, image_crop, Size(3,3));

      // imshow("Camera", image_crop);
      // waitKey(1);

      midWidth = image_crop.cols/2;

      

      inRange(image_crop, Scalar(0, 0, 0), Scalar(60, 255, 100), image_bw);
      
      imshow("Camera", image_bw);
      // waitKey(1);





      int erosion_type = MORPH_ELLIPSE; //  Other options: MORPH_RECT; MORPH_CROSS;
      element = getStructuringElement( erosion_type,
                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       Point( erosion_size, erosion_size ) );

      



      Mat mask_leftL(image_crop.rows, image_crop.cols, CV_8UC1, Scalar(0));
      Mat mask_rightL(image_crop.rows, image_crop.cols, CV_8UC1, Scalar(0));
      if (tracking_left == false){
         mask_leftL(Range(image_crop.rows *.9, image_crop.rows), Range(0, cut_number)) = 255;
         mask_leftG = mask_leftL.clone();
         //cout << "Inicializo MASCARA izq" << endl;
      }
      if (tracking_right == false){
         mask_rightL(Range(image_crop.rows *.9 , image_crop.rows), Range(image_crop.cols - cut_number, image_crop.cols)) = 255;
         mask_rightG = mask_rightL.clone();
         //cout << "Inicializo MASCARA der" << endl;
      }

      Mat image_lineLeft, image_lineRight;
      bitwise_and(image_bw, mask_leftG , image_lineLeft );
      bitwise_and(image_bw, mask_rightG, image_lineRight);

      // imshow("Camera_lineright", image_lineRight);
      imshow("camera_left", mask_leftG);
      imshow("Camera_rightg", mask_rightG);
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
            PuntoI = (Par.at<float>(0,0) * image_crop.rows + Par.at<float>(0,1));
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
            PuntoD = (Par.at<float>(0,0) * image_crop.rows + Par.at<float>(0,1));
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

      imshow("ambos carriles", image_crop);
      waitKey(1);

      ////////////////////////////////////////////////////////////

      bitwise_and(mask_leftG, mask_rightG, and_masks);
      
      
      minMaxLoc(and_masks, &min_and_masks, &max_and_masks );

      if(max_and_masks >= 255.0){
        
        encimadas = true;
      }

      ////////////////////////////////////////////////////////////


      if (tracking_left && tracking_right){
        cout << "Left: " << AnguloI << " Right: " << AnguloD << endl;
        if(encimadas){
          cout << "Encimadas: " << steering_angle << endl;
          float promedio_angulo = (AnguloD + AnguloI)/2;
          if(promedio_angulo > 0){
            //Go right
            steering_angle = -14;
            speed = min(speed + 0.01, 0.1);

          }
          else{
            //Go left
            steering_angle = +14;
            speed = min(speed + 0.01, 0.1);
          }
        }
        else{
          //No están encimadas
          steering_angle = -((PuntoD+PuntoI)/2.0 - midWidth) * k1;
          //steering_angle = -(AnguloI+AnguloD)/2.0;
          steering_angle = max(-14.0, min(steering_angle, 14.0));
          cout << steering_angle << endl;
          speed = min(speed + 0.01, 0.2);
        }


        //  float angle_diff = AnguloD - AnguloI;
        //  if(fabs(angle_diff) < 8){
           
        //    if(angle_diff > 0){
        //      steering_angle = -14.0;
        //    }
        //    else{
        //      steering_angle = +14.0;
        //    }
        
          
        // }
           

           
         
        //  else{
        //   steering_angle = -((PuntoD+PuntoI)/2.0 - midWidth) * k1;
        //   //steering_angle = -(AnguloI+AnguloD)/2.0;
        //   steering_angle = max(-14.0, min(steering_angle, 14.0));
        //   cout << steering_angle << endl;
        //   speed = min(speed + 0.01, 0.3);
        //  }
      }
      if (tracking_left && !tracking_right){
         cout << "TurnRight" << endl;
         speed = max(speed - 0.1, 0.1);
         steering_angle = max(steering_angle - k2, -14.0); 
         //steering_angle = -AnguloI;
       }
      if (!tracking_left && tracking_right){
         cout << "TurnLeft" << endl;
         speed = max(speed - 0.1, 0.1);
         steering_angle = min(steering_angle + k2, 14.0);
         //steering_angle = -AnguloD;
      }
      if (!tracking_left && !tracking_right){
         cout << "IamLost" << endl;
         speed = 0.0;
        //  steering_angle = 0.0;
      }

      // cout << "Speed = " << speed << ", Steering_angle[deg] = " << steering_angle << endl << endl;
      driveStruct message;

      message.speed = speed;
      message.steering_angle = steering_angle;

      return message;
      // ackermann_msgs::AckermannDriveStamped driveStamped;
      // driveStamped.drive.speed = speed;
      // driveStamped.drive.steering_angle = steering_angle*(CV_PI/180.0);
      // cmd_pub.publish(driveStamped);


  }
};

#endif