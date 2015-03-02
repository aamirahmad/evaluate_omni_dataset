#include <cstdio>
#include <iostream>
#include <string>

#include "ros/ros.h"
#include <rosbag/bag.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <sensor_msgs/Image.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.h>
#include <read_omni_dataset/BallData.h>
#include <read_omni_dataset/LRMLandmarksData.h>
#include <read_omni_dataset/LRMGTData.h>
#include <evaluate_omni_dataset/RobotState.h>


using namespace cv;
using namespace std;
using namespace ros;


const std::size_t NUM_ROBOTS = 5;
const std::size_t NUM_TARGETS = 1;
//Right Camera PARAMETERS (CALIBRATED FROM BEFORE)
/*** Intrinsic Parameters********/
  // LRM new GT system (ethernet)
  //Left camera
  double M1[] = {952.364130338909490,0,638.552089454614702, 0,953.190083002521192,478.055570057081638, 0,0,1};
  double D1[] = {-0.239034777360406 , 0.099854524375872 , -0.000493227284393 , 0.000055426659564 , 0.000000000000000 };  
  
  //Right Camera
  double M2[] = {949.936867662258351,0,635.037661574667936,0,951.155555785186380,482.586148439616579 ,0,0,1};
  double D2[] = { -0.243312251484708 , 0.107447328223015 , -0.000324393194744 , -0.000372030056928 , 0.000000000000000 };

  static const int circlePointsPerPosition = 50;
  static const int targetPntsPos = 1;
  static const int arrowPointsPerPosition = 20;
  static const int totPntsPerPos = 1 + circlePointsPerPosition + arrowPointsPerPosition;
  static const int totPntsPerPosGT = 1 + circlePointsPerPosition;
  static const float robRadius = 0.20; //in meters
  
  
  
class ImageOverlayer
{
  IplImage img; 
  
  char imagePathNameBaseName[100];
  geometry_msgs::PoseWithCovarianceStamped robot_state[NUM_ROBOTS];
  geometry_msgs::PoseWithCovarianceStamped target_state[NUM_TARGETS];
  bool robotActive[NUM_ROBOTS];
  bool targetActive[NUM_TARGETS];
  
  NodeHandle n; 
  Subscriber Image_sub,omni_sub, target1_sub;
  
  Publisher errorPublisherOMNI1,errorPublisherOMNI3,errorPublisherOMNI4,errorPublisherOMNI5;
  
  CvMat _M2;
  CvMat _D2;
  CvMat* Tvec_right;
  CvMat* Rvec_right_n;
  CvScalar color_est[6];
  geometry_msgs::PoseWithCovariance omniGTPose[5];
  bool foundOMNI_GT[5];
  read_omni_dataset::BallData targetGTPosition[1];
  
  
  public:
    
    ImageOverlayer(char *camParamsPath, char *camImageFolderPath)
    {
     
     strcpy(imagePathNameBaseName,camImageFolderPath); 
      
     cvNamedWindow("Grount Truth Images: Right Camera"); 
     
     //Image_sub = n.subscribe<read_omni_dataset::LRMGTData>("gtData_4robotExp", 1000, boost::bind(&ImageOverlayer::gtDataCallback,this,_1,&img)); 
     
     Image_sub = n.subscribe<read_omni_dataset::LRMGTData>("gtData_4robotExp_SyncedWithg2oEstimate", 1000, boost::bind(&ImageOverlayer::gtDataCallback,this,_1,&img)); 
     
     omni_sub = n.subscribe<evaluate_omni_dataset::RobotState>("mhls_omni_poses", 1000, boost::bind(&ImageOverlayer::omniCallback,this,_1,&img));  
     
     target1_sub = n.subscribe<read_omni_dataset::BallData>("orangeBallEstimatedState", 1000, boost::bind(&ImageOverlayer::target1Callback,this,_1,&img));
     
     errorPublisherOMNI1 = n.advertise<std_msgs::Float32>("/omni1EstimationError", 1000);
     errorPublisherOMNI3 = n.advertise<std_msgs::Float32>("/omni3EstimationError", 1000);
     errorPublisherOMNI4 = n.advertise<std_msgs::Float32>("/omni4EstimationError", 1000);
     errorPublisherOMNI5 = n.advertise<std_msgs::Float32>("/omni5EstimationError", 1000);

     initializeCamParams();
     
     if (!Tvec_right || !Rvec_right_n)
      {
	  ROS_WARN("!!! cvLoad failed... overlaying on gt system will not work");
      }
      else
      {
	  ROS_INFO(" cam params loaded... overlaying should work");
      }
     
     //change these to alter colors of the robots estimated state representation
     color_est[0] = cvScalar(0.0, 50.0, 255.0);
     color_est[1] = cvScalar(0.0, 0.0, 0.0);
     color_est[2] = cvScalar(19.0, 69.0, 139.0);
     color_est[3] = cvScalar(147.0, 20.0, 255.0);
     color_est[4] = cvScalar(255.0, 50.0, 0.0);
     color_est[5] = cvScalar(100.0, 100.2, 250.1);
     foundOMNI_GT[0] = false;foundOMNI_GT[1] = false;foundOMNI_GT[2] = false;foundOMNI_GT[3] = false;foundOMNI_GT[4] = false;
     
     for(int i = 0; i<NUM_ROBOTS; i++)
	robotActive[i] = false;
    }
    
    
    ///calls the methods OverlayEstimatedRobotPose and OverlayGTRobotPose whenever a new GT message is received from the bag
    void gtDataCallback(const read_omni_dataset::LRMGTData::ConstPtr& , IplImage* );
    
    ///callback that keeps updating the most recent robot poses for omnis when it receives this info from the omni_g2o_frontend
    void omniCallback(const evaluate_omni_dataset::RobotState::ConstPtr& , IplImage* );
    
    ///callback that keeps updating the most recent target pose for target 1 (orange ball) when it receives this info from the omni_g2o_frontend
    void target1Callback(const read_omni_dataset::BallData::ConstPtr& , IplImage* );    
    
    ///Important initializer: computes the reprojection matrices based on the stereo calibration. Do not change this method for the LRM GT images!!! Port it to other stereosystems if and when necessary
    void initializeCamParams(void);
    
    ///Overlay the colored circle, and arrow for the updated robot pose
    void OverlayEstimatedRobotPose(double, double, double, double, CvScalar, IplImage*);
    
    ///Overlay the black circle for the updated GT pose of the robot
    void OverlayGTRobotPose(double, double, double, CvScalar, IplImage*, int);
    
    ///Overlay the colored circle for the updated target pose
    void OverlayEstimatedTargetPosition(double, double, double, CvScalar, IplImage* baseImage);
  
    ///Overlay the colored circle for the GT target pose
    void OverlayGTTargetPosition(double, double, double, CvScalar, IplImage* baseImage);    
};



