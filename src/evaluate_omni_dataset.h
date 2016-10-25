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
#include <read_omni_dataset/RobotState.h>


using namespace cv;
using namespace std;
using namespace ros;

// Dataset related constants

///@INFO In truth we have only 4 robots in the dataset but since the robot IDs go upto 5, we will keep a phantom robot: OMNI2
const std::size_t NUM_ROBOTS = 5;  
///@INFO In truth we have 2 targets in the raw dataset but the 'pre-processed' data does not contain the measurements of the blue ball from the robots (we have that only for the orange ball). However, extend this to 2 in case you do your own pre-processing for the blue ball measurements using the raw images from the robot's camera. Check raw dataset details for other related params to perform this measurement
const std::size_t NUM_TARGETS = 1;

/******** Intrinsic Parameters ********/
/******** !!!!DO NOT MODIFY!!! ********/

///@INFO The params below are of the individual cameras of the GT camera system.
///@INFO Extrinsic stere calibration is performed on the fly (to avoid carrying a lot of xml files and make this code more portable) and called at the beginning of this node. Check method initializeCamParams()

//Left camera
double M1[] = {952.364130338909490,0,638.552089454614702, 0,953.190083002521192,478.055570057081638, 0,0,1};
double D1[] = {-0.239034777360406 , 0.099854524375872 , -0.000493227284393 , 0.000055426659564 , 0.000000000000000 };  

//Right Camera
double M2[] = {949.936867662258351,0,635.037661574667936,0,951.155555785186380,482.586148439616579 ,0,0,1};
double D2[] = { -0.243312251484708 , 0.107447328223015 , -0.000324393194744 , -0.000372030056928 , 0.000000000000000 };

// Some constants for drawing funcions... change these values if you want more points on the overlaid circle... an so on
static const int circlePointsPerPosition = 50;
static const int targetPntsPos = 1;
static const int arrowPointsPerPosition = 20;
static const int totPntsPerPos = 1 + circlePointsPerPosition + arrowPointsPerPosition;
static const int totPntsPerPosGT = 1 + circlePointsPerPosition;

// Some fixed omni robot-specific constants 
static const float robRadius = 0.20; //in meters
static const float robHeight = 0.805; //in meters  
  
  
class ImageOverlayer
{
  //ROS node specific private stuff
  NodeHandle n; 
  Subscriber Image_sub,omni_sub, target1_sub;
  Publisher errorPublisherOMNI1,errorPublisherOMNI3,errorPublisherOMNI4,errorPublisherOMNI5;
  Publisher orangeBallError;
  //Main image
  IplImage img;   
  char imagePathNameBaseName[100];
  
  //Estimated states
  geometry_msgs::PoseWithCovarianceStamped robot_state[NUM_ROBOTS];
  geometry_msgs::PoseWithCovarianceStamped target_state[NUM_TARGETS];
  bool robotActive[NUM_ROBOTS];
  bool targetActive[NUM_TARGETS];
  
  // GT Stereo system calibration params
  CvMat _M1;
  CvMat _D1;  
  CvMat _M2;
  CvMat _D2;
  CvMat* Tvec_right;
  CvMat* Rvec_right_n;
  CvMat* Tvec_left;
  CvMat* Rvec_left_n;  
  
  //Groud Truth (GT) poses and positions using custom ROS msgs
  CvScalar color_est[6];
  geometry_msgs::PoseWithCovariance omniGTPose[5];
  bool foundOMNI_GT[5];
  read_omni_dataset::BallData targetGTPosition[1];
  
  
  public:
    ImageOverlayer(char *camImageFolderPath)
    {
     
     strcpy(imagePathNameBaseName,camImageFolderPath); 
      
     cvNamedWindow("Overlaid Estimates on the Grount Truth Images of Right GT Camera"); 
     
     //Don't change the topic of this subscriber. It is the topic from GT ROSbags.
     Image_sub = n.subscribe<read_omni_dataset::LRMGTData>("gtData_4robotExp", 1000, boost::bind(&ImageOverlayer::gtDataCallback,this,_1,&img)); 
     
     //Use your own topic name here for the estimated (from another estimation algorithm) poses of the omnis The idea is to compare (and find the error) these estimated omni poses with the above GT poses. However make sure that the received message is packed according to the custom ROS msg RobotState provided with this package.
     omni_sub = n.subscribe<read_omni_dataset::RobotState>("estimated_omni_poses", 1000, boost::bind(&ImageOverlayer::omniCallback,this,_1,&img));  
     
     //Estimated ball poses: the idea, again, is to compare it with the ball GT poses. Same as robots, you can use your own topic name but the ros msg type should be the custom ROSmsg BallData.
     target1_sub = n.subscribe<read_omni_dataset::BallData>("estimatedOrangeBallState", 1000, boost::bind(&ImageOverlayer::target1Callback,this,_1,&img));
     
     //On these topics the eindividual errors in estimation is published
     errorPublisherOMNI1 = n.advertise<std_msgs::Float32>("/omni1/EstimationError", 1000);
     errorPublisherOMNI3 = n.advertise<std_msgs::Float32>("/omni3/EstimationError", 1000);
     errorPublisherOMNI4 = n.advertise<std_msgs::Float32>("/omni4/EstimationError", 1000);
     errorPublisherOMNI5 = n.advertise<std_msgs::Float32>("/omni5/EstimationError", 1000);
     orangeBallError = n.advertise<std_msgs::Float32>("/OrangeBallEstimationError", 1000);
     
     // Stereo cam calibration. Done only once in the beginning
     initializeCamParams();
     
     //change these to alter colors of the robots estimated state representation
     color_est[0] = cvScalar(0.0, 50.0, 255.0);
     color_est[1] = cvScalar(0.0, 0.0, 0.0);
     color_est[2] = cvScalar(19.0, 69.0, 139.0);
     color_est[3] = cvScalar(147.0, 20.0, 255.0);
     color_est[4] = cvScalar(255.0, 50.0, 0.0);
     color_est[5] = cvScalar(100.0, 100.2, 250.1);
     
     //some bool initialization
     foundOMNI_GT[0] = false;foundOMNI_GT[1] = false;foundOMNI_GT[2] = false;foundOMNI_GT[3] = false;foundOMNI_GT[4] = false;     
     for(int i = 0; i<NUM_ROBOTS; i++)
	robotActive[i] = false;
    }
    
    
    ///calls the methods OverlayEstimatedRobotPose and OverlayGTRobotPose whenever a new GT message is received from the bag
    void gtDataCallback(const read_omni_dataset::LRMGTData::ConstPtr& , IplImage* );
    
    ///callback that keeps updating the most recent robot poses for omnis when it receives this info from the omni_g2o_frontend
    void omniCallback(const read_omni_dataset::RobotState::ConstPtr& , IplImage* );
    
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