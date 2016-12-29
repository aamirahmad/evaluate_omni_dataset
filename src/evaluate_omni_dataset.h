#include <cstdio>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <fstream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <read_omni_dataset/BallData.h>
#include <read_omni_dataset/LRMLandmarksData.h>
#include <read_omni_dataset/LRMGTData.h>
#include <read_omni_dataset/RobotState.h>

using namespace ros;

class EvaluatePFUCLT
{
  typedef double data_t;
  typedef std::vector<data_t> dataVec_t;

  const uint& nRobots;
  const std::vector<bool>& playingRobots;

  // ROS node specific private stuff
  NodeHandle& nh;
  Subscriber robotStatesSub, target1_sub, gtSub;
  std::vector<Publisher> omniErrorPublishers;
  Publisher targetErrorPublisher;

  // Estimated states
  std::vector<geometry_msgs::Pose> robotStates;
  read_omni_dataset::BallData targetState;
  std::vector<bool> robotsActive;
  bool targetActive;

  // Groud Truth (GT) poses and positions using custom ROS msgs
  std::vector<geometry_msgs::Pose> omniGTPose;
  std::vector<bool> omniGTFound;
  read_omni_dataset::BallData targetGTPosition;

  // Error and ball seen history
  std::vector<uint8_t> targetSeen_hist;
  dataVec_t targetErr_hist;
  std::vector<dataVec_t> robotErr_hist;

public:
  EvaluatePFUCLT(ros::NodeHandle& nh, const uint& nRobots,
                 const std::vector<bool>& playingRobots)
      : nh(nh), nRobots(nRobots), playingRobots(playingRobots),
        omniErrorPublishers(nRobots), robotStates(nRobots), omniGTPose(nRobots),
        robotsActive(nRobots, false), omniGTFound(nRobots, false),
        robotErr_hist(nRobots)
  {
    // Don't change the topic of this subscriber. It is the topic from GT
    // ROSbags.
    gtSub = nh.subscribe<read_omni_dataset::LRMGTData>(
        "/gtData_synced_pfuclt_estimate", 1000,
        boost::bind(&EvaluatePFUCLT::gtDataCallback, this, _1));

    // Use your own topic name here for the estimated (from another estimation
    // algorithm) poses of the omnis The idea is to compare (and find the error)
    // these estimated omni poses with the above GT poses. However make sure
    // that the received message is packed according to the custom ROS msg
    // RobotState provided with this package.
    robotStatesSub = nh.subscribe<read_omni_dataset::RobotState>(
        "/estimated_omni_poses", 1000,
        boost::bind(&EvaluatePFUCLT::omniCallback, this, _1));

    // Estimated ball poses: the idea, again, is to compare it with the ball GT
    // poses. Same as robots, you can use your own topic name but the ros msg
    // type should be the custom ROSmsg BallData.
    target1_sub = nh.subscribe<read_omni_dataset::BallData>(
        "/estimatedOrangeBallState", 1000,
        boost::bind(&EvaluatePFUCLT::target1Callback, this, _1));

    // On these topics the eindividual errors in estimation is publisher
    for (uint r = 0; r < nRobots; ++r)
      omniErrorPublishers[r] = nh.advertise<std_msgs::Float32>(
          "/omni" + boost::lexical_cast<std::string>(r + 1) +
              "/EstimationError",
          1000);

    targetErrorPublisher =
        nh.advertise<std_msgs::Float32>("/OrangeBallEstimationError", 1000);
  }

  /// calls the methods OverlayEstimatedRobotPose and OverlayGTRobotPose
  /// whenever a new GT message is received from the bag
  void gtDataCallback(const read_omni_dataset::LRMGTData::ConstPtr&);

  /// callback that keeps updating the most recent robot poses for omnis when it
  /// receives this info from the omni_g2o_frontend
  void omniCallback(const read_omni_dataset::RobotState::ConstPtr&);

  /// callback that keeps updating the most recent target pose for target 1
  /// (orange ball) when it receives this info from the omni_g2o_frontend
  void target1Callback(const read_omni_dataset::BallData::ConstPtr&);

  /// save history of desired data to file
  void saveHistory(std::string file);
};
