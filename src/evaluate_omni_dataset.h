#include <cstdio>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include <rosbag/bag.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <read_omni_dataset/BallData.h>
#include <read_omni_dataset/LRMLandmarksData.h>
#include <read_omni_dataset/LRMGTData.h>
#include <read_omni_dataset/RobotState.h>

using namespace std;
using namespace ros;

// Some fixed omni robot-specific constants
static const float robRadius = 0.20; // in meters
static const float robHeight = 0.805; // in meters

class EvaluatePFUCLT
{
  const uint& nRobots;
  const std::vector<bool>& playingRobots;

  // ROS node specific private stuff
  NodeHandle& nh;
  Subscriber robotStatesSub, target1_sub;
  std::vector<Publisher> omniErrorPublishers;
  Publisher targetErrorPublisher;

  // Estimated states
  std::vector<geometry_msgs::Pose> robotStates;
  geometry_msgs::Pose targetState;
  std::vector<bool> robotsActive;
  bool targetActive;

  // Groud Truth (GT) poses and positions using custom ROS msgs
  std::vector<geometry_msgs::Pose> omniGTPose;
  std::vector<bool> omniGTFound;
  read_omni_dataset::BallData targetGTPosition;

public:
  EvaluatePFUCLT(ros::NodeHandle& nh, const uint& nRobots,
                 const std::vector<bool>& playingRobots)
      : nh(nh), nRobots(nRobots), playingRobots(playingRobots),
        omniErrorPublishers(nRobots), robotStates(nRobots), omniGTPose(nRobots),
        robotsActive(nRobots, false), omniGTFound(nRobots, false)
  {
    // Use your own topic name here for the estimated (from another estimation
    // algorithm) poses of the omnis The idea is to compare (and find the error)
    // these estimated omni poses with the above GT poses. However make sure
    // that the received message is packed according to the custom ROS msg
    // RobotState provided with this package.
    robotStatesSub = nh.subscribe<read_omni_dataset::RobotState>(
        "estimated_omni_poses", 1000,
        boost::bind(&EvaluatePFUCLT::omniCallback, this, _1));

    // Estimated ball poses: the idea, again, is to compare it with the ball GT
    // poses. Same as robots, you can use your own topic name but the ros msg
    // type should be the custom ROSmsg BallData.
    target1_sub = nh.subscribe<read_omni_dataset::BallData>(
        "estimatedOrangeBallState", 1000,
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
};
