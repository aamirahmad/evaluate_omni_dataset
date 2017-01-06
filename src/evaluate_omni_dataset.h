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
#include <read_omni_dataset/Estimate.h>

using namespace ros;

struct historyIteration
{
  bool filterConverged;
  float computationTime;
  bool targetSeen;
  std::vector<double> robotErrors;
  double targetError;
  std::vector<bool> targetVisibility;
};

struct GTBuffer_s
{
  const static size_t maxBufferSize = 10;
  typedef read_omni_dataset::LRMGTData gt_t;
  std::vector<gt_t> buffer;

  GTBuffer_s() : buffer(0) {}

  void insertData(const read_omni_dataset::LRMGTData gt)
  {
    // If full
    if (maxBufferSize == buffer.size())
    {
      // Erase first element
      buffer.erase(buffer.begin());
    }

    // Insert into buffer
    buffer.push_back(gt);
  }

  gt_t* closestGT(ros::Time t)
  {
    ros::Duration minDiff(100);
    gt_t* minDiffGT = NULL;

    // Look over the whole vector, find the closest data before time t
    for (std::vector<gt_t>::iterator it = buffer.begin(); it != buffer.end();
         ++it)
    {
      ros::Duration diff = t - it->header.stamp;
      if (diff < ros::Duration(0))
        continue;

      if (diff < minDiff)
      {
        minDiff = diff;
        minDiffGT = &(*it);
      }
    }

    return minDiffGT;
  }
};

class EvaluatePFUCLT
{
  typedef double data_t;
  typedef std::vector<data_t> dataVec_t;

  const int nRobots;
  const std::vector<bool>& playingRobots;

  ros::Time latestStamp;

  // ROS node specific private stuff
  NodeHandle& nh;
  Subscriber estimateSub, gtSub;
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

  // History of iteration information
  std::vector<historyIteration> hist;

  // Buffer of GT data
  GTBuffer_s gtBuffer;

public:
  EvaluatePFUCLT(ros::NodeHandle& nh, const uint& nRobots,
                 const std::vector<bool>& playingRobots)
      : nh(nh), nRobots(nRobots), playingRobots(playingRobots),
        omniErrorPublishers(nRobots), robotStates(nRobots), omniGTPose(nRobots),
        robotsActive(nRobots, false), omniGTFound(nRobots, false), hist(0)
  {
    // Don't change the topic of this subscriber. It is the topic from pfuclt
    gtSub = nh.subscribe<read_omni_dataset::LRMGTData>(
        "/gtData_synced_pfuclt_estimate", 1000,
        boost::bind(&EvaluatePFUCLT::gtDataCallback, this, _1));

    estimateSub = nh.subscribe<read_omni_dataset::Estimate>(
        "/pfuclt_estimate", 100,
        boost::bind(&EvaluatePFUCLT::estimateCallback, this, _1));

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

  // estimate callback
  void estimateCallback(const read_omni_dataset::Estimate::ConstPtr&);

  /// save history of desired data to file
  int saveHistory(std::string file);
};
