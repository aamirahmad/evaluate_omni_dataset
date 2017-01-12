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
  std::vector<float> targetObsNoises;
};

struct GTBuffer_s
{
  const static size_t maxBufferSize = 50;
  typedef read_omni_dataset::LRMGTData gt_t;
  uint nRobots;
  struct bufferData_s
  {
    gt_t gt;
    std::vector<float> targetObsNoises;
  };

  std::vector<bufferData_s> buffer;

  GTBuffer_s(uint nRobots) : buffer(0), nRobots(nRobots) {}

  void insertData(const read_omni_dataset::LRMGTData gt)
  {
    // If full
    if (maxBufferSize == buffer.size())
    {
      // Erase first element
      buffer.erase(buffer.begin());
    }

    // Create struct and insert into buffer
    // Copy latest noises in case no new noises are available for this GT
    bufferData_s data;
    if(!buffer.empty())
      data.targetObsNoises = buffer.back().targetObsNoises;
    else
      data.targetObsNoises = std::vector<float>(nRobots, 0.0);
    data.gt = gt;
    buffer.push_back(data);
  }

  void insertData(const float noise, const uint robotNumber)
  {
    // If buffer empty, return
    if(buffer.empty())
      return ;

    // Insert noise into latest GT
    bufferData_s& back = buffer.back();
    back.targetObsNoises[robotNumber] = noise;
  }

  bufferData_s* closestGT(ros::Time t)
  {
    //    std::cout << "Given time: " << t << std::endl;

    ros::Duration minDiff(100);
    bufferData_s* minDiffGT = NULL;

    // Look over the whole vector, find the closest data before time t
    for (std::vector<bufferData_s>::iterator it = buffer.begin();
         it != buffer.end(); ++it)
    {
      ros::Duration diff = t - it->gt.header.stamp;

      //      std::cout << "Buffer time: " << it->header.stamp << " ; diff: " <<
      //      diff
      //                << std::endl;

      if (diff < ros::Duration(0))
        continue;

      if (diff < minDiff)
      {
        minDiff = diff;
        minDiffGT = &(*it);
      }
    }

    if (!minDiffGT)
      return NULL;

    //    std::cout << "Chosen!! time: " << minDiffGT->header.stamp.toNSec()
    //              << " ; diff: " << minDiff << std::endl;

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
  std::vector<Subscriber> targetObsNoiseSubscribers;
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
      robotsActive(nRobots, false), omniGTFound(nRobots, false), hist(0),
      gtBuffer(nRobots), targetObsNoiseSubscribers(nRobots)
  {
    // Don't change the topic of this subscriber. It is the topic from rosbag
    gtSub = nh.subscribe<read_omni_dataset::LRMGTData>(
          "/gtData", 1000,
          boost::bind(&EvaluatePFUCLT::gtDataCallback, this, _1));

    estimateSub = nh.subscribe<read_omni_dataset::Estimate>(
          "/pfuclt_estimate", 100,
          boost::bind(&EvaluatePFUCLT::estimateCallback, this, _1));

    // On these topics the eindividual errors in estimation is publisher
    for (uint r = 0; r < nRobots; ++r)
    {
      omniErrorPublishers[r] = nh.advertise<std_msgs::Float32>(
            "/omni" + boost::lexical_cast<std::string>(r + 1) +
            "/EstimationError",
            1000);

      targetObsNoiseSubscribers[r] = nh.subscribe<std_msgs::Float32>(
            "/omni" + boost::lexical_cast<std::string>(r + 1) + "/targetObsNoise", 5,
            boost::bind(&EvaluatePFUCLT::targetObsNoisesCallback, this, _1, r));
    }

    targetErrorPublisher =
        nh.advertise<std_msgs::Float32>("/OrangeBallEstimationError", 1000);
  }

  /// calls the methods OverlayEstimatedRobotPose and OverlayGTRobotPose
  /// whenever a new GT message is received from the bag
  void gtDataCallback(const read_omni_dataset::LRMGTData::ConstPtr&);

  // estimate callback
  void estimateCallback(const read_omni_dataset::Estimate::ConstPtr&);

  // target obs noises callback
  void targetObsNoisesCallback(const std_msgs::Float32::ConstPtr&, uint robot);

  /// save history of desired data to file
  int saveHistory(std::string file);
};
