#include "evaluate_omni_dataset.h"

void EvaluatePFUCLT::gtDataCallback(
    const read_omni_dataset::LRMGTData::ConstPtr& msg)
{
  targetActive = msg->orangeBall3DGTposition.found;
  // Save to buffer
  gtBuffer.insertData(*msg);
}

void EvaluatePFUCLT::estimateCallback(
    const read_omni_dataset::Estimate::ConstPtr& msg)
{
  latestStamp = msg->header.stamp;

  // Test for errors
  if (msg->robotEstimates.size() != nRobots)
  {
    ROS_FATAL("Size of read_omni_dataset::Estimate.robotEstimates vector %d is "
              "different from the MAX_ROBOTS parameter %d",
              (int)msg->robotEstimates.size(), nRobots);
    nh.shutdown();
    return;
  }

  // At this time turn on all the robots that are "playing"
  robotsActive = playingRobots;

  // Copy all information on the robot states
  for (uint r = 0; r < nRobots; ++r)
    robotStates[r] = msg->robotEstimates[r];

  // Estimated target state
  targetState.x = msg->targetEstimate.x;
  targetState.y = msg->targetEstimate.y;
  targetState.z = msg->targetEstimate.z;
  targetState.found = msg->targetEstimate.found;

  // ----------------------
  // Begin error evaluation
  // ----------------------

  if (gtBuffer.buffer.empty())
    return;

  // find closest GT data to latest stamp
  GTBuffer_s::bufferData_s* buffDataPtr = gtBuffer.closestGT(latestStamp);

  if (!buffDataPtr)
    buffDataPtr = &gtBuffer.buffer.back();

  read_omni_dataset::LRMGTData& msgGT = buffDataPtr->gt;

  // unpack the message into local variables
  for (int r = 0; r < nRobots; ++r)
  {
    if (!robotsActive[r])
    {
      continue;
    }

    switch (r)
    {
    case 0:
      omniGTPose[r] = msgGT.poseOMNI1.pose;
      omniGTFound[r] = msgGT.foundOMNI1;
      break;

    case 2:
      omniGTPose[r] = msgGT.poseOMNI3.pose;
      omniGTFound[r] = msgGT.foundOMNI3;
      break;

    case 3:
      omniGTPose[r] = msgGT.poseOMNI4.pose;
      omniGTFound[r] = msgGT.foundOMNI4;
      break;

    case 4:
      omniGTPose[r] = msgGT.poseOMNI5.pose;
      omniGTFound[r] = msgGT.foundOMNI5;
      break;
    }
  }

  targetGTPosition = msgGT.orangeBall3DGTposition;

  // Begin new iteration for history
  historyIteration iterHist;

  iterHist.msgSeq = msgGT.header.seq;
  iterHist.targetFoundGT = msgGT.orangeBall3DGTposition.found;
  iterHist.omniGTfound = omniGTFound;

  // Publish all related errors
  for (int r = 0; r < nRobots; ++r)
  {
    if (!robotsActive[r])
    {
      iterHist.robotErrors.push_back(0);
      iterHist.targetVisibility.push_back(0);
      continue;
    }

    double errorX = fabs(omniGTPose[r].position.x - robotStates[r].position.x);
    double errorY = fabs(omniGTPose[r].position.y - robotStates[r].position.y);
    double error_ecldn = pow(errorX * errorX + errorY * errorY, 0.5);

    std_msgs::Float32 error_msg;
    error_msg.data = error_ecldn;
    omniErrorPublishers[r].publish(error_msg);

    // Save to history
    iterHist.robotErrors.push_back((double)error_ecldn);
    iterHist.targetVisibility.push_back((uint8_t)msg->targetVisibility[r]);
  }

  double errTarX = fabs(targetGTPosition.x - targetState.x);
  double errTarY = fabs(targetGTPosition.y - targetState.y);
  double errTarZ = fabs(targetGTPosition.z - targetState.z);
  double error_ecldn =
      pow(errTarX * errTarX + errTarY * errTarY + errTarZ * errTarZ, 0.5);

  std_msgs::Float32 error_target;
  error_target.data = error_ecldn;

  targetErrorPublisher.publish(error_target);

  // Save to history
  iterHist.targetSeen = ((uint8_t)targetState.found);

  // Save to history
  iterHist.targetError = ((double)error_ecldn);

  std::cout << "Difference from GT to estimate = "
            << latestStamp - msgGT.header.stamp << std::endl;

  // Save to history
  iterHist.computationTime = msg->computationTime;
  iterHist.filterConverged = msg->converged;

  hist.push_back(iterHist);
}

int EvaluatePFUCLT::saveHistory(std::string file)
{
  std::ofstream Output(file);
  if (!Output.is_open())
  {
    std::cout << "Couldn't open file " << file << std::endl;
    return 0;
  }

  size_t iters = hist.size();
  // Write number of iterations to first line
  Output << iters << std::endl;

  // Write number of robots to second line
  uint nRobotsInactive =
      std::count(robotsActive.begin(), robotsActive.end(), 0);
  Output << nRobots - nRobotsInactive << std::endl;

  // Begin loop for iterations, in every line we will write the info for 1
  // iteration
  for (uint32_t i = 0; i < iters; ++i)
  {
    historyIteration& iter = hist[i];

    // First write if filter is converged
    Output << iter.filterConverged;

    // Second write the computation time
    Output << " " << iter.computationTime;

    // Third write the state of global target visibility (only 1 if found by GT
    // and robots)
    Output << " " << iter.targetSeen&& iter.targetFoundGT;

    // Fourth write every robot error
    for (uint r = 0; r < nRobots; ++r)
    {
      if (!robotsActive[r])
        continue;

      else if (!iter.omniGTfound[r])
      {
        iter.robotErrors[r] = NAN;
      }

      // Check some bad GT values - write NaN so that we can filter later
      else if (r == 0)
      {
        if (!((iter.msgSeq < 1570) ||
              (iter.msgSeq >= 1607 && iter.msgSeq <= 2870) ||
              (iter.msgSeq >= 3336 && iter.msgSeq <= 3800) ||
              (iter.msgSeq >= 3816 && iter.msgSeq <= 5030) ||
              (iter.msgSeq >= 5197 && iter.msgSeq <= 5870)))
          iter.robotErrors[r] = NAN;
      }
      else if (r == 2)
      {
        if (!((iter.msgSeq < 1500) ||
              (iter.msgSeq >= 2871 && iter.msgSeq <= 2950) ||
              (iter.msgSeq >= 3006 && iter.msgSeq <= 3750) ||
              (iter.msgSeq >= 3797 && iter.msgSeq <= 4260) ||
              (iter.msgSeq >= 5442 && iter.msgSeq <= 5540) ||
              (iter.msgSeq > 5567)))
          iter.robotErrors[r] = NAN;
      }
      else if (r == 3)
      {
        if (!((iter.msgSeq < 555) ||
              (iter.msgSeq >= 564 && iter.msgSeq <= 1080) ||
              (iter.msgSeq >= 2314 && iter.msgSeq <= 3724) ||
              (iter.msgSeq > 4526)))
          iter.robotErrors[r] = NAN;
      }
      else if (r == 4)
      {
        if (!((iter.msgSeq < 530) ||
              (iter.msgSeq >= 1276 && iter.msgSeq <= 2260) ||
              (iter.msgSeq >= 3069 && iter.msgSeq <= 3273) ||
              (iter.msgSeq >= 3300 && iter.msgSeq <= 3800) ||
              (iter.msgSeq >= 4335 && iter.msgSeq <= 5173) ||
              (iter.msgSeq >= 5191 && iter.msgSeq <= 5240)))
          iter.robotErrors[r] = NAN;
      }

      Output << " " << iter.robotErrors[r];
    }

    // Fifth write the target error
    Output << " " << iter.targetError;

    // Sixth write individual target visibility
    for (uint r = 0; r < nRobots; ++r)
    {
      if (!robotsActive[r])
        continue;

      Output << " " << iter.targetVisibility[r];
    }

    // Go to next line
    Output << std::endl;
  }

  // closed by leaving scope
  return 1;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "evaluate_omni_dataset");
  ros::NodeHandle nh("~");

  int nRobots;

  std::string file;
  if (!nh.getParam("file", file))
  {
    ROS_ERROR("Couldn't read parameter file");
    ROS_BREAK();
  }

  while (!nh.hasParam("/MAX_ROBOTS"))
  {
    ROS_INFO("Waiting for parameter /MAX_ROBOTS, sleeping 1 second");
    ros::WallDuration(1).sleep();
  }

  nh.getParam("/MAX_ROBOTS", nRobots);

  std::vector<bool> playingRobots(nRobots);

  if (!nh.getParam("/PLAYING_ROBOTS", playingRobots))
  {
    ROS_ERROR("Couldn't read parameter PLAYING_ROBOTS");
    ROS_BREAK();
  }

  ROS_INFO(
      "Starting evaluation for max %d robots (there may be inactive robots)",
      nRobots);

  EvaluatePFUCLT node(nh, nRobots, playingRobots);
  ros::spin();

  // Node shutdown, save history to file
  if (node.saveHistory(file))
    std::cout << "Finished evaluation. Logs written to " << file << std::endl;

  exit(EXIT_SUCCESS);
}
