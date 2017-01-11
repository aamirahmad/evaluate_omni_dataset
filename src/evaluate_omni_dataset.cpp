#include "evaluate_omni_dataset.h"

void EvaluatePFUCLT::gtDataCallback(
    const read_omni_dataset::LRMGTData::ConstPtr& msg)
{
  // Test for errors
  if (msg->poseOMNI.size() != nRobots || msg->foundOMNI.size() != nRobots)
  {
    ROS_FATAL("Size of read_omni_dataset::LRMGTData vectors was different from "
              "the MAX_ROBOTS parameter %d",
              nRobots);
    nh.shutdown();
    return;
  }

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
  targetActive = msg->targetEstimate.found;
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
  for (uint r = 0; r < nRobots; ++r)
  {
    omniGTPose[r] = msgGT.poseOMNI[r].pose;
    omniGTFound[r] = msgGT.foundOMNI[r];
  }

  targetGTPosition = msgGT.orangeBall3DGTposition;

  // Begin new iteration for history
  historyIteration iterHist;

  // Publish all related errors
  for (int r = 0; r < nRobots; ++r)
  {
    if (!robotsActive[r] || !omniGTFound[r])
      continue;

    double errorX = fabs(omniGTPose[r].position.x - robotStates[r].position.x);
    double errorY = fabs(omniGTPose[r].position.y - robotStates[r].position.y);
    double error_ecldn = pow(errorX * errorX + errorY * errorY, 0.5);

    std_msgs::Float32 error_msg;
    error_msg.data = error_ecldn;
    omniErrorPublishers[r].publish(error_msg);

    // Save to history
    iterHist.robotErrors.push_back((double)error_ecldn);
    iterHist.targetVisibility.push_back((uint8_t)msg->targetVisibility[r]);
    iterHist.targetObsNoises.push_back((float)buffDataPtr->targetObsNoises[r]);
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

void EvaluatePFUCLT::targetObsNoisesCallback(
    const std_msgs::Float32::ConstPtr& noise, uint robot)
{
  gtBuffer.insertData(noise->data, robot);
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
  Output << nRobots << std::endl;

  // Begin loop for iterations, in every line we will write the info for 1
  // iteration
  for (uint32_t i = 0; i < iters; ++i)
  {
    historyIteration& iter = hist[i];

    // First write if filter is converged
    Output << iter.filterConverged;

    // Second write the computation time
    Output << " " << iter.computationTime;

    // Third write the state of global target visibility
    Output << " " << iter.targetSeen;

    // Fourth write every robot error
    for (uint r = 0; r < iter.robotErrors.size(); ++r)
    {
      Output << " " << iter.robotErrors[r];
    }

    // Fifth write the target error
    Output << " " << iter.targetError;

    // Sixth write individual target visibility
    for (uint r = 0; r < iter.targetVisibility.size(); ++r)
    {
      Output << " " << iter.targetVisibility[r];
    }

    // Seventh write individual target obs noise
    for (uint r = 0; r < nRobots; ++r)
    {
      Output << " " << iter.targetObsNoises[r];
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

  ROS_INFO("Starting evaluation for %d robots", nRobots);

  EvaluatePFUCLT node(nh, nRobots, playingRobots);
  ros::spin();

  // Node shutdown, save history to file
  if (node.saveHistory(file))
    std::cout << "Finished evaluation. Logs written to " << file << std::endl;

  exit(EXIT_SUCCESS);
}
