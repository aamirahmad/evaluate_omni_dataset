#include "evaluate_omni_dataset.h"

void EvaluatePFUCLT::gtDataCallback(
    const read_omni_dataset::LRMGTData::ConstPtr& msg)
{
  // Test for errors
  if (msg->poseOMNI.size() != nRobots || msg->foundOMNI.size() != nRobots)
  {
    ROS_FATAL("Size of read_omni_dataset::LRMGTData vectors was different from "
              "the NUM_ROBOTS parameter %d",
              nRobots);
    nh.shutdown();
    return;
  }

  // unpack the message into local variables
  for (uint r = 0; r < nRobots; ++r)
  {
    omniGTPose[r] = msg->poseOMNI[r].pose;
    omniGTFound[r] = msg->foundOMNI[r];
  }

  targetGTPosition = msg->orangeBall3DGTposition;

  // Publish all related errors
  for (int r = 0; r < nRobots; ++r)
  {
    if (!robotsActive[r] || !omniGTFound[r])
      continue;

    double errorX = fabs(omniGTPose[r].position.x - robotStates[r].position.x);
    double errorY = fabs(omniGTPose[r].position.y - robotStates[r].position.y);
    double error_ecldn = pow(errorX * errorX + errorY * errorY, 0.5);

    ///@INFO For each robot below, the error gets published. However, as the GT
    /// positions are not present for many time chunks due to occlusions from
    /// the
    /// GT cameras , we manually/visually inspected each GT image and the
    /// positions and noted the image intervas where the GT is absent. Hence,
    /// the
    /// magic numbers below!
    std_msgs::Float32 error_msg;
    error_msg.data = error_ecldn;
    omniErrorPublishers[r].publish(error_msg);

    // Save to history
    robotErr_hist[r].push_back((data_t)error_ecldn);
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
  targetSeen_hist.push_back((uint8_t)targetState.found);

  // Save to history
  targetErr_hist.push_back((data_t)error_ecldn);
}

void EvaluatePFUCLT::omniCallback(
    const read_omni_dataset::RobotState::ConstPtr& msg)
{
  // Test for errors
  if (msg->robotPose.size() != nRobots)
  {
    ROS_FATAL("Size of read_omni_dataset::RobotState robot vector %d is "
              "different from the NUM_ROBOTS parameter %d",
              (int)msg->robotPose.size(), nRobots);
    nh.shutdown();
    return;
  }

  // At this time turn on all the robots that are "playing"
  robotsActive = playingRobots;

  // Copy all information on the robot states
  for (uint r = 0; r < nRobots; ++r)
    robotStates[r] = msg->robotPose[r].pose;
}

void EvaluatePFUCLT::target1Callback(
    const read_omni_dataset::BallData::ConstPtr& msg)
{
  // estimated target state obtained from the corresponding rostopic
  targetActive = msg->found;
  targetState.x = msg->x;
  targetState.y = msg->y;
  targetState.z = msg->z;
  targetState.found = msg->found;
}

void EvaluatePFUCLT::saveHistory(std::string file)
{
  std::ofstream Output(file);
  if (!Output.is_open())
  {
    std::cout << "Couldn't open file " << file << std::endl;
    return;
  }

  size_t sz = targetSeen_hist.size();
  bool flag_diffSize = (sz != targetErr_hist.size());
  for (auto& robot : robotErr_hist)
    flag_diffSize |= (sz != robot.size());

  Output << sz << std::endl;

  std::copy(targetSeen_hist.begin(), targetSeen_hist.end(),
            std::ostream_iterator<int>(Output, " "));
  Output << std::endl;
  ;

  std::copy(targetErr_hist.begin(), targetErr_hist.end(),
            std::ostream_iterator<data_t>(Output, " "));
  Output << std::endl;
  ;

  int nr = robotErr_hist.size();
  for (int r = 0; r < nr; ++r)
  {
    std::copy(robotErr_hist[r].begin(), robotErr_hist[r].end(),
              std::ostream_iterator<data_t>(Output, " "));
    Output << std::endl;
    ;
  }

  if (flag_diffSize)
    std::cout << "Beware - different sizes for the history vectors"
              << std::endl;

  // closed by leaving scope
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
    nh.shutdown();
    exit(EXIT_FAILURE);
  }

  if (!nh.getParam("/MAX_ROBOTS", nRobots))
  {
    ROS_ERROR("Couldn't read parameter MAX_ROBOTS");
    nh.shutdown();
    exit(EXIT_FAILURE);
  }

  std::vector<bool> playingRobots(nRobots);

  if (!nh.getParam("/PLAYING_ROBOTS", playingRobots))
  {
    ROS_ERROR("Couldn't read parameter PLAYING_ROBOTS");
    nh.shutdown();
    exit(EXIT_FAILURE);
  }

  EvaluatePFUCLT node(nh, nRobots, playingRobots);
  ros::spin();

  // Node shutdown, save history to file
  node.saveHistory(file);

  exit(EXIT_SUCCESS);
}
