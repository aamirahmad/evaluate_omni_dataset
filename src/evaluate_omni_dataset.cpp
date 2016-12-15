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
  }

  double errTarX = fabs(targetGTPosition.x - targetState.position.x);
  double errTarY = fabs(targetGTPosition.y - targetState.position.y);
  double errTarZ = fabs(targetGTPosition.z - targetState.position.z);

  std_msgs::Float32 error_target;
  error_target.data =
      pow(errTarX * errTarX + errTarY * errTarY + errTarZ * errTarZ, 0.5);
  targetErrorPublisher.publish(error_target);
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
  targetActive = true;
  targetState.position.x = msg->x;
  targetState.position.y = msg->y;
  targetState.position.z = msg->z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "evaluate_omni_dataset");
  ros::NodeHandle nh;

  int nRobots;

  if (!nh.getParam("MAX_ROBOTS", nRobots))
  {
    ROS_ERROR("Couldn't read parameter MAX_ROBOTS");
    nh.shutdown();
    exit(EXIT_FAILURE);
  }

  std::vector<bool> playingRobots(nRobots);

  if (!nh.getParam("PLAYING_ROBOTS", playingRobots))
  {
    ROS_ERROR("Couldn't read parameter PLAYING_ROBOTS");
    nh.shutdown();
    exit(EXIT_FAILURE);
  }

  EvaluatePFUCLT node(nh, nRobots, playingRobots);
  ros::spin();

  exit(EXIT_SUCCESS);
}
