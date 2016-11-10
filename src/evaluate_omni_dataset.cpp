#include "evaluate_omni_dataset.h"

inline void getActualShift(int i, double* shift_a_, double* shift_b_)
{

  if (i == 0)
  {
    *shift_a_ = 0.098947;
    *shift_b_ = 0.02391;
  }
  if (i == 2)
  {
    *shift_a_ = 0.153636;
    *shift_b_ = 0.025022;
  }
  if (i == 3)
  {
    *shift_a_ = 0.061141;
    *shift_b_ = 0.0272;
  }
  if (i == 4)
  {
    *shift_a_ = 0.102613;
    *shift_b_ = 0.01631;
  }
}

void ImageOverlayer::initializeCamParams()
{
  int nframes = 1;
  int n = 9; // Number of points used to process
  int N = nframes * n;

  vector<CvPoint2D32f> temp(n);
  vector<int> npoints;
  vector<CvPoint3D32f> objectPoints;
  vector<CvPoint2D32f> points[2];
  points[0].resize(n);
  points[1].resize(n);

  double R[3][3], T[3], E[3][3], F[3][3];
  double Q[4][4];

  /*************************************************************/

  _M1 = cvMat(3, 3, CV_64F, M1);
  _M2 = cvMat(3, 3, CV_64F, M2);
  _D1 = cvMat(1, 5, CV_64F, D1);
  _D2 = cvMat(1, 5, CV_64F, D2);
  CvMat _R = cvMat(3, 3, CV_64F, R);
  CvMat _T = cvMat(3, 1, CV_64F, T);
  CvMat _E = cvMat(3, 3, CV_64F, E);
  CvMat _F = cvMat(3, 3, CV_64F, F);
  CvMat _Q = cvMat(4, 4, CV_64F, Q);

  vector<CvPoint2D32f>& pts = points[0];

  // Pontos XY pixels left
  points[0][0].x = 34.0;
  points[0][0].y = 336.0;
  points[0][1].x = 502.0;
  points[0][1].y = 156.0;
  points[0][2].x = 1280.0;
  points[0][2].y = 279.0;
  points[0][3].x = 664.0;
  points[0][3].y = 174.0;
  points[0][4].x = 914.0;
  points[0][4].y = 209.0;
  points[0][5].x = 248.0;
  points[0][5].y = 300.0;
  points[0][6].x = 663.0;
  points[0][6].y = 482.0;
  points[0][7].x = 185.0;
  points[0][7].y = 364.0;
  points[0][8].x = 400.0;
  points[0][8].y = 507.0;

  // Pontos XY pixels right
  points[1][0].x = 866.0;
  points[1][0].y = 942.0;
  points[1][1].x = 98.0;
  points[1][1].y = 376.0;
  points[1][2].x = 856.0;
  points[1][2].y = 72.0;
  points[1][3].x = 445.0;
  points[1][3].y = 222.0;
  points[1][4].x = 690.0;
  points[1][4].y = 128.0;
  points[1][5].x = 779.0;
  points[1][5].y = 442.0;
  points[1][6].x = 1162.0;
  points[1][6].y = 161.0;
  points[1][7].x = 1061.0;
  points[1][7].y = 413.0;
  points[1][8].x = 1244.0;
  points[1][8].y = 215.0;

  npoints.resize(nframes, n);
  objectPoints.resize(nframes * n);

  /* 3D points (x,y,z) related to the 2D points from left and right image -
   * minimum: 8 points*/

  objectPoints[0] = cvPoint3D32f(6.0, -4.5, 0.0);
  objectPoints[1] = cvPoint3D32f(0.0, -4.5, 0.0);
  objectPoints[2] = cvPoint3D32f(0.0, +4.5, 0.0);
  objectPoints[3] = cvPoint3D32f(0.0, -1.5, 0.0);
  objectPoints[4] = cvPoint3D32f(0.0, +1.5, 0.0);
  objectPoints[5] = cvPoint3D32f(4.3, -2.7, 0.0);
  objectPoints[6] = cvPoint3D32f(4.3, +2.7, 0.0);
  objectPoints[7] = cvPoint3D32f(5.25, -1.75, 0.0);
  objectPoints[8] = cvPoint3D32f(5.25, +1.75, 0.0);

  for (int i = 1; i < nframes; i++)
    copy(objectPoints.begin(), objectPoints.begin() + n,
         objectPoints.begin() + i * n);

  CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0]);
  CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0]);
  CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0]);
  CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0]);

  /**************************************************************************/
  double R1[3][3], R2[3][3], P1[3][4], P2[3][4];

  CvMat _R1 = cvMat(3, 3, CV_64F, R1);
  CvMat _R2 = cvMat(3, 3, CV_64F, R2);
  CvMat _P1 = cvMat(3, 4, CV_64F, P1);
  CvMat _P2 = cvMat(3, 4, CV_64F, P2);

  /************************************** StereoCalibration - returns R T R1 R2
   * T1 T2 **************************************/
  CvSize imageSize = cvSize(1294, 964);

  cvStereoCalibrate(
      &_objectPoints, &_imagePoints1, &_imagePoints2, &_npoints, &_M1, &_D1,
      &_M2, &_D2, imageSize, &_R, &_T, &_E, &_F,
      cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 1e-5),
      CV_CALIB_USE_INTRINSIC_GUESS + CV_CALIB_FIX_ASPECT_RATIO);

  /*************************************************************************************************************************/

  /***************************************** Extrinsic Parameters
   * **********************************************************/
  CvMat* Rvec_left = cvCreateMat(3, 1, CV_64F);
  Tvec_left = cvCreateMat(3, 1, CV_64F);
  CvMat* Rvec_right = cvCreateMat(3, 1, CV_64F);
  Tvec_right = cvCreateMat(3, 1, CV_64F);

  cvFindExtrinsicCameraParams2(&_objectPoints, &_imagePoints1, &_M1, &_D1,
                               Rvec_left, Tvec_left);

  Rvec_left_n = cvCreateMat(3, 3, CV_64F);
  cvRodrigues2(Rvec_left, Rvec_left_n, 0);

  cvFindExtrinsicCameraParams2(&_objectPoints, &_imagePoints2, &_M2, &_D2,
                               Rvec_right, Tvec_right);

  Rvec_right_n = cvCreateMat(3, 3, CV_64F);
  cvRodrigues2(Rvec_right, Rvec_right_n, 0);

  /*************************************************************************************************************************/
}

void ImageOverlayer::OverlayGTTargetPosition(double x, double y, double z,
                                             CvScalar color,
                                             IplImage* baseImage)
{
  vector<CvPoint3D32f> Target_PositionsToReProject;
  Target_PositionsToReProject.resize(targetPntsPos);

  CvMat _Target_PositionsToReProject =
      cvMat(1, targetPntsPos, CV_32FC3, &Target_PositionsToReProject[0]);

  Target_PositionsToReProject[0] = cvPoint3D32f(x, y, z);

  vector<CvPoint2D32f> reprojectedPoints_Target;
  reprojectedPoints_Target.resize(targetPntsPos);

  CvMat _imageReprojectedPoints_TargetRight =
      cvMat(1, targetPntsPos, CV_32FC2, &reprojectedPoints_Target[0]);

  cvProjectPoints2(&_Target_PositionsToReProject, Rvec_right_n, Tvec_right,
                   &_M2, &_D2, &_imageReprojectedPoints_TargetRight, NULL, NULL,
                   NULL, NULL, NULL);

  CvPoint target_PointToBeShownRight =
      cvPoint(reprojectedPoints_Target[0].x, reprojectedPoints_Target[0].y);
  cvCircle(baseImage, target_PointToBeShownRight, 20, color, 2, 8, 0);
}

// This function is redundant... merge it with the previous one!
void ImageOverlayer::OverlayEstimatedTargetPosition(double x, double y,
                                                    double z, CvScalar color,
                                                    IplImage* baseImage)
{
  vector<CvPoint3D32f> Target_PositionsToReProject;
  Target_PositionsToReProject.resize(targetPntsPos);

  CvMat _Target_PositionsToReProject =
      cvMat(1, targetPntsPos, CV_32FC3, &Target_PositionsToReProject[0]);

  Target_PositionsToReProject[0] = cvPoint3D32f(x, y, z);

  vector<CvPoint2D32f> reprojectedPoints_Target;
  reprojectedPoints_Target.resize(targetPntsPos);

  CvMat _imageReprojectedPoints_TargetRight =
      cvMat(1, targetPntsPos, CV_32FC2, &reprojectedPoints_Target[0]);

  cvProjectPoints2(&_Target_PositionsToReProject, Rvec_right_n, Tvec_right,
                   &_M2, &_D2, &_imageReprojectedPoints_TargetRight, NULL, NULL,
                   NULL, NULL, NULL);

  CvPoint target_PointToBeShownRight =
      cvPoint(reprojectedPoints_Target[0].x, reprojectedPoints_Target[0].y);
  cvCircle(baseImage, target_PointToBeShownRight, 20, color, 2, 8, 0);
}

void ImageOverlayer::OverlayEstimatedRobotPose(double x, double y, double z,
                                               double thetaRob, CvScalar color,
                                               IplImage* baseImage)
{

  vector<CvPoint3D32f> Robot_PositionsToReProject;
  Robot_PositionsToReProject.resize(totPntsPerPos);

  CvMat _Robot_PositionsToReProject =
      cvMat(1, totPntsPerPos, CV_32FC3, &Robot_PositionsToReProject[0]);

  Robot_PositionsToReProject[0] = cvPoint3D32f(x, y, robHeight);
  for (int pts = 0; pts < circlePointsPerPosition;
       pts++) // circlePointsPerPosition points out of totPntsPerPos for circle
  {
    float theta = -M_PI + (float)pts * 2 * M_PI / (circlePointsPerPosition);
    Robot_PositionsToReProject[1 + pts] = cvPoint3D32f(
        x + robRadius * cosf(theta), y + robRadius * sinf(theta), robHeight);
  }
  for (int pts = 0; pts < arrowPointsPerPosition;
       pts++) // arrowPointsPerPosition points out of totPntsPerPos for th arrow
  {
    Robot_PositionsToReProject[1 + circlePointsPerPosition + pts] =
        cvPoint3D32f(
            x +
                (float)pts * (robRadius / (float)arrowPointsPerPosition) *
                    cosf(thetaRob),
            y +
                (float)pts * (robRadius / (float)arrowPointsPerPosition) *
                    sinf(thetaRob),
            robHeight);
  }

  vector<CvPoint2D32f> reprojectedPoints_Robot;
  reprojectedPoints_Robot.resize(totPntsPerPos);

  CvMat _imageReprojectedPoints_RobotRight =
      cvMat(1, totPntsPerPos, CV_32FC2, &reprojectedPoints_Robot[0]);

  cvProjectPoints2(&_Robot_PositionsToReProject, Rvec_right_n, Tvec_right, &_M2,
                   &_D2, &_imageReprojectedPoints_RobotRight, NULL, NULL, NULL,
                   NULL, NULL);

  for (int pts = 0; pts < totPntsPerPos; pts++)
  {
    CvPoint robot_PointToBeShownRight =
        cvPoint(reprojectedPoints_Robot[pts].x, reprojectedPoints_Robot[pts].y);
    cvCircle(baseImage, robot_PointToBeShownRight, 0, color, 2, 8, 0);
  }
}

void ImageOverlayer::OverlayGTRobotPose(double x, double y, double z,
                                        CvScalar color, IplImage* baseImage,
                                        int sequenceNo)
{
  vector<CvPoint3D32f> Robot_PositionsToReProject;
  Robot_PositionsToReProject.resize(totPntsPerPosGT);

  CvMat _Robot_PositionsToReProject =
      cvMat(1, totPntsPerPosGT, CV_32FC3, &Robot_PositionsToReProject[0]);

  Robot_PositionsToReProject[0] = cvPoint3D32f(x, y, robHeight);
  for (
      int pts = 0; pts < circlePointsPerPosition;
      pts++) // circlePointsPerPosition points out of totPntsPerPosGT for circle
  {
    float theta = -M_PI + (float)pts * 2 * M_PI / (circlePointsPerPosition);
    Robot_PositionsToReProject[1 + pts] = cvPoint3D32f(
        x + robRadius * cosf(theta), y + robRadius * sinf(theta), robHeight);
  }

  // orientation GT is not available for now hence this part is commented out.
  //     for(int pts = 0; pts < arrowPointsPerPosition; pts++)
  //     //arrowPointsPerPosition points out of totPntsPerPosGT for th arrow
  //     {
  //       Robot_PositionsToReProject[1 + circlePointsPerPosition + pts] =
  //       cvPoint3D32f( x +
  //       (float)pts*(robRadius/(float)arrowPointsPerPosition)*cosf(thetaRob),
  //       y +
  //       (float)pts*(robRadius/(float)arrowPointsPerPosition)*sinf(thetaRob),
  //       robHeight);
  //     }
  vector<CvPoint2D32f> reprojectedPoints_Robot;
  reprojectedPoints_Robot.resize(totPntsPerPosGT);

  CvMat _imageReprojectedPoints_RobotRight =
      cvMat(1, totPntsPerPosGT, CV_32FC2, &reprojectedPoints_Robot[0]);

  cvProjectPoints2(&_Robot_PositionsToReProject, Rvec_right_n, Tvec_right, &_M2,
                   &_D2, &_imageReprojectedPoints_RobotRight, NULL, NULL, NULL,
                   NULL, NULL);

  // GT for all robots (and the ball) is painted black.
  for (int pts = 0; pts < totPntsPerPosGT; pts++)
  {
    CvPoint robot_PointToBeShownRight =
        cvPoint(reprojectedPoints_Robot[pts].x, reprojectedPoints_Robot[pts].y);
    cvCircle(baseImage, robot_PointToBeShownRight, 0, cvScalar(0.0, 0.0, 0.0),
             2, 8, 0);
  }

  // Put the imgage sequenceNo on top of the image;
  CvFont fontLegend;
  double hScaleLegend = 1.0;
  double vScaleLegend = 1.0;
  int lineWidthLegend = 2;
  cvInitFont(&fontLegend, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_NORMAL,
             hScaleLegend, vScaleLegend, 2, lineWidthLegend);
  char seqNo[50];
  sprintf(seqNo, "Image= %d", sequenceNo);
  cvPutText(baseImage, seqNo, cvPoint(20, 40), &fontLegend,
            cvScalar(24.0, 30.0, 56.0));
}

void ImageOverlayer::gtDataCallback(
    const read_omni_dataset::LRMGTData::ConstPtr& msg, IplImage* img_)
{

  // unpack the message into local variables

  string imageFile = msg->RightFilename;

  omniGTPose[0] = msg->poseOMNI1;
  omniGTPose[2] = msg->poseOMNI3;
  omniGTPose[3] = msg->poseOMNI4;
  omniGTPose[4] = msg->poseOMNI5;
  targetGTPosition[0] = msg->orangeBall3DGTposition;

  foundOMNI_GT[0] = msg->foundOMNI1;
  foundOMNI_GT[2] = msg->foundOMNI3;
  foundOMNI_GT[3] = msg->foundOMNI4;
  foundOMNI_GT[4] = msg->foundOMNI5;
  // ROS_INFO("I heard: %s", imageFile.c_str());

  char imagePathName[100];
  strcpy(imagePathName, imagePathNameBaseName);
  strcat(imagePathName, imageFile.c_str());

  img_ = cvLoadImage(imagePathName);

  ///@INFO The robot GT positions are basically the positions of the colored
  /// marker's center placed on the robot. This does not exactly match the
  /// robot's center of mass. There is a fixed translational shift, obtained as
  /// follows. The robot is manually placed at a fixed coordinate on the ground
  /// using a plummet hanging from the CM and touching the point. Then, the GT
  /// pose of the center of the marker is obtained (which obviously did not
  /// match
  /// the position of the fixed coordinate on which the robot was placed).
  /// Obtaining the translation is then a piece of cake!
  double shift_a = 0.0, shift_b = 0.0;

  // Overlay robot poses
  for (int i = 0; i < NUM_ROBOTS; i++)
  {
    getActualShift(i, &shift_a, &shift_b);
    if (robotActive[i] || foundOMNI_GT[i])
    {
      double thetaRob =
          2 * asin(robot_state[i].pose.pose.orientation.z); // the robot is on
      // 2d plane so only
      // YAW
      double tempPosX = robot_state[i].pose.pose.position.x;
      double tempPosY = robot_state[i].pose.pose.position.y;
      double tempPosTheta = thetaRob;

      double realEstimatedX = tempPosX + shift_a * cosf(tempPosTheta) -
                              shift_b * sinf(tempPosTheta);
      double realEstimatedY = tempPosY + shift_a * sinf(tempPosTheta) +
                              shift_b * cosf(tempPosTheta);

      OverlayEstimatedRobotPose(realEstimatedX, realEstimatedY,
                                robot_state[i].pose.pose.position.z, thetaRob,
                                color_est[i], img_);

      // Finding the pixels a few centimeter above the robot's estimated
      // position to overlay the robot name "OMNI 1", or so on.
      vector<CvPoint3D32f> OMNI_status;
      OMNI_status.resize(1);

      CvMat _OMNI_status = cvMat(1, 1, CV_32FC3, &OMNI_status[0]);
      OMNI_status[0] = cvPoint3D32f(realEstimatedX, realEstimatedY, 1.00);

      vector<CvPoint2D32f> reprojectedOMNI_status;
      reprojectedOMNI_status.resize(1);

      CvMat _reprojectedOMNI_status =
          cvMat(1, 1, CV_32FC2, &reprojectedOMNI_status[0]);
      cvProjectPoints2(&_OMNI_status, Rvec_right_n, Tvec_right, &_M2, &_D2,
                       &_reprojectedOMNI_status, NULL, NULL, NULL, NULL, NULL);

      CvFont font;
      double hScale = 1.0;
      double vScale = 1.0;
      int lineWidth = 1;
      cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_NORMAL, hScale,
                 vScale, 2, lineWidth);

      double errorX = fabs(omniGTPose[i].pose.position.x - realEstimatedX);
      double errorY = fabs(omniGTPose[i].pose.position.y - realEstimatedY);
      double error_ecldn = pow(errorX * errorX + errorY * errorY, 0.5);

      ///@INFO For each robot below, the error gets published. However, as the
      /// GT positions are not present for many time chunks due to occlusions
      /// from the GT cameras , we manually/visually inspected each GT image and
      /// the positions and noted the image intervas where the GT is absent.
      /// Hence, the magic numbers below!
      if (i == 0)
      {
        std_msgs::Float32 error_msg;
        // error_msg.header.stamp = msg->header.stamp;
        error_msg.data = error_ecldn;

        if ((msg->header.seq < 1570) ||
            (msg->header.seq >= 1607 && msg->header.seq <= 2870) ||
            (msg->header.seq >= 3336 && msg->header.seq <= 3800) ||
            (msg->header.seq >= 3816 && msg->header.seq <= 5030) ||
            (msg->header.seq >= 5197 && msg->header.seq <= 5870))
        {
          OverlayGTRobotPose(omniGTPose[i].pose.position.x,
                             omniGTPose[i].pose.position.y,
                             omniGTPose[i].pose.position.z,
                             cvScalar(0.0, 0.0, 0.0), img_, msg->header.seq);

          errorPublisherOMNI1.publish(error_msg);
        }

        cvPutText(img_, "OMNI 3", cvPoint(reprojectedOMNI_status[0].x - 70,
                                          reprojectedOMNI_status[0].y),
                  &font, cvScalar(0.0, 50.0, 255.0));
      }
      if (i == 2)
      {
        std_msgs::Float32 error_msg;
        // error_msg.header.stamp = msg->header.stamp;
        error_msg.data = error_ecldn;

        if ((msg->header.seq < 1500) ||
            (msg->header.seq >= 2871 && msg->header.seq <= 2950) ||
            (msg->header.seq >= 3006 && msg->header.seq <= 3750) ||
            (msg->header.seq >= 3797 && msg->header.seq <= 4260) ||
            (msg->header.seq >= 5442 && msg->header.seq <= 5540) ||
            (msg->header.seq > 5567))
        {
          OverlayGTRobotPose(omniGTPose[i].pose.position.x,
                             omniGTPose[i].pose.position.y,
                             omniGTPose[i].pose.position.z,
                             cvScalar(0.0, 0.0, 0.0), img_, msg->header.seq);

          errorPublisherOMNI3.publish(error_msg);
        }

        cvPutText(img_, "OMNI 2", cvPoint(reprojectedOMNI_status[0].x - 70,
                                          reprojectedOMNI_status[0].y),
                  &font, cvScalar(19.0, 69.0, 139.0));
      }
      if (i == 3)
      {
        std_msgs::Float32 error_msg;
        // error_msg.header.stamp = msg->header.stamp;
        error_msg.data = error_ecldn;

        if ((msg->header.seq < 555) ||
            (msg->header.seq >= 564 && msg->header.seq <= 1080) ||
            (msg->header.seq >= 2314 && msg->header.seq <= 3724) ||
            (msg->header.seq > 4526))
        {
          OverlayGTRobotPose(omniGTPose[i].pose.position.x,
                             omniGTPose[i].pose.position.y,
                             omniGTPose[i].pose.position.z,
                             cvScalar(0.0, 0.0, 0.0), img_, msg->header.seq);

          errorPublisherOMNI4.publish(error_msg);
        }

        cvPutText(img_, "OMNI 1", cvPoint(reprojectedOMNI_status[0].x - 70,
                                          reprojectedOMNI_status[0].y),
                  &font, cvScalar(147.0, 20.0, 255.0));
      }
      if (i == 4)
      {
        std_msgs::Float32 error_msg;
        // error_msg.header.stamp = msg->header.stamp;
        error_msg.data = error_ecldn;

        if ((msg->header.seq < 530) ||
            (msg->header.seq >= 1276 && msg->header.seq <= 2260) ||
            (msg->header.seq >= 3069 && msg->header.seq <= 3273) ||
            (msg->header.seq >= 3300 && msg->header.seq <= 3800) ||
            (msg->header.seq >= 4335 && msg->header.seq <= 5173) ||
            (msg->header.seq >= 5191 && msg->header.seq <= 5240))
        {
          OverlayGTRobotPose(omniGTPose[i].pose.position.x,
                             omniGTPose[i].pose.position.y,
                             omniGTPose[i].pose.position.z,
                             cvScalar(0.0, 0.0, 0.0), img_, msg->header.seq);

          errorPublisherOMNI5.publish(error_msg);
        }

        cvPutText(img_, "Robot 4", cvPoint(reprojectedOMNI_status[0].x - 70,
                                           reprojectedOMNI_status[0].y),
                  &font, cvScalar(255.0, 50.0, 0.0));
      }
    }
  }

  // Overlaying Targets
  for (int i = 0; i < NUM_TARGETS; i++)
  {
    if (targetActive[i] || 1) // remove this default true condition if you do
    // not want to display the ball GT if you are not
    // obtaining the estimated ball position and/or
    // not interested
    {
      OverlayEstimatedTargetPosition(target_state[i].pose.pose.position.x,
                                     target_state[i].pose.pose.position.y,
                                     target_state[i].pose.pose.position.z,
                                     color_est[NUM_ROBOTS + i], img_);

      // The if is to simply remove the GT poses of the ball that are 0,0 (by
      // default 0,0 is uknown GT position!)
      if (!(targetGTPosition[0].x == 0 && targetGTPosition[0].y == 0))
        OverlayGTTargetPosition(targetGTPosition[0].x, targetGTPosition[0].y,
                                targetGTPosition[0].z, cvScalar(0.0, 0.0, 0.0),
                                img_);
    }
  }

  bool sendTargetError = true;
  static double maxTimeUntilDisable = 1.0; // seconds

  if (!foundBallOMNI)
  {
    ros::Time now = msg->header.stamp;
    double timeSinceSeen = (now - timeBallNotFound).toNSec() * 1e-9;
    ROS_DEBUG("Ball not seen by any robot for %.2fs", timeSinceSeen);

    if (timeSinceSeen > maxTimeUntilDisable)
    {
      ROS_WARN("Disabling in this iteration since ball has not been seen for "
               "more than %.2fs",
               maxTimeUntilDisable);
      sendTargetError = false;
    }
  }

  if (sendTargetError)
  {
    double errX =
        fabs(targetGTPosition[0].x - target_state[0].pose.pose.position.x);
    double errY =
        fabs(targetGTPosition[0].y - target_state[0].pose.pose.position.y);
    double errZ =
        fabs(targetGTPosition[0].z - target_state[0].pose.pose.position.z);

    std_msgs::Float32 error_target;
    error_target.data = sqrt(errX * errX + errY * errY + errZ * errZ);
    orangeBallError.publish(error_target);
  }

  cvShowImage(
      "Overlaid Estimates on the Grount Truth Images of Right GT Camera", img_);
  cvReleaseImage(&img_);
}

void ImageOverlayer::omniCallback(
    const read_omni_dataset::RobotState::ConstPtr& msg, IplImage* img_)
{
  // Basically switch on all robots, except OMNI2, by default. Then the
  // estimated state for each other is obtained from the corresponding rostopic
  for (int i = 0; i < 5; i++)
  {
    robotActive[i] = true;
    robot_state[i] = msg->robotPose[i];
  }
  robotActive[1] = false; // Recall that OMNI2 is absent
}

void ImageOverlayer::target1Callback(
    const read_omni_dataset::BallData::ConstPtr& msg, IplImage* img_)
{
  // estimated target state obtained from the corresponding rostopic
  targetActive[0] = true;
  target_state[0].pose.pose.position.x = msg->x;
  target_state[0].pose.pose.position.y = msg->y;
  target_state[0].pose.pose.position.z = msg->z;

  // record time since ball seen
  if (msg->found)
  {
    foundBallOMNI = true;
    return;
  }

  else
  {
    // ball not seen. if previously not seen, don't do anything. But if this is
    // the first time not seen, we must record this time
    if (true == foundBallOMNI)
      timeBallNotFound = msg->header.stamp;
  }

  foundBallOMNI = msg->found;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "evaluate_overlay_omni_dataset");

  if (argc != 2)
  {
    ROS_WARN("WARNING: you should specify images folder path (on which GT and "
             "estimated poses/positions will be overlaid)! which might be "
             "%s\n, respectively",
             argv[1]);
  }
  else
  {
    printf("INFO: you have set camera images folder path: %s\n", argv[1]);
  }

  cvStartWindowThread();

  ImageOverlayer node(argv[1]);
  ros::spin();
  return 0;
}
