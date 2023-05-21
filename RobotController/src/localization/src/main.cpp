#include "iostream"
#include <opencv2/core/mat.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "RobotVision/CameraCalibration/camera_calibration.h"
#include "RobotVision/aruco_scanner.h"
#include "RobotVision/Map/map.h"
#include "RobotVision/Map/map_creator.h"
#include "RobotVision/Map/map_viewer.h"
#include "RobotVision/Map/a_star.cpp"
#include "RobotVision/aruco_gnerator.h"
#include "RobotVision/opencv_constants.h"
#include "Loclization/localization.h"
#include "RobotVision/CameraCalibration/pose_collection.h"
#include <ros/ros.h>

void captureImages()
{
  ArucoScanner as = ArucoScanner();
  std::function<void(cv::Mat &)> captureImagesProccess = [=](cv::Mat &frame)
  {
    int key = cv::waitKey(1);
    if (key == 32)
    {
      std::ostringstream filename;
      filename << constants::calibrationImagesFolder << "image_" << time(0) << ".jpg";
      cv::imwrite(filename.str(), frame);
    }
  };
  as.openCamera(captureImagesProccess);
}

void Odometer_Updatei(const tf2_msgs::TFMessage::ConstPtr &msg)
{
  double odom_x = msg->transforms[0].transform.translation.x;
  double odom_y = msg->transforms[0].transform.translation.y;
  std::cout << odom_x << "\n";

  // std::cout << "TEST\n";
}
// int main(int argc, char **argv)
// {

//   // Localization loc = Localization();
//   // loc.start(false);
//   ros::init(argc, argv, "localization");
//   ros::NodeHandle n;
//   // this->publisher = n.advertise<geometry_msgs::Pose2D>("Location", 10);
//   // ros::Rate rate(10);
//   ros::Subscriber s = n.subscribe("/tf", 1, Odometer_Updatei);
//   while(1){
//     ros::spinOnce();
//     // rate.sleep();
//   }
//   return 0;
// }

int main(int argc, char **argv)
{
  Localization loc = Localization();
  loc.start(true);
  return 0;
}
