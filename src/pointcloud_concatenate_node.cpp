#include "pointcloud_concatenate/pointcloud_concatenate.hpp"

int main(int argc, char** argv) {
  // Create node
  ros::init(argc, argv, "pointcloud_concatenate");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");  // Private nodehandle for parameters
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  // Create object
  ROS_INFO("Setting up class");
  PointcloudConcatenate node(nh, pnh);

  /*                    Periodic spinning with rate               */
  ROS_INFO("Spinning...");
  double hz = node.getHz();
  ros::Rate rate(hz); // Defing the looping rate
  while (ros::ok())
  {
    try {
      node.update();
      ros::spinOnce();
      rate.sleep();
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Error in update: " << e.what());
    }
  }

  return 0;
}