/**
 * @file   ScanInverterNode.h
 * @author evocortex (info@evocortex.com) - HEN, MMA, MPL
 *
 * @brief Contains the declaration of the ScanInverterNode class.
 *
 * @date 2019-10-30
 *
 * @copyright MIT
 */

#pragma once

#include <string>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "tf2_ros/static_transform_broadcaster.h"


namespace evo {

/**
 * @brief Inverts laser scans in order to compensate an updside down installation of the
 *        scanner.
 */
class ScanInverterNode
{
 public:
   /// Constructor.
   ScanInverterNode();


   // Start running.
   void run();


 private:
   ros::NodeHandle _pNh;

   ros::Subscriber _scanSub;
   ros::Publisher  _scanPub;

   std::string _invertedSuffix;

   tf2_ros::StaticTransformBroadcaster _tfBroadcaster;


   void scanCallback(const sensor_msgs::LaserScan& in);
};

} // end namespace evo