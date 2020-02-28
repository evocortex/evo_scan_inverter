/**
 * @file  laser_scan_inverter.h
 * @brief Contains the declaration of the LaserScanInverter class.
 *
 * @author evocortex (MPL)
 * @date 2019-10-30
 */

#ifndef SCAN_INVERTER_H
#define SCAN_INVERTER_H

#include <string>
#include <algorithm>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "tf/transform_broadcaster.h"

namespace evo {

/// Inverts laser scans in order to compensate an updside down installation of the
/// scanner.
class LaserScanInverter
{
 public:
   /**
    * @brief Contructor.
    *
    * @param nh ROS node handle.
    */
   LaserScanInverter(ros::NodeHandle& nh) :
      _tfPub()   
   {
      std::string inTopic("/scan_raw"), outTopic("/scan");

      ROS_INFO_STREAM_NAMED("RavenScanInverter", "RAVEN SCAN INVERTER");

      nh.param<std::string>("in_topic", inTopic, inTopic);
      nh.param<std::string>("out_topic", outTopic, outTopic);

      ROS_INFO_STREAM_NAMED("RavenScanInverter", "Inverting laser scans from "
                                                     << inTopic << " to " << outTopic
                                                     << ".");

      _scanPub = nh.advertise<sensor_msgs::LaserScan>(outTopic, 5);
      _scanSub = nh.subscribe(inTopic, 5, &LaserScanInverter::scanCallback, this);

      _tf_invert.setOrigin(tf::Vector3(0, 0, 0));
      _tf_invert.setRotation(tf::Quaternion(1, 0, 0, 0));  
   }

   // Start running;
   void run() { ros::spin(); }

 private:
   ros::Subscriber _scanSub;
   ros::Publisher _scanPub;

   tf::TransformBroadcaster _tfPub;
   tf::StampedTransform _tf_invert;

   void scanCallback(const sensor_msgs::LaserScan& in)
   {
      sensor_msgs::LaserScan out(in);

      out.angle_min = -in.angle_max;
      out.angle_max = -in.angle_min;

      std::reverse(std::begin(out.ranges), std::end(out.ranges));

      // also publish inverted sensor frame
      out.header.frame_id = in.header.frame_id + "_inverted";
      
      _tf_invert.frame_id_ = in.header.frame_id;
      _tf_invert.child_frame_id_ = out.header.frame_id;
      _tf_invert.stamp_ = in.header.stamp;
      _tfPub.sendTransform(_tf_invert);
      
      _scanPub.publish(out);
   }
};

} // namespace

#endif /* SCAN_INVERTER_H */
