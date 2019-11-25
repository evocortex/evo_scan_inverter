/**
 * @file  scan_inverter_node.cpp
 * @brief Contains the implementation of the LaserScanInverter class.
 *
 * @author evocortex (MPL)
 * @date 2019-10-30
 */

#include "laser_scan_inverter.h"

/// Main entry point
int main(int argc, char** argv)
{
   ros::init(argc, argv, "scan_inverter");

   ros::NodeHandle nh("~");

   evo::LaserScanInverter inverter(nh);

   inverter.run();
}