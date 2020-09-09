/**
 * @file   EvoScanInverterMain.cpp
 * @author evocortex (info@evocortex.com) - HEN, MMA, MPL
 *
 * @brief Contains the main entry point of the scan_inverter_node.
 *
 * @date 2019-10-30
 *
 * @copyright MIT
 */

#include "ScanInverterNode.h"


/// Main entry point.
int main(int argc, char** argv)
{
   ros::init(argc, argv, "evo_scan_inverter_node");

   evo::ScanInverterNode inverter;

   inverter.run();
}