/**
 * @file   ScanInverterNode.h
 * @author evocortex (info@evocortex.com) - HEN, MMA, MPL
 *
 * @brief Contains the definition of the ScanInverterNode class.
 *
 * @date 2019-10-30
 *
 * @copyright MIT
 */

#include "ScanInverterNode.h"

#include <algorithm>

#include "geometry_msgs/TransformStamped.h"


namespace evo {

ScanInverterNode::ScanInverterNode()
    : _pNh("~")
    , _invertedSuffix("_inverted")   
{
    std::string inTopic("/scan_raw");
    std::string outTopic("/scan");

    ROS_INFO_STREAM("EVO SCAN INVERTER");

    _pNh.param<std::string>("in_topic",  inTopic,  inTopic);
    _pNh.param<std::string>("out_topic", outTopic, outTopic);

    ROS_INFO_STREAM("[EVO SCAN INVERTER] Inverting laser scans from "
                                         << inTopic  << " to " 
                                         << outTopic << ".");

    bool invertScannerFrame = false;

    _pNh.param<bool>("invert_scanner_frame", invertScannerFrame, invertScannerFrame);

    if (invertScannerFrame)
    {
        _pNh.param<std::string>("inverted_frame_suffix", _invertedSuffix, _invertedSuffix);
    }
    else 
    {
        _invertedSuffix.clear();
    }

    _scanPub = _pNh.advertise<sensor_msgs::LaserScan>(outTopic, 5);
    _scanSub = _pNh.subscribe(inTopic, 5, &ScanInverterNode::scanCallback, this); 
}


void ScanInverterNode::run()
{ 
    ros::spin(); 
}


void ScanInverterNode::scanCallback(const sensor_msgs::LaserScan& in)
{
    static std::string scannerFrame = in.header.frame_id;

    // If required, broadcast inverted laser scanner frame
    if (!_invertedSuffix.empty())
    {
        scannerFrame.append(_invertedSuffix);
        _invertedSuffix.clear();

        geometry_msgs::TransformStamped tInv;

        tInv.header.frame_id = in.header.frame_id;
        tInv.header.stamp    = in.header.stamp;
        tInv.child_frame_id  = scannerFrame;

        tInv.transform.translation.x = 0.;
        tInv.transform.translation.y = 0.;
        tInv.transform.translation.z = 0.;

        tInv.transform.rotation.x = 1.;
        tInv.transform.rotation.y = 0.;
        tInv.transform.rotation.z = 0.;
        tInv.transform.rotation.w = 0.;

        _tfBroadcaster.sendTransform(tInv);

        ROS_INFO_STREAM("[EVO SCAN INVERTER] Broadcasting inverted laser scanner frame " << scannerFrame << " via tf.");
    }

    // Invert scan
    sensor_msgs::LaserScan out(in);

    out.header.stamp = in.header.stamp + ros::Duration((in.ranges.size() - 1) * in.time_increment);

    out.angle_min      = -in.angle_max;
    out.angle_max      = -in.angle_min;
    out.time_increment = -in.time_increment;

    std::reverse(std::begin(out.ranges), std::end(out.ranges));

    // Also publish inverted sensor frame if needed (double inversion? MPL)
    out.header.frame_id = scannerFrame;
    
    _scanPub.publish(out);
}

} // end namespace evo