/*
 * BagfileParser.h
 *
 *  Created on: 08.03.2016
 *      Author: amndan
 */

#ifndef SRC_BAGFILEPARSER_H_
#define SRC_BAGFILEPARSER_H_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <vector>
#include <string>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

class BagfileParser
{
public:
  BagfileParser(std::string filename);
  virtual ~BagfileParser();
  std::vector<geometry_msgs::PoseStamped> parsePoseStamped(std::string topic);
private:
  rosbag::Bag _bagfile;
};

#endif /* SRC_BAGFILEPARSER_H_ */
