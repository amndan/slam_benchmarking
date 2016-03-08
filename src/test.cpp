/*
 * test.cpp
 *
 *  Created on: 08.03.2016
 *      Author: amndan
 */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <vector>
#include <string>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include "BagfileParser.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char ** argv)
{

  BagfileParser bagfileParser("/media/amndan/Daten/bags/bosch_ceilCam/bags/uc_kupfer_photog_map/01/2016-02-29-12-54-13_0.bag");
  std::vector<geometry_msgs::PoseStamped> poses = bagfileParser.parsePoseStamped("/pose");
  
  for(int i = 2; i < poses.size(); i++)
  {
    std::cout << poses.at(i).pose.position.x << std::endl;
  }
}


