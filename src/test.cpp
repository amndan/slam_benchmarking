/*
 * test.cpp
 *
 *  Created on: 08.03.2016
 *      Author: amndan
 */

#include "BagfileParser.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

int main(int argc, char ** argv)
{
  BagfileParser bagfileParser("/media/amndan/Daten/bags/bosch_ceilCam/bags/uc_kupfer_photog_map/01/2016-02-29-12-54-13_0.bag");

  int i = 0;



  std::vector<geometry_msgs::PoseStamped> poses = bagfileParser.parse<geometry_msgs::PoseStamped>("/pose");
  
  for(i = 0; i < poses.size(); i++)
  {
    //std::cout << poses.at(i).pose.position.x << std::endl;
  }

  std::cout << i << std::endl;




  std::vector<nav_msgs::Odometry> odom = bagfileParser.parse<nav_msgs::Odometry>("/wheelodom");

  for(i = 0; i < odom.size(); i++)
  {
    //std::cout << odom.at(i).pose.pose.position.x << std::endl;
  }

  std::cout << i << std::endl;
}


