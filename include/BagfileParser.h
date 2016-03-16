/*
 * BagfileParser.h
 *
 *  Created on: 08.03.2016
 *      Author: amndan
 */

#ifndef SRC_BAGFILEPARSER_H_
#define SRC_BAGFILEPARSER_H_

#include "ros/ros.h"
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
  template <class T> inline std::vector<T> parse(std::string topic);
private:
  rosbag::Bag _bagfile;
};


template <class T> inline std::vector<T> BagfileParser::parse(std::string topic)
{
  std::vector<T> retval;
  rosbag::View view(_bagfile, rosbag::TopicQuery(topic));

  foreach(rosbag::MessageInstance const m, view)
  {
    typename T::ConstPtr val = m.instantiate<T>();

    if (val != NULL)
    {
      retval.push_back(*val);
    }
    else
    {
      std::cout << __PRETTY_FUNCTION__ << "--> WARNING: got NULL pointer" << std::endl;
    }
  }
  return retval;
}

#endif /* SRC_BAGFILEPARSER_H_ */
