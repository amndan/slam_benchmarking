/*
 * BagfileParser.cpp
 *
 *  Created on: 08.03.2016
 *      Author: amndan
 */

#include "BagfileParser.h"

BagfileParser::BagfileParser(std::string filename)
{
  try
  {
    _bagfile.open(filename, rosbag::bagmode::Read);
  }
  catch(...)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> cannot open bagfile -> exit" << std::endl;
    exit(1);
  }
  
  std::cout << __PRETTY_FUNCTION__ << "--> opended bagfile with size: " << _bagfile.getSize() << std::endl;
}

BagfileParser::~BagfileParser()
{
  _bagfile.close();
}


