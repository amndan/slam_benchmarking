/*
 * test.cpp
 *
 *  Created on: 08.03.2016
 *      Author: amndan
 */

// gt   :  x   x    |  |   |   |   x  x
// slam :         |  |  |   |  | |

#include "BagfileParser.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/time.h"
#include <fstream>
#include <string>
#include <cstdio>
#include "tf/transform_datatypes.h"
#include "assert.h"
#include <cmath>
#include "Eigen/Dense"
#include <ctime>

#define DIFFMAX 0.0001
#define MAXDIST 50
#define SAMPLES 90000
#define SIMULATED_ERROR 800
#define SIMULATED_ERROR_TIME 0.001

void checkInput(int argc, char ** argv);
void printUsage();
void parseMsgs();
void checkPath(const std::string& name);
void checkTimestamps(const std::vector<geometry_msgs::PoseStamped>& poses);
void cropMsgs();
void syncMsgs();
void syncMsg(std::vector<geometry_msgs::PoseStamped>::const_iterator gt);
double durToSec(ros::Duration dur);
Eigen::Matrix3d poseToMat(geometry_msgs::PoseStamped pose);
void scaleTrans(Eigen::Matrix3d& trans, double factor);
void evaluate();
void printResult();
void editData(); //testing code

std::string g_pathSlam;
std::string g_pathGt;
std::string g_topicPoseSlam;
std::string g_topicPoseGt;
std::string g_pathRelations;

std::vector<geometry_msgs::PoseStamped> g_posesSlam;
std::vector<geometry_msgs::PoseStamped> g_posesGt;
std::vector<Eigen::Matrix3d> g_transGt;
std::vector<Eigen::Matrix3d> g_transSlam;
std::vector<Eigen::Vector3d> g_errorsAbs;

const ros::Duration g_zero(0.0);

bool g_relationsProvided;

int main(int argc, char ** argv)
{
  std::srand(time(NULL));

  checkInput(argc, argv);
  parseMsgs();

  editData(); // testing
  //parse relations()

  cropMsgs();
  syncMsgs();

  evaluate();
  printResult();
  
}

void cropMsgs()
{
  ros::Time stampSlamBegin = g_posesSlam.begin()->header.stamp;
  ros::Time stampSlamEnd = (g_posesSlam.end()-1)->header.stamp;

  std::vector<geometry_msgs::PoseStamped>::const_iterator it1 = g_posesGt.begin();
  std::vector<geometry_msgs::PoseStamped>::const_iterator it2 = g_posesGt.end()-1;

  while(it1->header.stamp < stampSlamBegin)
  {
    it1++;

    if(it1 == g_posesGt.end()-1)
    {
      std::cout << __PRETTY_FUNCTION__ << "--> timing error 01 --> exit" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  std::cout << it2->header.stamp << " "<<  stampSlamEnd<< std::endl;

  while(it2->header.stamp > stampSlamEnd)
  {
    it2--;

    if(it2 == g_posesGt.begin())
    {
      std::cout << __PRETTY_FUNCTION__ << "--> timing error 02 --> exit" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  if(it1 == g_posesGt.begin() && it2 == g_posesGt.end()-1)
  {
    std::cout << "no messages croped; timing OK" << std::endl;
  }
  else
  {
    std::cout << "WARNING Messages Croped" << std::endl; // TODO: better output here
    std::cout << "leading cropped messages: " << std::distance((std::vector<geometry_msgs::PoseStamped>::const_iterator) g_posesGt.begin(), it1) << std::endl;
    std::cout << "trailing cropped messages: " << std::distance(it2, (std::vector<geometry_msgs::PoseStamped>::const_iterator) g_posesGt.end()-1) << std::endl;

    std::vector<geometry_msgs::PoseStamped> tmp(it1, it2); // crop vector
    g_posesGt = tmp;
  }
}

void checkInput(int argc, char ** argv)
{
  std::cout << "Argument List:" << std::endl;
  for(int i = 0; i < argc; i++)
  {
    std::cout << argv[i] << std::endl;
  }

  switch(argc)
  {
  case 5:
    g_pathSlam = argv[1];
    g_topicPoseSlam = argv[2];
    g_pathGt = argv[3];
    g_topicPoseGt = argv[4];
    g_relationsProvided = false;

    checkPath(g_pathSlam);
    checkPath(g_pathGt);

    return;
  case 6:
    g_pathSlam = argv[1];
    g_topicPoseSlam = argv[2];
    g_pathGt = argv[3];
    g_topicPoseGt = argv[4];
    g_pathRelations = argv[5];
    g_relationsProvided = true;

    checkPath(g_pathSlam);
    checkPath(g_pathGt);
    checkPath(g_pathRelations);

    return;
  default:
    printUsage();
    exit(EXIT_FAILURE);
  }
}

void parseMsgs()
{
  BagfileParser bagfileParserSlam(g_pathSlam);
  g_posesSlam = bagfileParserSlam.parse<geometry_msgs::PoseStamped>(g_topicPoseSlam);
  checkTimestamps(g_posesSlam);
  std::cout << "parsed messages slam: " << g_posesSlam.size() << std::endl;

  BagfileParser bagfileParserGt(g_pathGt);
  g_posesGt = bagfileParserSlam.parse<geometry_msgs::PoseStamped>(g_topicPoseGt);
  checkTimestamps(g_posesGt);
  std::cout << "parsed messages ground truth " << g_posesGt.size() << std::endl;
}

void checkPath(const std::string& name)
{
  if(FILE *file = fopen(name.c_str(), "r"))
  {
    fclose(file);
  }
  else
  {
    std::cout << __PRETTY_FUNCTION__ << "--> " << name << "--> no valid file name" << std::endl;
    exit(EXIT_FAILURE);
  }
}

void printUsage()
{
  std::cout << "usage: ./evaluator bag1.bag posetopic bag2.bag posetopic relations.txt" << std::endl;
}

void checkTimestamps(const std::vector<geometry_msgs::PoseStamped>& poses)
{
  ros::Time lastStamp;
  lastStamp = poses.at(0).header.stamp;

  for(int i = 1; i < poses.size(); i++)
  {
    if(poses.at(i).header.stamp - lastStamp > g_zero)
    {
      lastStamp = poses.at(i).header.stamp;
    }
    else
    {
      std::cout << __PRETTY_FUNCTION__ << "--> timing error --> exit" << std::endl;
      exit(EXIT_FAILURE);
    }
  }
}

void syncMsgs()
{
  ros::Duration diff;
  ros::Duration diffmax(std::abs(DIFFMAX));
  bool foundStamp = false;


  for(std::vector<geometry_msgs::PoseStamped>::const_iterator gt = g_posesGt.begin(); gt != g_posesGt.end(); ++gt)
  {
    for(std::vector<geometry_msgs::PoseStamped>::const_iterator slam = g_posesSlam.begin(); slam != g_posesSlam.end(); ++slam)
    {
      foundStamp = false;

      diff = slam->header.stamp - gt->header.stamp;

      if(diff < g_zero) diff = -diff;

      if(diff <= diffmax)
      {
        g_transSlam.push_back(poseToMat(*slam));
        g_transGt.push_back(poseToMat(*gt));
        foundStamp = true;
        break;
      }
    }

    if(foundStamp == false) syncMsg(gt);
  }

  assert(g_transGt.size() == g_transSlam.size());

}

void syncMsg(std::vector<geometry_msgs::PoseStamped>::const_iterator gt)
{
  ros::Time stampGt = gt->header.stamp;

  std::vector<geometry_msgs::PoseStamped>::const_iterator it1 = g_posesSlam.end()-1;
  std::vector<geometry_msgs::PoseStamped>::const_iterator it2 = g_posesSlam.begin();

  while(it2->header.stamp < stampGt)
  {
    it2++;

    if(it2 == g_posesGt.end()-1)
    {
      std::cout << __PRETTY_FUNCTION__ << "--> timing error --> exit" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  while(it1->header.stamp > stampGt)
  {
    it1--;

    if(it1 == g_posesGt.begin())
    {
      std::cout << __PRETTY_FUNCTION__ << "--> timing error --> exit" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  assert(std::distance(it1,it2) == 1); // debug

  ros::Duration diffSlam;
  diffSlam = it2->header.stamp - it1->header.stamp;
  assert(diffSlam > g_zero);

  ros::Duration diffGt;
  diffGt = gt->header.stamp - it1->header.stamp;
  assert(diffGt > g_zero);

  assert(diffSlam > diffGt);

  double linFactor = durToSec(diffGt) / durToSec(diffSlam);

  assert(linFactor > 0.0 && linFactor < 1.0);

  Eigen::Matrix3d tfGt = poseToMat(*gt);
  Eigen::Matrix3d tfSlam1 = poseToMat(*it1);
  Eigen::Matrix3d tfSlam2 = poseToMat(*it2);
  Eigen::Matrix3d tfSlam12 = tfSlam1.inverse() * tfSlam2;

  scaleTrans(tfSlam12, linFactor);

  Eigen::Matrix3d tfSlam = tfSlam1 * tfSlam12;

  g_transSlam.push_back(tfSlam);
  g_transGt.push_back(tfGt);
}

double durToSec(ros::Duration dur)
{
  return dur.sec + dur.nsec * 1E-9;
}

Eigen::Matrix3d poseToMat(geometry_msgs::PoseStamped pose)
{
  Eigen::Matrix3d ret;
  ret.setIdentity();

  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double theta = tf::getYaw(pose.pose.orientation);

  assert(theta >= -M_PI && theta <= M_PI);

  ret(0, 0) = std::cos(theta) + 0.0;
  ret(0, 1) = -std::sin(theta) + 0.0;
  ret(0, 2) = x + 0.0;
  ret(1, 0) = std::sin(theta) + 0.0;
  ret(1, 1) = std::cos(theta) + 0.0;
  ret(1, 2) = y + 0.0;

  return ret;
}

void scaleTrans(Eigen::Matrix3d& trans, double factor)
{
  trans(0,2) *= factor;
  trans(1,2) *= factor;

  double theta = std::asin(trans(1,0));

  theta *= factor;

  trans(0, 0) = std::cos(theta) + 0.0;
  trans(0, 1) = -std::sin(theta) + 0.0;
  trans(1, 0) = std::sin(theta) + 0.0;
  trans(1, 1) = std::cos(theta) + 0.0;

  return;
}

void evaluate()
{
  std::srand(time(NULL));

  for(long int i = 0; i < SAMPLES; i++)
  {
    unsigned int indexRand1 = std::rand() % g_transGt.size();
    unsigned int indexRand2 = std::rand() % g_transGt.size();
    // TODO: Check for multible index using

    Eigen::Matrix3d gt1 = g_transGt.at(indexRand1);
    Eigen::Matrix3d gt2 = g_transGt.at(indexRand2);
    Eigen::Matrix3d slam1 = g_transSlam.at(indexRand1);
    Eigen::Matrix3d slam2 = g_transSlam.at(indexRand2);

    Eigen::Matrix3d g = gt1.inverse() * gt2;      // ground truth
    Eigen::Matrix3d s = slam1.inverse() * slam2;  // slam
    Eigen::Matrix3d d = g.inverse() * s;          // diff

    Eigen::Vector3d error;
    error(0) = std::abs(d(0, 2));
    error(1) = std::abs(d(1, 2));
    error(2) = std::abs(std::asin(d(1,0)));

    g_errorsAbs.push_back(error);
  }
}

void printResult()
{
  std::cout << "size of errors: " << g_errorsAbs.size() << std::endl;

  double meanTrans = 0.0;
  double meanPhi = 0.0;
  double varTrans = 0.0;
  double varPhi = 0.0;
  double maxTrans = 0.0;
  double maxPhi = 0.0;

  for(int i = 0; i < g_errorsAbs.size(); i++)
  {
    double trans = std::sqrt(std::pow(g_errorsAbs.at(i).x(),2) + std::pow(g_errorsAbs.at(i).y(),2));
    double phi = std::abs(std::asin(g_errorsAbs.at(i).z()));

    meanTrans += trans / g_errorsAbs.size();
    meanPhi += phi / g_errorsAbs.size();

    maxPhi = std::max(maxPhi, std::abs(phi));
    maxTrans = std::max(maxTrans, std::abs(trans));
  }

  std::cout <<
      "mean trans: " << meanTrans <<
      " mean phi: " << meanPhi <<
      " max trans: " << maxTrans <<
      " maxPhi: " << maxPhi << std::endl;
}

void editData()
{
  for(int i = 0; i < g_posesSlam.size(); i++)
  {
    if (SIMULATED_ERROR != 0)
    {
      g_posesSlam.at(i).pose.position.x += (unsigned int) (std::rand() % ((2 * SIMULATED_ERROR) - SIMULATED_ERROR));
      g_posesSlam.at(i).pose.position.y += (unsigned int) (std::rand() % ((2 * SIMULATED_ERROR) - SIMULATED_ERROR));
      ros::Duration dur(0);
      g_posesSlam.at(i).header.stamp = g_posesSlam.at(i).header.stamp + dur;
    }
  }
}

