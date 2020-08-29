#ifndef __UTIL_H
#define __UTIL_H
#include"ros/ros.h"
#include<time.h>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include <chrono>

namespace semloam
{ 
  /** \brief A standard non-ROS alternative to ros::Time.*/
  using Time = std::chrono::system_clock::time_point;

  // helper function
  inline double toSec(Time::duration duration)
  {
    return std::chrono::duration<double>(duration).count();
  };
  inline Time fromROSTime(ros::Time const& rosTime){
	  auto epoch = std::chrono::system_clock::time_point();
	  auto since_epoch = std::chrono::seconds(rosTime.sec) + std::chrono::nanoseconds(rosTime.nsec);
	  return epoch + since_epoch;
  }
  
  inline ros::Time toROSTime(Time const& time_point){
	  return ros::Time().fromNSec(std::chrono::duration_cast<std::chrono::nanoseconds>(time_point.time_since_epoch()).count());
  }

}

#endif
