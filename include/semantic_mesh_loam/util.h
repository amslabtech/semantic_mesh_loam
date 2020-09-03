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
  using Time = ros::Time;
  
  struct color_data{
	  int r;
	  int g;
	  int b;
  };

}

#endif
