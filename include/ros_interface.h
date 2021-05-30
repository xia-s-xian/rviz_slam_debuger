#ifndef __ROS_INTERFACE_H__
#define __ROS_INTERFACE_H__

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>

extern ros::Publisher map_pub;

typedef struct __header
{
    int seq;
    int sec;
    int nsec;
}Header;

#endif

