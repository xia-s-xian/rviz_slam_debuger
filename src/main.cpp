
extern "C"
{
    #include <yar.h>
}

#include "ros_interface.h"
#include "yar_server_rpc.h"
#include <iostream>

//add rpc handlers
yar_server_handler handlers[] = {
    {(char*)"map",sizeof("map") - 1,yar_map_handler},
    {NULL,0,NULL}
};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"RviZ_Debuger");
    ros::NodeHandle ros_node_handler;
    map_pub = ros_node_handler.advertise<nav_msgs::OccupancyGrid>("map",1);
    if(map_pub == nullptr)
        exit(1);

    int standalone = 1;
    char* hostname = (char*)"0.0.0.0:3900";
    if(yar_server_init(hostname))
    {
        yar_server_set_opt(YAR_STAND_ALONE,&standalone);
        yar_server_register_handler(handlers);
        yar_server_run();
    }
    return 0;
}
