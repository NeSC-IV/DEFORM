#include <ros/ros.h>
#include "plan_manager/plan_manager.h"

int main(int argc,char* argv[])
{
    ros::init(argc, argv, "formation_manager_node");

    PlanManager pm;

    ros::spin();
    return 0;
}
