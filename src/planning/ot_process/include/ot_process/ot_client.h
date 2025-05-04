#include <ros/ros.h>
#include "ot_process/OtTransmit.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <eigen3/Eigen/Eigen>

#include <vector>
#include <iostream>
#include <string>

class OT_Transfer
{
public:
    OT_Transfer() = default;
    void Init(ros::NodeHandle &nh);
    int compute_assignment(std::vector<geometry_msgs::Point>& spts_msg,
                           std::vector<geometry_msgs::Point>& gpts_msg,
                           std::vector<int>& assigned_result);

    typedef std::shared_ptr<OT_Transfer> Ptr;
private:

    //Member variables
    int tb_num;
    int tb_id;
    ros::ServiceClient ot_client;
//    ros::NodeHandle node_;
};
