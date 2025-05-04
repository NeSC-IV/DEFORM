
#include "ot_process/ot_client.h"

void OT_Transfer::Init(ros::NodeHandle &nh)
{
    //Init
    nh.param("ot_client/tb_num", tb_num, -1);
    nh.param("ot_client/tb_id", tb_id, -1);

    ot_client = nh.serviceClient<ot_process::OtTransmit>(std::string("/tb_") + std::to_string(tb_id) + std::string("_ot_callback"));
}

int OT_Transfer::compute_assignment(
        std::vector<geometry_msgs::Point>& spts_msg,
        std::vector<geometry_msgs::Point>& gpts_msg,
        std::vector<int>& assigned_result)
{
    if ((spts_msg.size() != tb_num) || (gpts_msg.size() != tb_num)) {
        assigned_result = std::vector<int>(tb_num, -1);
        return 1;
    }

    if (tb_num == -1) {
        assigned_result = std::vector<int>(tb_num, -1);
        return 1;
    }

    assigned_result.resize(tb_num);

    ot_process::OtTransmit ot_srv;
    ot_srv.request.alloc_num = static_cast<short>(tb_num);
    ot_srv.request.start_pts = spts_msg;
    ot_srv.request.goal_pts = gpts_msg;

    if (ot_client.call(ot_srv)) {
        for (int i = 0; i < ot_srv.response.alloc_result.size(); i++)
            if(ot_srv.response.alloc_result[i] == 1) {
                assigned_result[i / tb_num] = i % tb_num;
            }

        return 0;
    } else {
        ROS_ERROR("[OT_Transfer]:Check if ot_srv is opened! LINE:65") ;
        assigned_result = std::vector<int>(4, -2);
        return 2;
    }
}



