#!/usr/bin/env python3

import rospy
from ot_process.srv import OtTransmit,OtTransmitResponse

import numpy as np
import ot
import ot.plot

def OtHandle(req):

    tb_num = req.alloc_num
    form1 = np.zeros(tb_num*2).reshape(tb_num, 2)
    form2 = np.zeros(tb_num*2).reshape(tb_num, 2)
    for number in range(tb_num):
        form1[number,0] = req.start_pts[number].x
        form1[number,1] = req.start_pts[number].y
        form2[number,0] = req.goal_pts[number].x
        form2[number,1] = req.goal_pts[number].y

    M = ot.dist(form1, form2)
    a, b = np.ones(tb_num), np.ones(tb_num)
    G0 = ot.emd(a, b, M)

    allo_result = []
    for count in range(G0.size):
        allo_result.append(int(G0[count // tb_num,count % tb_num]))

    return OtTransmitResponse(allo_result)

def run_ot():

    rospy.init_node('ot_srv')
    tb_id = rospy.get_param('~ot_srv/tb_id', 100)
    ot_compute_service = rospy.Service('/tb_' + str(tb_id) + '_ot_callback', OtTransmit, OtHandle)
    rospy.spin()

if __name__ ==  "__main__":
    run_ot()
