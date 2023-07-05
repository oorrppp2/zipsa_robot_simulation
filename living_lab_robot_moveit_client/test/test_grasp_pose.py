#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from pose_msg.msg import Result
import numpy as np

def main():
    result_pose_pub = rospy.Publisher('/contact_graspnet/results', Result, queue_size=1)
    # pose = np.asarray([-0.02850431 , 0.9983714  , 0.0494163  ,-0.23437642,
    #                     -0.45060104, -0.05696213,  0.8909063 , -0.10014199,
    #                      0.89227027,  0.00312764,  0.45149088,  0.6862629 ,
    #                      0.        ,  0.        ,  0.        ,  1.        ])
    # pose = np.asarray([ 0.09441529,  0.99262965,  0.07597552, -0.1223664,
    #         -0.38136083 ,-0.03443251 , 0.92378485 ,-0.09528625,
    #          0.9195921  ,-0.1161935  , 0.37529913 , 0.7968892 ,
    #          0.         , 0.         , 0.         , 1.        ])
    
    pose = np.asarray([ 0.93857527, -0.27467564, -0.20887704, -0.08091207,
                        0.32525826,  0.9063651 ,  0.26964647,  0.03462505,
                        0.11525353, -0.32102248,  0.94003254,  0.69954574,
                        0.        ,  0.        ,  0.        ,  1.        ])

    
    rospy.sleep(0.5)
    result_pose_pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('test_grasp_pose', anonymous=False)
    main()