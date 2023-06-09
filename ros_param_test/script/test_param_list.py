#!/usr/bin/env python3
"""
This script is used to test the parameter list
"""

import rospy

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('test_param_list', anonymous=True)

    # Get parameters
    
    input_topic = rospy.get_param("~input_topics", "/camera1/color/image_raw, /camera2/color/image_raw")
    print('input_topic:',input_topic)

