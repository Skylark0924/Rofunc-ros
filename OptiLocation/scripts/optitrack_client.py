#!/usr/bin/env python

import rospy
from rofunc_ros.optilocation.linux.client import OptiTrackClient
from rofunc_ros.utils.common import (
    get_param,
    pretty_print_configs,
    is_ip_valid,
    is_port_valid,
)



if __name__ == "__main__":
    try:
        rospy.init_node("roport_optitrack_client", anonymous=True)

        configs = {
            "ip": get_param("ip"),
            "port": get_param("port"),
            "odom_topic": get_param("odom_topic"),
            "pose_topic": get_param("pose_topic"),
            "rate": get_param("rate", 100.0),
            "transform": get_param("base_to_markers_transform"),
        }

        if not is_ip_valid(configs["ip"]) or not is_port_valid(configs["port"]):
            exit(-1)

        pretty_print_configs(configs)
        client = OptiTrackClient(configs)
        rospy.loginfo("RoPort OptiTrack Client ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
