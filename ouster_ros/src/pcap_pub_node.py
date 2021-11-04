#!/usr/bin/env python3
"""This node iterates thru a pcap file and publishes the content
as PacketMsg.
os_cloud_node must be present in conjunction
"""
import rospy
from ouster import client, pcap
from ouster_ros.msg import PacketMsg
from sensor_msgs.msg import PointCloud2
import numpy as np
if __name__ == '__main__':
    rospy.init_node('pcap_pub_node')
    meta_path = rospy.get_param("~meta_path")
    pcap_path = rospy.get_param("~pcap_path")
    imu_pub = rospy.Publisher('/os1_node/imu_packets', PacketMsg, queue_size=10)
    lidar_pub = rospy.Publisher('/os1_node/lidar_packets', PacketMsg, queue_size=10)
    with open(meta_path, 'r') as f:
        metadata = client.SensorInfo(f.read())
    source = pcap.Pcap(pcap_path, metadata)
    for packet in source:
        if isinstance(packet, client.LidarPacket):
            # Now we can process the LidarPacket. In this case, we access
            # the encoder_counts, timestamps, and ranges
            lidar_msg = PacketMsg()
            lidar_msg.buf =packet._data.tolist()
            lidar_pub.publish(lidar_msg)
        elif isinstance(packet, client.ImuPacket):
            imu_msg = PacketMsg()
            imu_msg.buf = packet._data.tolist()
            imu_pub.publish(imu_msg)
