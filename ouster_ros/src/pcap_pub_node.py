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
def nano_time():
    now = rospy.Time.now()
    return now.nsecs + now.secs * 1e9
if __name__ == '__main__':
    rospy.init_node('pcap_pub_node')
    rate = rospy.Rate(10000000)
    meta_path = rospy.get_param("~meta_path")
    pcap_path = rospy.get_param("~pcap_path")
    imu_pub = rospy.Publisher('/os1_node/imu_packets', PacketMsg, queue_size=10)
    lidar_pub = rospy.Publisher('/os1_node/lidar_packets', PacketMsg, queue_size=10)
    with open(meta_path, 'r') as f:
        metadata = client.SensorInfo(f.read())
    source = pcap.Pcap(pcap_path, metadata)
    start_time = None
    start_stamp = None
    timestamps = None
    for packet in source:
        if isinstance(packet, client.LidarPacket):
            timestamps = packet.header(client.ColHeader.TIMESTAMP)[0]
        elif isinstance(packet, client.ImuPacket):
            timestamps = packet.gyro_ts
        
        if start_time:
            #seconds since start from packet
            diff = (timestamps-start_stamp)                 
            #seconds since start of this script
            diff_clock = (nano_time()-start_time)
            #both time should be aligned to simulate realtime playback
            rospy.sleep((diff-diff_clock)/1e9)
        else:
            start_time = nano_time()
            start_stamp = timestamps
        msg = PacketMsg()
        msg.buf = packet._data.tolist()
        if isinstance(packet, client.LidarPacket):
            lidar_pub.publish(msg)
        elif isinstance(packet,client.ImuPacket):
            imu_pub.publish(msg)
    print("completed {} secs of data.".format((nano_time()-start_time)/1e9))
