#!/usr/bin/env python3
"""This node iterates thru a pcap file and publishes the content
as PacketMsg.
os_cloud_node must be present in conjunction
"""
import rospy
import time
from ouster import client, pcap
from ouster_ros.msg import PacketMsg
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
import numpy as np
def nano_time():
    now = rospy.Time.now()
    return now.nsecs + now.secs * 1e9
if __name__ == '__main__':
    rospy.init_node('pcap_pub_node')
    meta_path = rospy.get_param("~meta_path")
    pcap_path = rospy.get_param("~pcap_path")
    imu_pub = rospy.Publisher('/os_node/imu_packets', PacketMsg, queue_size=10000)
    lidar_pub = rospy.Publisher('/os_node/lidar_packets', PacketMsg, queue_size=10000)
    with open(meta_path, 'r') as f:
        metadata = client.SensorInfo(f.read())
    source = pcap.Pcap(pcap_path, metadata)
    start_time = None
    start_stamp = None
    timestamps = None
    cnt = 0 #counter for lidarpackets
    column_count = 0
    for packet in source:
        if isinstance(packet, client.LidarPacket):
            timestamps = packet.timestamp[0]
            column_count = len(packet.timestamp)
        elif isinstance(packet, client.ImuPacket):
            #gyro time in nanoseconds int
            timestamps = packet.gyro_ts
        else:
            print("invalid packet instance")
            continue

        if start_time:
            #seconds since start from packet
            diff = (timestamps-start_stamp)                 
            #seconds since start of this script
            diff_clock = (nano_time()-start_time)
            #both time should be aligned to simulate realtime playback
            sleep_time = (diff-diff_clock)/1e9
            if sleep_time>0:
                rospy.sleep(sleep_time)
        else:
            start_time = nano_time()
            start_stamp = timestamps
        msg = PacketMsg()
        #convert numpy data to list to fit msg
        msg.buf = packet._data.tolist()
        if isinstance(packet, client.LidarPacket):
            lidar_pub.publish(msg)
            cnt+=1
        elif isinstance(packet,client.ImuPacket):
            imu_pub.publish(msg)
    rospy.set_param("lidar_packet_cnt",cnt)
    #this is the number of packetmsgs needed to create one full pointcloud
    rospy.set_param("ideal_ratio", metadata.mode.cols/column_count)
    print("completed {} secs of data.".format((nano_time()-start_time)/1e9))
