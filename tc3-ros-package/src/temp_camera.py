#!/usr/bin/python3

#
#   Developer : Alexey Zakharov (alexey.zakharov@vectioneer.com)
#   All rights reserved. Copyright (c) 2017-2020 VECTIONEER.
#

import motorcortex
import time
import mcx_tracking_cam_pb2 as tracking_cam_msg
from aruco_msgs.msg import Marker, MarkerArray
from blobs_msgs.msg import Blob, BlobArray
from circle_msgs.msg import CircleArray
from line_msgs.msg import LineArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import os
import rospy
import tf
from math import cos, sin, sqrt
import numpy as np


def cameraVal(val, id):
    print("cameraVal")
    print(id, val[0].timestamp, val[0].value)
    print(id, val[1].timestamp, len(val[1].value))


def newVal(val, id):
    print("newVal")
    print(id, val[0].timestamp, val[0].value)
    print(id, val[1].timestamp, val[1].value)


def onBlob(val):
    print("find blob")
    blobs = tracking_cam_msg.Blobs()
    blobs.ParseFromString(val[0].value)
    send_blobs_to_ros(blobs.value)

def onMarker(val):
    print("find marker")
    markers = tracking_cam_msg.Markers()
    markers.ParseFromString(val[0].value)
    send_markers_to_ros(markers.value)

def onBlobNew(val):
    print("find newBlob")
    blobs = tracking_cam_msg.Blobs()
    blobs.ParseFromString(val[0].value)
    send_new_blobs_to_ros(blobs.value)





def main():
    pub_blobs = rospy.Publisher('/blobs', BlobArray, queue_size=1)
    pub_aruco = rospy.Publisher('/markers', MarkerArray, queue_size=1)
    pub_new_blobs = rospy.Publisher('/new_blobs', BlobArray, queue_size=1)
    aruco_tf = tf.TransformBroadcaster()
    # Creating empty object for parameter tree
    parameter_tree = motorcortex.ParameterTree()

    # Loading protobuf types and hashes
    motorcortex_types = motorcortex.MessageTypes()

    dir_path = os.path.dirname(os.path.realpath(__file__))
    req, sub = motorcortex.connect(
        "ws://" + "192.168.42.1" + ":5558:5557",
        motorcortex_types,
        parameter_tree,
        # certificate=dir_path + "/motorcortex.crt",
        timeout_ms=1000,
        login="root",
        password="vectioneer",
    )
    rospy.init_node('motorcortex_proxy')


    
    print("CONNECTED")

    
    # subscription = sub.subscribe(["root/Comm_task/utilization_max",
    #                               "root/Control/Camera/fameCounter"], "test", 1)
    # subscription.get()
    # subscription1 = sub.subscribe(
    #     ["root/Comm_task/utilization_max", "root/Control/Camera/image"], "camera", 1
    # )

    subscription2 = sub.subscribe(["root/Processing/BlobDetector/blobBuffer"], "blob", 1)
    subscription2.get()
    subscription2.notify(onBlob)

    subscription1 = sub.subscribe(["root/Processing/ArucoDetector/markerBuffer"], "marker", 1)
    subscription1.get()
    subscription1.notify(onMarker)

    

    

    subscription4 = sub.subscribe(["root/Processing/BlobDetectorNew/blobBuffer"], "blob", 1)
    subscription4.get()
    subscription4.notify(onBlobNew)

    

    

    # subscription1.get()
    # subscription.notify(lambda a : newVal(a, 0))
    # subscription1.notify(lambda a: cameraVal(a, 1))
    time.sleep(500)

    req.close()
    sub.close()

def send_blobs_to_ros(blobs):
    msg_array = BlobArray()
    msg_array.header.stamp = rospy.Time.now()
    msg_array.header.frame_id = "tracking_cam3"
    for blob in blobs:
        msg = Blob()
        msg.id = blob.id
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "tracking_cam3"

        msg.pose.x = float(blob.cx)
        msg.pose.y = float(blob.cy)
        msg_array.blobs.append(msg)
    pub_blobs.publish(msg_array)

def send_markers_to_ros(markers):
    msg_array = MarkerArray()
    msg_array.header.stamp = rospy.Time.now()
    msg_array.header.frame_id = "tracking_cam3"
    for marker in markers:
        msg = Marker()
        msg.id = marker.id
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "tracking_cam3"
        msg.pose.pose.position.x = marker.transVec3[0]
        msg.pose.pose.position.y = marker.transVec3[1]
        msg.pose.pose.position.z = marker.transVec3[2]
        ax = marker.rotVec3[0]
        ay = marker.rotVec3[1]
        az = marker.rotVec3[2]
        angle = sqrt(ax*ax + ay*ay + az*az)
        cosa = cos(angle*0.5)
        sina = sin(angle*0.5)
        msg.pose.pose.orientation.x = ax*sina/angle
        msg.pose.pose.orientation.y = ay*sina/angle
        msg.pose.pose.orientation.z = az*sina/angle
        msg.pose.pose.orientation.w = cosa
        msg_array.markers.append(msg)
        aruco_tf.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                            msg.header.stamp, "marker_"+str(msg.id), "tracking_cam3")
    pub_aruco.publish(msg_array)

def send_new_blobs_to_ros(blobs):
    msg_array = BlobArray()
    msg_array.header.stamp = rospy.Time.now()
    msg_array.header.frame_id = "tracking_cam3"
    for blob in blobs:
        msg = Blob()
        msg.id = blob.id
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "tracking_cam3"
        msg.pose.x = float(blob.cx)
        msg.pose.y = float(blob.cy)
        msg_array.blobs.append(msg)
    pub_new_blobs.publish(msg_array)

if __name__ == "__main__":
    main()
