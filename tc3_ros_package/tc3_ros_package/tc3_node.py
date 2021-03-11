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
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from math import cos, sin, sqrt
import numpy as np

class tc3_node(Node):
    def __init__(self):
        super().__init__('tc3_node')
        self.ip = '192.168.42.1'
        self.frame = 'tc3'
        self.self.namespace = 'tc3'   
        self.publish_image = False
        parameter_tree = motorcortex.ParameterTree()
        # Loading protobuf types and hashes
        motorcortex_types = motorcortex.MessageTypes()
        # Open request connection
        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.req, self.sub = motorcortex.connect("ws://"+self.ip+":5558:5557", motorcortex_types, parameter_tree,
                                    certificate=dir_path+"/motorcortex.crt", timeout_ms=1000,
                                    login="root", password="vectioneer")
        self.MarkersMarkers = tracking_cam_msg.Markers
        self.BlobsBlobs = tracking_cam_msg.Blobs
        self.subscription2 = self.sub.subscribe(["root/Processing/BlobDetector/blobBuffer"], "blob", 1)
        self.subscription2.get()
        self.subscription2.notify(self.onBlob)

        self.subscription1 = self.sub.subscribe(["root/Processing/ArucoDetector/markerBuffer"], "marker", 1)
        self.subscription1.get()
        self.subscription1.notify(self.onMarker)

        self.subscription3 = self.sub.subscribe(["root/Processing/CircleDetector/markerCircle"], "circle", 1)
        self.subscription3.get()
        self.subscription3.notify(self.onCircle)
        
        self.subscription4 = self.sub.subscribe(["root/Processing/BlobDetectorNew/blobBuffer"], "blob", 1)
        self.subscription4.get()
        self.subscription4.notify(self.onBlobNew)

        self.subscription5 = self.sub.subscribe(["root/Processing/ProjectionModule/projectionBuffer"], "projection", 1)
        self.subscription5.get()
        self.subscription5.notify(self.onProjection)
        if(self.publish_image == True):
            self.subscription6 = self.sub.subscribe(["root/Comm_task/utilization_max","root/Processing/image"], "camera", 1)
            self.subscription6.get()
            self.subscription6.notify(self.onImage)
    
    def onLog(self,val):
        print(val[0].value)
    def onError(self,val):
        try:
            errors = motorcortex.ErrorList()
            if errors.ParseFromString(val[0].value):
                print(errors)
        except Exception as e:
            print(e)
    def onProjection(self,val):
        try:
            projection = tracking_cam_msg.Projection()
            if projection.ParseFromString(val[0].value):
                print(projection)
        except Exception as e:
            print(e)
    def onBlob(self,val):
        print("find blob")
        try:
            blobs = tracking_cam_msg.Blobs()
            if blobs.ParseFromString(val[0].value):
                self.send_blobs_to_ros(blobs.value)
        except Exception as e:
            print(e)
    def onBlobNew(self,val):
        try:
            blobs = tracking_cam_msg.Blobs()
            if blobs.ParseFromString(val[0].value):
                self.send_new_blobs_to_ros(blobs.value)
        except Exception as e:
            print(e)
    def onMarker(self,val):
        print("find marker")
        try:
            markers = tracking_cam_msg.Markers()
            if markers.ParseFromString(val[0].value):
                self.send_markers_to_ros(markers.value)
        except Exception as e:
            print(e)
    def onCircle(self,val):
        print("find circle")
        try:
            circles = tracking_cam_msg.Circles()
            if circles.ParseFromString(val[0].value):
                # print(circles.value)
                self.send_circles_to_ros(circles.value)
        except Exception as e:
            print(e)
    def onLines(self,val):
        print("find line")
        try:
            lines = tracking_cam_msg.Lines()
            if lines.ParseFromString(val[0].value):
                self.send_lines_to_ros(lines.value)
        except Exception as e:
            print(e)
def main():
    rclpy.init()
    node = tc3_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()