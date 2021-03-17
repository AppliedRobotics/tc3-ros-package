import motorcortex
import time
import tc3_ros_package.mcx_tracking_cam_pb2 as tracking_cam_msg
from aruco_msgs.msg import Marker, MarkerArray
from blobs_msgs.msg import Blob, BlobArray
from circle_msgs.msg import CircleArray
from line_msgs.msg import LineArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import TransformStamped
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
        print("ss")
        self.frame = 'tc3'
        self.namespace = 'tc3'   
        self.publish_image = False
        self.pub_blobs = self.create_publisher(BlobArray, "blobs", 10)
        self.pub_new_blobs = self.create_publisher(BlobArray, "new_blobs", 10)
        self.pub_circles = self.create_publisher(CircleArray, "circles", 10)
        self.pub_lines = self.create_publisher(LineArray, "lines", 10)
        self.pub_aruco = self.create_publisher(MarkerArray, "markers", 10)
        self.Image = self.create_publisher(Image, "image", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

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
        try:
            markers = tracking_cam_msg.Markers()
            if markers.ParseFromString(val[0].value):
                self.send_markers_to_ros(markers.value)
        except Exception as e:
            print(e)
    def onCircle(self,val):
        try:
            circles = tracking_cam_msg.Circles()
            if circles.ParseFromString(val[0].value):
                # print(circles.value)
                self.send_circles_to_ros(circles.value)
        except Exception as e:
            print(e)
    def onLines(self,val):
        try:
            lines = tracking_cam_msg.Lines()
            if lines.ParseFromString(val[0].value):
                self.send_lines_to_ros(lines.value)
        except Exception as e:
            print(e)
    def send_blobs_to_ros(self,blobs):
        msg_array = BlobArray()
        now = self.get_clock().now().to_msg()
        msg_array.header.stamp = now
        msg_array.header.frame_id = self.frame
        for blob in blobs:
            msg = Blob()
            msg.id = blob.id
            msg.header.stamp = now
            msg.header.frame_id = self.frame
            msg.pose.x = float(blob.cx)
            msg.pose.y = float(blob.cy)
            msg_array.blobs.append(msg)
        self.pub_blobs.publish(msg_array)

    def send_new_blobs_to_ros(self,blobs):
        msg_array = BlobArray()
        now = self.get_clock().now().to_msg()
        msg_array.header.stamp = now
        msg_array.header.frame_id = self.frame
        for blob in blobs:
            msg = Blob()
            msg.id = blob.id
            msg.header.stamp = now
            msg.header.frame_id = self.frame
            msg.pose.x = float(blob.cx)
            msg.pose.y = float(blob.cy)
            msg_array.blobs.append(msg)
        self.pub_new_blobs.publish(msg_array)

    def send_circles_to_ros(self,circles):
        msg_array = CircleArray()
        now = self.get_clock().now().to_msg()
        msg_array.header.stamp = now
        msg_array.header.frame_id = self.frame
        for circle in circles:
            msg = Circle()
            msg.center.x = float(circle.x)
            msg.center.y = float(circle.y)
            msg.radius = float(circle.r)
            msg_array.circles.append(msg)
        self.pub_circles.publish(msg_array)


    def send_lines_to_ros(self,lines):
        msg_array = LineArray()
        now = self.get_clock().now().to_msg()
        msg_array.header.stamp = now
        msg_array.header.frame_id = self.frame
        for line in lines:
            msg = Line()
            msg.first.x = float(line.x0)
            msg.first.y = float(line.y0)
            msg.second.x = float(line.x1)
            msg.second.y = float(line.y1)
            msg_array.lines.append(msg)
        self.pub_lines.publish(msg_array)


    def send_markers_to_ros(self,markers):
        msg_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        msg_array.header.stamp = now
        msg_array.header.frame_id = self.frame
        for marker in markers:
            msg = Marker()
            msg.id = marker.id
            msg.header.stamp = now
            msg.header.frame_id = self.frame
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
            transform_stamped_msg = TransformStamped()
            transform_stamped_msg.header.stamp = now
            transform_stamped_msg.header.frame_id = self.frame
            transform_stamped_msg.child_frame_id = "marker"+str(marker.id)
            transform_stamped_msg.transform.translation.x = marker.transVec3[0]
            transform_stamped_msg.transform.translation.y = marker.transVec3[1]
            transform_stamped_msg.transform.translation.z = marker.transVec3[2]
            transform_stamped_msg.transform.rotation.x = ax*sina/angle
            transform_stamped_msg.transform.rotation.y = ay*sina/angle
            transform_stamped_msg.transform.rotation.z = az*sina/angle
            transform_stamped_msg.transform.rotation.w = cosa
            self.tf_broadcaster.sendTransform(transform_stamped_msg)
        self.pub_aruco.publish(msg_array)
    
    def onImage(self,val):
        # image = cv2.imdecode(np.frombuffer(val[1].value, np.uint8), -1)
        # self.pub_image.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        # self.pub_compressed_image.publish(self.bridge.cv2_to_compressed_imgmsg(image))
        a = 1
def main():
    rclpy.init()
    node = tc3_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()