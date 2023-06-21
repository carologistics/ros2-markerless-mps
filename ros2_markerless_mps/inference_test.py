#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
import mmcv
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from mmdet.apis import inference_detector, init_detector
from mmyolo.registry import VISUALIZERS
from mmcv.visualization import imshow_det_bboxes
from mmcv.transforms import Compose
from tf2_ros import TransformBroadcaster
import tf2_ros
from tf_transformations import quaternion_from_euler
import geometry_msgs.msg
import pyrealsense2 as rs2
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from tf2_geometry_msgs import do_transform_pose
from tf2_ros.buffer import Buffer

from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped


from builtin_interfaces.msg import Time

from rosgraph_msgs.msg import Clock as ClockMsg

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from message_filters import Subscriber, TimeSynchronizer

import time 

from std_srvs.srv import SetBool

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    depth=1
)

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('box_detect_node')

        model_config = '/home/robotino/mmyolo/configs/rtmdet/rtmdet_tiny_fast_rcll.py'
        model_file = '/home/robotino/markerless_mps_ws/src/ros2-markerless-mps/model/rcll.pth'
        self.model = init_detector(model_config, model_file, device='cpu')
        self.bridge = CvBridge()
        self.depth_image = None

        # build test pipeline
        self.model.cfg.test_dataloader.dataset.pipeline[0].type = 'mmdet.LoadImageFromNDArray'
        self.test_pipeline = Compose(self.model.cfg.test_dataloader.dataset.pipeline)
        # init visualizer
        self.visualizer = VISUALIZERS.build(self.model.cfg.visualizer)

        # Initialize the tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # the dataset_meta is loaded from the checkpoint and
        # then pass to the model in init_detector
        self.visualizer.dataset_meta = self.model.dataset_meta
        self.class_names = ('BS', 'CS', 'DS', 'MPS', 'RS', 'SS', 'TL')
        
        self.intrinsics = None
        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = 1280
        self.intrinsics.height = 720
        self.intrinsics.ppx = 638.511
        self.intrinsics.ppy = 358.906
        self.intrinsics.fx = 644.21
        self.intrinsics.fy = 644.21
        self.intrinsics.model = rs2.distortion.brown_conrady
        self.intrinsics.coeffs = [0, 0, 0, 0, 0]

        self.image_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        self.info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_callback, 10)
        self.image_pub = self.create_publisher(Image, '/camera/color/image_raw_with_detections', 10)
        
        # Create a synchronizer for the depth and color images
        sync = TimeSynchronizer([self.image_sub, self.depth_sub], 10)
        sync.registerCallback(self.image_callback)
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        self.enabled = True
        
        # create enabled service
        self.srv = self.create_service(SetBool, 'enable_box_detect', self.enable_object_detector_callback)
        
        #self.transform_buffer = Buffer()
        # Create a transform listener to lookup transforms
        #self.transform_listener = TransformListener(self.transform_buffer, self, spin_thread = True)
        #self.send_camera_static_tf()
        # ros2 run tf2_ros static_transform_publisher --x -0.05 --y 0.1 --z 0.3 --roll -1.74533 --pitch 0.0 --yaw 0.785398 --frame-id robotinobase3plate_top --child-frame-id camera_link

        
        self.current_time = time.time()


        self.get_logger().info('Object detector node has been started')
    
    def enable_object_detector_callback(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = 'Node is enabled.' if self.enabled else 'Node is disabled.'
        return response

    def send_camera_static_tf(self):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'robotinobase1plate_top'
        t.child_frame_id = 'camera_link'
        t.transform.translation.x = -0.05
        t.transform.translation.y = 0.1
        t.transform.translation.z = 0.3
        quat = quaternion_from_euler(-1.74533, 0.0, 0.785398)
        #self.get_logger().info(quat)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_static_broadcaster.sendTransform(t)

    def info_callback(self, msg):
        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = msg.width
        self.intrinsics.height = msg.height
        self.intrinsics.ppx = msg.k[2]
        self.intrinsics.ppy = msg.k[5]
        self.intrinsics.fx = msg.k[0]
        self.intrinsics.fy = msg.k[4]
        self.intrinsics.model = rs2.distortion.brown_conrady
        self.intrinsics.coeffs = [0, 0, 0, 0, 0]

    def image_callback(self, color_msg, depth_msg):
        #self.get_logger().info('Received image at time: ' + str(msg.header.stamp))
        if(self.enabled):
            if(time.time() - self.current_time < 0):
                return
            self.current_time = time.time()
            cv_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            time_header = color_msg.header.stamp
            result = self.detect_objects(cv_image)
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            self.publish_objects(result, depth_image,time_header)
    
    def detect_objects(self, cv_image):
        # Run inference on the image
        result = inference_detector(self.model, cv_image)
        #rospy.loginfo(result)
        # Extract bounding boxes from the result
        # Get candidate predict info with score threshold
        # Show the results
        #img = mmcv.imread(cv_image)
        #img = mmcv.imconvert(cv_image, 'bgr', 'rgb')
        bboxes = []
        score_thr = 0.7
        for i, bbox in enumerate(result.pred_instances.bboxes):
            if(result.pred_instances.scores[i] > score_thr):
                bboxn = [bbox[0], bbox[1], bbox[2], bbox[3]]
                bboxn.append(result.pred_instances.scores[i])
                bboxn.append(result.pred_instances.labels[i])
                bboxes.append(bboxn)


        """ self.visualizer.add_datasample(
            'result',
            image=img,
            data_sample=result,
            draw_gt=False,
            show=False,
            pred_score_thr=score_thr) 
        img = self.visualizer.get_image() """
        
        # publish image with detections
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
        return bboxes
    
    # Define a function to publish a tf transform
    def publish_tf_transform(self, parent_frame, child_frame, translation, rotation, time_header):
        # Create a tf2_ros.TransformStamped message
        
        t = TransformStamped()
        t.header.stamp = time_header
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = translation[0] / 1000.0
        t.transform.translation.y = translation[1] / 1000.0
        t.transform.translation.z = translation[2] / 1000.0
        t.transform.rotation.x = rotation[0] / 1.0
        t.transform.rotation.y = rotation[1] / 1.0
        t.transform.rotation.z = rotation[2] / 1.0
        t.transform.rotation.w = rotation[3] / 1.0


        
        # Publish the transform
        self.tf_broadcaster.sendTransform(t)    

    def publish_objects(self, bboxes, depth_image, time_header):
        
        for i, bbox in enumerate(bboxes):
            
            xmin = int(bbox[0])
            ymin = int(bbox[1])
            xmax = int(bbox[2])
            ymax = int(bbox[3])
            object_id = i # you can set an object_id if you want

            # Estimate the position of the object based on the bounding box and the depth image
            depth_roi = depth_image[ymin:ymax, xmin:xmax]
            #if class is not MPS or TL
            if bbox[5] != 3 and bbox[5] != 6:
                # median depth value of lower 40% of the depth image
                depth_roi = depth_roi[int(depth_roi.shape[0] * 0.8):]
                ymin = int(ymin + depth_roi.shape[0] * 0.8)
            
            #filter RS and CS if score is under 0.76
            if bbox[5] == 1 or bbox[5] == 4:
                if bbox[4] < 0.76:
                    continue
            #EXPERIMENTAL
            #if class is MPS
            #if bbox[5] == 3:
            #    depth_roi = depth_roi + 0.15
            #filter all zeros from depth np array
            depth_roi = np.where(depth_roi == 0, np.nan, depth_roi)
            depth = np.nanmedian(depth_roi)
            #depth = np.nanmean(depth_roi)
            if np.isnan(depth):
                continue # ignore NaN depth values
            
            
            # fx, fy, cx, cy = self.get_camera_intrinsics() # get camera intrinsics
            x = int((xmin + xmax) / 2)
            y = int((ymin + ymax) / 2)
            
            z = depth
            # if class is not tl
            #if bbox[5] != 6:
            z = depth * 1.04 + 50
            
            
            #rospy.loginfo("x: " + str(x) + " y: " + str(y) + " z: " + str(z))
            #u = (x - cx) / fx
            #v = (y - cy) / fy
            
            coords = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [x, y], z)
            #rospy.loginfo("class: " + class_names[bbox[5]] + " x: " + str(coords[0]) + " y: " + str(coords[1]) + " z: " + str(coords[2]))
            #self.get_logger().info(f"class: {self.class_names[bbox[5]]} x: {coords[0]} y: {coords[1]} z: {coords[2]} score: {bbox[4]}")
            if(coords[2] < 0.1):
                self.get_logger().info("Object too close, ignoring")
                continue
            translation = [coords[0], coords[1], coords[2]]
            
            rotation = [0, 0, 0, 1] # you can set a rotation if you want

            # Publish the object as a tf transform
            self.publish_tf_transform('camera_link', '{}_{}'.format(self.class_names[bbox[5]],object_id), translation, rotation, time_header)


def main(args=None):
    rclpy.init(args=args)

    objectdetectornode = ObjectDetectorNode()

    rclpy.spin(objectdetectornode)

    rclpy.shutdown()
    
    
    
    
    rclpy.spin(objectdetectornode)

        
if __name__ == '__main__':
    main()


