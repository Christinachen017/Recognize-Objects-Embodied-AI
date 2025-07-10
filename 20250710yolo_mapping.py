

import sys
import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from locobot_subscribers.rgbd import RGBD
from arguments import names, landmark
from publishers import YoloClient
from nav_utils import analyze_yolo_msg
from ultralytics import YOLO

# Load a pretrained YOLO model
model = YOLO("yolo11n.pt")

CLASS_COLORS = {
    "person": (1.0, 0.0, 0.0),  # Red
    "chair": (0.0, 1.0, 0.0),    # Green
    "bottle": (0.0, 0.0, 1.0),   # Blue
}

class YoloVisualizer:
    def __init__(self, confidence_threshold=0.5):
        self.bridge = CvBridge()
        self.marker_pub = rospy.Publisher('/yolo/markers', MarkerArray, queue_size=1)
        self.image_pub = rospy.Publisher('/yolo/image_with_boxes', Image, queue_size=1)
        self.confidence_threshold = confidence_threshold
        
    def draw_boxes(self, image, detections):
        """Draw 2D bounding boxes on the image"""
        image_with_boxes = image.copy()
        for bbox, score, cls in detections:
            if score < self.confidence_threshold:
                continue
                
            x1, y1, x2, y2 = map(int, bbox)
            color = tuple(int(255 * c) for c in CLASS_COLORS.get(cls, (1.0, 1.0, 0.0)))
            cv2.rectangle(image_with_boxes, (x1, y1), (x2, y2), color, 2)
            label = f"{cls} {score:.2f}"
            cv2.putText(image_with_boxes, label, (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return image_with_boxes
    
    def publish_markers(self, detections, depth_image, camera_info):
        """Publish 3D markers for Rviz visualization"""
        if not self._validate_camera_info(camera_info):
            return

        marker_array = MarkerArray()
        fx, fy, cx, cy = self._get_camera_params(camera_info)

        # Preprocess depth image
        depth_image = np.nan_to_num(depth_image, nan=0.0)
        depth_image = np.clip(depth_image, 100, 10000)  # 10cm-10m range

        for i, (bbox, score, cls) in enumerate(detections):
            if not self._validate_detection(bbox, depth_image):
                continue
            
            center_y, center_x = int((bbox[1]+bbox[3])/2), int((bbox[0]+bbox[2])/2)
            depth_value = float(depth_image[center_y, center_x])

            if depth_value <= 100:  # Skip if too close (100mm = 0.1m)
                continue

            depth = depth_value / 1000.0  # Convert to meters
            marker = self._create_marker(i, bbox, depth, fx, fy, cx, cy, cls)
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
    
    def _validate_camera_info(self, camera_info):
        """Validate camera calibration data"""
        if not hasattr(camera_info, 'K') or len(camera_info.K) < 6:
            rospy.logwarn("Invalid camera calibration: K matrix missing or incomplete")
            return False
        return True

    def _get_camera_params(self, camera_info):
        """Extract camera intrinsic parameters"""
        return (
            float(camera_info.K[0]),  # fx
            float(camera_info.K[4]),   # fy
            float(camera_info.K[2]),   # cx
            float(camera_info.K[5])    # cy
        )

    def _validate_detection(self, bbox, depth_image):
        """Validate detection bounding box"""
        if len(bbox) != 4 or None in bbox:
            return False
        
        try:
            center_x = int((bbox[0] + bbox[2]) / 2)
            center_y = int((bbox[1] + bbox[3]) / 2)
            return (0 <= center_x < depth_image.shape[1] and 
                    0 <= center_y < depth_image.shape[0])
        except (TypeError, IndexError):
            return False

    def _create_marker(self, marker_id, bbox, depth, fx, fy, cx, cy, cls):
        """Create a 3D marker for visualization"""
        marker = Marker()
        marker.header.frame_id = "locobot/camera_depth_optical_frame"
        marker.header.stamp = rospy.Time.now()
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Calculate 3D position
        center_x, center_y = (bbox[0]+bbox[2])/2, (bbox[1]+bbox[3])/2
        marker.pose.position.x = depth
        marker.pose.position.y = -(center_x - cx) * depth / fx
        marker.pose.position.z = -(center_y - cy) * depth / fy
            
        # Set dimensions based on bounding box
        marker.scale.x = (bbox[2] - bbox[0]) * depth / fx
        marker.scale.y = (bbox[3] - bbox[1]) * depth / fy
        marker.scale.z = 0.1  # Fixed depth for visualization

        # Set color based on class
        r, g, b = CLASS_COLORS.get(cls, (1.0, 1.0, 0.0))  # Default: yellow
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.5  # Semi-transparent
        marker.lifetime = rospy.Duration(0.2)  # Real-time updates

        return marker

    def _preprocess_depth(self, depth_img):
        """Preprocess depth image"""
        return np.nan_to_num(depth_img, nan=0.0)

    def _get_detections(self, color_img):
        """Get detections from YOLO model"""
        results = model(color_img)
        detections = []
        for result in results:
            for box in result.boxes:
                bbox = box.xyxy[0].tolist()
                score = box.conf[0].item()
                cls = model.names[int(box.cls[0].item())]
                detections.append((bbox, score, cls))
        return detections

    def _visualize_results(self, color_img, depth_img, camera_info, detections):
        """Visualize all results"""
        # Publish image with 2D boxes
        img_with_boxes = self.draw_boxes(color_img.copy(), detections)
        try:
            img_msg = self.bridge.cv2_to_imgmsg(img_with_boxes, "bgr8")
            self.image_pub.publish(img_msg)
            self.publish_markers(detections, depth_img, camera_info)
        except CvBridgeError as e:
            rospy.logerr(f"Image conversion failed: {str(e)}")

def main():
    rospy.init_node('yolo_visualization', anonymous=True)

    try:
        visualizer = YoloVisualizer(confidence_threshold=0.4)
        rgbd = RGBD()
        camera_info = rospy.wait_for_message(
            '/locobot/camera/color/camera_info', 
            CameraInfo, 
            timeout=5.0
        )
        rospy.loginfo("Camera calibration data received")
    except rospy.ROSException as e:
        rospy.logerr(f"Failed to get camera info: {str(e)}")
        return
    except Exception as e:
        rospy.logerr(f"Initialization error: {str(e)}")
        return

    rate = rospy.Rate(10)  # 10Hz
    
    while not rospy.is_shutdown():
        try:
            color_img = rgbd.get_rgb()
            depth_img = rgbd.get_depth()
            
            if color_img is None or depth_img is None:
                rospy.logwarn_throttle(1.0, "Waiting for camera frames...")
                rate.sleep()
                continue

            depth_img = visualizer._preprocess_depth(depth_img)
            detections = visualizer._get_detections(color_img)

            if detections:
                visualizer._visualize_results(color_img, depth_img, camera_info, detections)

        except KeyboardInterrupt:
            rospy.loginfo("Shutting down...")
            break
        except Exception as e:
            rospy.logerr_throttle(1.0, f"Error in main loop: {str(e)}")
                         
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
