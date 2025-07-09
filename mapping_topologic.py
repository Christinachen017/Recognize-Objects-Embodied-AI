import signal
import sys
import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from locobot_subscribers.rgbd import RGBD
from project_hxy.robot.src.arguments import get_args, names, landmark
from publishers import YoloClient
from nav_utils import analyze_yolo_msg

CLASS_COLORS = {
    "person": (1.0, 0.0, 0.0),  # Red
    "chair": (0.0, 1.0, 0.0),   # Green
    "bottle": (0.0, 0.0, 1.0),  # Blue
}

# Publish YOLO detections
class YoloVisualizer:
    # Initialization
    def __init__(self, confidence_threshold=0.5):
        self.bridge = CvBridge() # convert OpenCV(numpy) to ROS(image)
        self.marker_pub = rospy.Publisher('/yolo/markers', MarkerArray, queue_size=1) # publsih 3D bounding box markers for Rviz
        self.image_pub = rospy.Publisher('/yolo/image_with_boxes', Image, queue_size=1) # publsh RGB image with YOLO boxes
        self.confidence_threshold = confidence_threshold # setting minimum score of detection
        
    # Draw 2D bounding boxes        
    def draw_boxes(self, image, detections):
         for bbx, score, cls in detections:
            if score < self.confidence_threshold:
                continue
                # ignore lower scores
            x1, y1, x2, y2 = map(int, bbx)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2) # draw a rectangle on image-detected
            label = f"{cls} {score:.2f}"
            cv2.putText(image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2) # class name and score 
        return image
    
    # Create 3D cubes as Rviz markers
    def publish_markers(self, detections, depth_image, camera_info):
        marker_array = MarkerArray()
        for i, (bbx, score, cls) in enumerate(detections):
            if score < self.confidence_threshold:
                continue
            
            # Find center of the bounding cube
            center_x = int((bbx[0] + bbx[2])/2)
            center_y = int((bbx[1] + bbx[3])/2)
            
            # Skip if center is outside image
            if center_y >= depth_image.shape[0] or center_x >= depth_image.shape[1]:
                rospy.logwarn(f"Skipping detection {cls}: index out of bounds.")
                continue
            
            # Calculate depth at center
            depth = depth_image[center_y, center_x] / 1000.0  # mm → meters
            if np.isnan(depth) or depth <= 0.1:
                rospy.logwarn(f"Invalid depth at center of {cls}: {depth}")
                continue
            
            # Create marker
            marker = Marker()
            marker.header.frame_id = "locobot/camera_depth_optical_frame"
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Set 3D position
            try:
                marker.pose.position.x = depth # fwd from camera
                marker.pose.position.y = -(center_x - camera_info.width/2) * depth / camera_info.fx # sideways
                marker.pose.position.z = -(center_y - camera_info.height/2) * depth / camera_info.fy # height
            except AttributeError:
                rospy.logwarn("Camera intrinsics not available.")
                continue

            # Set cube dimensions based on bounding box
            marker.scale.x = (bbx[2] - bbx[0]) * depth / camera_info.fx
            marker.scale.y = (bbx[3] - bbx[1]) * depth / camera_info.fy
            marker.scale.z = 0.1

            # Color by class
            r, g, b = CLASS_COLORS.get(cls, (1.0, 1.0, 0.0))  # Default: yellow
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 0.5
            marker.lifetime = rospy.Duration(0.1) # real-time updates

            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

def main():
    rospy.init_node('yolo_visualization', anonymous=True) # Create an ROS node
    rospy.loginfo("YOLO Visualizer Node Started")
    
    # Initialize components
    visualizer = YoloVisualizer(confidence_threshold=0.4) # Drawing boxes and Rviz Markers
    rgbd = RGBD() # Subscribe to RGBD sensor
    client = YoloClient() # Send RGB image to YOLO detector
    
    rate = rospy.Rate(10)  # 10Hz
    
    while not rospy.is_shutdown(): # Process incoming data at 10Hz rate until shutdown
        # Get camera data
        try:
            camera_bgr = rgbd.get_rgb() # color image (2D)
            camera_depth = rgbd.get_depth() # Depth image (3D)
            camera_info = rgbd.camera_info # Projecting 2D to 3D

            if camera_bgr is None or camera_depth is None:
                rospy.logwarn("No image data received. Skipping frame.")
                continue
        
            # Get YOLO detections
            resp = client.img_result(camera_bgr) # Client sends image result
            detections = analyze_yolo_msg(resp, id2names=names, valid_names=landmark) # Analyzing YOLO results
        
            # Visualize results
            if detections:
                # Publish image with 2D boxes
                img_with_boxes = visualizer.draw_boxes(camera_bgr.copy(), detections) # Draw rectangles and tag labels
                visualizer.image_pub.publish(visualizer.bridge.cv2_to_imgmsg(img_with_boxes, "bgr8")) # Pulish images
                visualizer.image_pub.publish(img_msg)

                # Publish 3D markers
                visualizer.publish_markers(detections, camera_depth, camera_info) # 3D Rviz

            else:
                rospy.loginfo("No detections in current frame.")

        except Exception as e:
            rospy.logerr(f"Error in main loop: {str(e)}"
                         
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



'''
import signal
import sys
import os
# interbotix package path

interbotix_path = "/home/monitor/interbotix_ws1/src/interbotix_ros_toolboxes/interbotix_xs_toolbox"
sys.path.append(interbotix_path)

from interbotix_xs_modules.locobot import InterbotixLocobotXS
import cv2

#from topological_map import TopologicalMap
#from topological_map import draw_graph_rospub
import pickle
import time
from socket import *

import numpy as np
import rospy
import os
import time
from locobot_subscribers.rgbd import RGBD
from project_hxy.robot.src.arguments import get_args, names, landmark
from publishers import Visualization, YoloClient
# from explore import ExploreMonitor
from nav_utils import analyze_yolo_msg
from locobot_subscribers import CostMap
import logging
import shutil
from explore import ExploreMonitor


def signal_handler(signal, frame):
    """中断信号处理
    TODO:退出前保存
    Args:
        signal (_type_): _description_
        frame (_type_): _description_
    """
    print("结束建图程序")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def make_log_dirs(log_dir):
    if os.path.exists(log_dir):
        overwrite = input(f'save log path \"{log_dir}\" already exist, overwrite? y|n')
        if overwrite == 'y':
            try:
                shutil.rmtree(log_dir)
            except Exception as e:
                rospy.logerr(f'Error {e} occured while overwriting {log_dir}')
                exit(-1)
        else:
            rospy.logerr("Use \'--output\' option to specify a new output file location.")
    os.makedirs(log_dir)
    os.mkdir(os.path.join(log_dir, "color"))
    os.mkdir(os.path.join(log_dir, "depth"))
    os.mkdir(os.path.join(log_dir, "odom"))
    os.mkdir(os.path.join(log_dir, "topo_map"))
    os.mkdir(os.path.join(log_dir, "topo_map_png"))

'''
def topological_map_init(origin_map_file, delete_origin_map=False):
    # 现功能：若有语义地图文件提供，则加载提供的地图，否则重新建图，运行Explore
    do_exploration = False
    if delete_origin_map:
        topological_map = TopologicalMap()
        do_exploration = True
    elif origin_map_file is None:
        choice = input("no previous built map file is provided, rebuild map? Enter y to continue.")
        if choice == 'y':
            topological_map = TopologicalMap()
            do_exploration = True
        else:
            logging.critical('use opt \"-i\" \{file\} to load existing topological map.')
            exit(-1)
    else:
        topological_map = TopologicalMap.load(origin_map_file)
        # TODO: 检查语义地图与costmap是否一致，一致则不再建图，不一致则需要重新建图
    return topological_map, do_exploration
'''

def main():
    locobot = InterbotixLocobotXS(
        robot_model="locobot_wx250s", 
        use_move_base_action=True  # 阻塞base.move_to_pose()，没到达指定地点前阻塞进程
    )
    time.sleep(0.2)
    rospy.logdebug('Initialized locobot.')
    args = get_args()
    save_path = args.output
    start_mdh = time.strftime("%m_%d_%H", time.localtime())
    # load locobot model instance
    rospy.loginfo(f'Output dir = {save_path}')
    # mapping log data
    if args.save_log:
        save_path_log = os.path.join(save_path, "mapping_log", start_mdh)
        rospy.loginfo(f'Log saved to {save_path_log}')
        make_log_dirs(save_path_log)
    # ===================== initialize ========================

    # 要使用locobot.base.get_locobot_position()，机器人必须先移动一点，否则返回为None
    locobot.base.reset_odom()  # Reset odometry to zero
    rospy.logdebug("reset_odom()")
    locobot.camera.pan_tilt_go_home()   # initialize camera
    rospy.logdebug("pan_tilt_go_home()")
    res = locobot.base.move_to_pose(0.1, 0, 0, wait=True)  # 阻塞执行
    if res:
        rospy.loginfo("Successful moved")
    else:
        rospy.logerr("Unsuceessful move! Retring...")
        while(not res):
            res = locobot.base.move_to_pose(0.1, 0, 0, wait=True)
        rospy.loginfo("Successful moved")
    # global_costmap = CostMap(global_costmap=True)  # 用于校正拓扑图与语义地图时使用
    topological_map, do_exploration = topological_map_init(
        args.origin_map_file, args.delete_origin_map)  # 语义拓扑地图初始化
    client = YoloClient()  # yolo msg client
    rgbd = RGBD()  # 接收机器人的相机data
    visualization_pub = Visualization()  # 发布建立的拓扑图的可视化
    # 自动探索
    explore_monitor = None
    # 自动建图
    if do_exploration:
        explore_monitor = ExploreMonitor()
    ccount = 0
    x, y = np.array([]),np.array([])  # locobot的历史轨迹
    start_time = time.time()
    rospy.loginfo(f'start service time:\t {start_time}')
    while not rospy.is_shutdown():
        if explore_monitor is not None:
            # 若正在建图
            if explore_monitor.completed == True:
                rospy.loginfo("自动探索已结束！")
        # locobot msgs
        odom = locobot.base.get_locobot_position()  
        odom = np.array(odom)  # 机器人坐标 [x,y,角度]
        # rospy.logdebug('odom', odom)
        camera_bgr = rgbd.get_rgb()
        camera_depth = rgbd.get_depth()  # shape=480*640, 0.2m~20m为有效区域，其余部分设为0
        rospy.logdebug(f"Loop {ccount+1} get camera data took {time.time()-start_time:.6f} seconds")
        time1 = time.time()
        
        # 解析yolo结果
        resp = client.img_result(camera_bgr)
        rospy.logdebug(f"YOLO result = {resp}")
        # rospy.logdebug(f"Loop {ccount+1} get yolo took {(time.time()-time1):.6f} seconds")
        for bbx, score, cls in analyze_yolo_msg(resp, id2names=names, valid_names=landmark):
            topological_map.add_node(odom, bbx, camera_depth, cls, score)
        topological_map.refresh_zones()
        # 发布可视化的拓扑图
        x, y = np.append(x, odom[0]), np.append(y, odom[1])
        topo_img = draw_graph_rospub(topological_map, x, y)  # rgb cv2 np.array(np.uint8)
        visualization_pub.img_pub_topo_img(topo_img)
        # 保存log
        if args.save_log:
            depth_mapped = np.uint8(camera_depth / 2000 * 255)
            cv2.imwrite(os.path.join(save_path_log, 'color', f'{ccount}.png'), camera_bgr)
            cv2.imwrite(os.path.join(save_path_log, 'depth', f'{ccount}.png'), depth_mapped)
            np.save(os.path.join(save_path_log, 'odom', f'{ccount}.npy'), odom)
            cv2.imwrite(os.path.join(save_path_log, 'topo_map_png', f'{ccount}.png'), topo_img)
            topological_map.save(os.path.join(save_path_log, 'topo_map', f'{ccount}.pkl'))
        ccount = ccount + 1

        end_time = time.time()  # 记录循环结束时的时间
        elapsed_time = end_time - start_time  # 计算消耗的时间
        start_time = end_time
        # 输出本次循环消耗的时间，保留六位小数
        rospy.logdebug(f"Loop {ccount+1} took {elapsed_time:.6f} seconds")
    rospy.spin()


if __name__=='__main__':
    main()
    
'''
