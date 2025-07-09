## locobot启动
1. 局域网，ping通，改hosts和.bashrc，两台电脑systemctl stop firewalld，用rostopic list检查。
# 修改主机映射
sudo nano /etc/hosts
# 加入：
# 192.168.104.224    locobot
# 192.168.104.36     monitor

2. 远程连接：```ssh -X locobot@locobot```，密码```locobot```。
3. 机器人启动(重新开始建图)：```roslaunch interbotix_xslocobot_nav xslocobot_nav.launch robot_model:=locobot_wx250s use_lidar:=true rtabmap_args:=-d use_camera:=true```
4. 机器人启动（基于之前的地图继续）:```roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx250s use_nav:=true localization:=true use_lidar:=true```
5. 键盘控制：```roslaunch kobuki_keyop keyop.launch __ns:=locobot```
6. RVIZ控制与可视化：```roslaunch interbotix_xslocobot_descriptions remote_view.launch rviz_frame:=map```

## 建图与导航(实时建图并可视化)
### Joystick
1. bluetooth connect
````
roslaunch interbotix_xslocobot_joy xslocobot_joy.launch robot_model:=locobot_wx250s launch_driver:=false
````
### 实时建图
1. 启动locobot的slam：```roslaunch interbotix_xslocobot_nav xslocobot_nav.launch robot_model:=locobot_wx250s use_lidar:=true rtabmap_args:=-d use_camera:=true```
2. monitor启动rviz窗口：```roslaunch interbotix_xslocobot_descriptions remote_view.launch rviz_frame:=map```。
3. monitor启动yolo：```python3 /home/hxy/yolov5/test_rossevice_server.py```
4. locobot启动拓扑图：```bash /home/locobot/project_hxy/robot/src/remap.sh```

* 指定rtabmap.db位置：```database_path:=~/.ros/rtabmap.db```

### 实时导航
1. 快速启动locobot：```roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx250s use_nav:=true localization:=true use_lidar:=true```
2. monitor启动rviz窗口：```roslaunch interbotix_xslocobot_descriptions remote_view.launch rviz_frame:=map```
3. monitor启动yolo：```python3 /home/hxy/yolov5/test_rossevice_server.py```
4. locobot启动拓扑图：``` python3 /home/locobot/new_topological_graph/xuexi19_asr_room1119_service.py``` 
5. 导航的targetobj手动修改：/home/locobot/new_topological_graph/asr/bin/test.txt


roslaunch explore_lite explore.launch
rostopic echo /explore/navigation_complete

/bin/python3 /home/locobot/project_hxy/robot/src/mapping_topologic_copy.py
