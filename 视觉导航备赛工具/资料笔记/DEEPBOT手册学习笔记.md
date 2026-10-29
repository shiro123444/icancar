# DEEPBOT 使用文档学习笔记

来源：`DEEPBOT 使用文档_V1.4_1008.pdf`，65 页，文档创建时间 2023-10-08。此处是厂商示例行为，不代表当前小车实测结果或 2026 比赛程序。

## 1. 设备与系统（第 1–10 页）

- 手册的 DEEPBOT：x86 工控机、STM32F103RCT6 驱动板和 A4950、四电机麦克纳姆轮，板载 MPU6050；两个 USB 广角摄像头、PS2 手柄、HDMI 屏、电池。实际设备配置须核对。
- 手册给出出厂账号 `deepbot`；出厂密码以原手册为准，实际系统可能已修改。开机分驱动板供电与工控机启动；可接键鼠或 NoMachine 远程登录，网络 IP 在车上查。
- 手柄配对后 POWER 和 MODE 指示灯同时亮；L1 控制前后/旋转，L2 控制前后/横移。

## 2. 底盘与标定（第 11–18 页）

以下是手册 ROS 1 命令示例，当前比赛要求 ROS 2，不能视为 ROS 2 命令：

```bash
roslaunch deepbot bringup.launch
rostopic list
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

- 手册话题：`/cmd_vel` 控制速度，`/battery` 电量，`/imu/data` 融合姿态，`/odom` IMU/编码器融合里程计，`/raw_imu`、`/raw_odom`、`/raw_vel` 是原始数据，另有 `/diagnostics`、`/pid`、`/servo`、`/sonar`。实际消息类型、坐标系、刷新频率应上车核实。
- IMU 静置不动，在 `~/catkin_ws/src/deepbot_project/deepbot/param/imu` 运行 `rosrun imu_calib do_calib`。
- 角速度校正用 `rosrun deepbot_nav calibrate_angular.py` 和 `rosrun rqt_reconfigure rqt_reconfigure`，手册按实际旋转角/360 计算 `anguler_scale`，实测迭代后写回 bringup.launch；拼写与手册一致，需要以实际代码为准。
- 线速度校正用 `calibrate_linear.py` 和 rqt_reconfigure，行驶 1 米后按实际距离/1 米计算 `linear_scale`，再测量并写回配置。手册目标是复测误差约 1–2 cm。电机运动测试需现场安全保障。

## 3. 视觉功能（第 18–35 页）

- USB 摄像头：`roslaunch deepbot camera.launch`。HSV 工具：`roslaunch deepbot_object_detect hsv_tool.launch`；按环境调 LH/LS/LV、UH/US/UV，在结果画面隔离目标，记录阈值；HSV 对光照敏感。
- 色块：`deepbot_object_detect/node/ball_detect.py` 中调整阈值；`ball_object.launch`，图像话题 `object_image`。另有 Haar 人脸 `facehaar_object.launch`、face_recognition `face_object.launch`、身份识别 `facename_object.launch`（先放 jpg 并改代码）。这些是功能演示，非本赛题必要模块。
- 单线视觉跟随：`deepbot_line_follower`，摄像头向下；`deepbot_line.launch` 的 `max_speed`、`color`（0 黄、1 红、2 绿、3 蓝）、`detect_mode`（true 检测调试不运动）、`servo_ctrl`。先 bringup，再 `roslaunch deepbot_line_follower camera.launch`、`roslaunch deepbot_line_follower deepbot_line.launch`。
- AR：`rosrun ar_track_alvar createMarker -s 5 0` 等可生成 5cm 标签；`ar_camera.launch` 的 `maker_size` 要与实物一致，`/ar_pose_marker` 给出相对摄像头姿态。它不是赛场全局定位，需要外参和地图对应关系。
- SVM 人体检测用 `people_object.launch`，手册提示受光线影响，误识别较多。`opencv_apps` 演示 edge_detection、hough_circles、find_contours、convex_hull、general_contours、goodfeature_track、lk_flow、phase_corr。
- YOLOv4-tiny：`roslaunch deepbot_yolo_detection detection.launch`；结果图 `/yolo_object_detection/output_image`，模型位于 `deepbot_yolo_detection/conf`，类别参看 `coco.names`。旧竞速模型 `race.launch` 检测人行道、虚线、直行标志；检测信息 `/yolo_detect_node/detections`。

## 4. 旧版竞速车道逻辑（第 36–42 页）

- 采用 HSV 找可见一侧的内线或外线，结合 `/odom` 姿态在赛段切换；示例判断四元数 z 接近 0 或 1 是其特定起始朝向和场景约定，不能通用地用 z 单独代替 yaw。
- 调整 `auto_run.py` 中 `self.hue_yellow`，多处检查阈值、摄像头角度和光线。`roslaunch deepbot_auto_driver race.launch` 启动示例；`driver_way.py` 订阅里程计与 YOLO 检测，决定换线及动作；`auto_run.py` 负责车道保持。`outer_ring_center_target`、`Inner_ring_center_target` 用于调整距对应线偏移。
- 示例描述了识别人行道停车及绕环岛到终点，与当前赛题两圈、挡板和播报要求不能直接等同。
- 手册提供特定旧固件键盘松键行为作为升级判定；这并不证明当前车需升级，也不应以松键后继续行驶作为一般安全目标。固件与系统修改前先备份并核对版本。

## 5. 数据集与训练（第 42–53 页）

- 采集使用 USB 摄像头与 camorama，指定输出路径和图像格式；使用 labelImg 画框、保存 Pascal VOC XML；图像与标注一一对应。
- VOC 目录包含 `JPEGImages`、`Annotations`、`ImageSets/Main`；手册 `val.py` 通过随机采样生成 trainval/train/val/test（随机分组未固定种子，重复运行可能不一致）。同一连续视频的相邻帧宜按场景分组后再切分，避免测试集过于相似。
- 文中 YOLOv4-tiny 训练章节明确为 Jetson（建议 NX 以上）示例：`voc_label.py` 转 YOLO 标签；`custom.data` 配 classes/train/valid/names/backup；cfg 需要对应 classes、检测层前 filters = `(classes+5)*3`，输入宽高为 32 倍数，batch/subdivisions 依据算力；再用 Darknet 训练和验证。参数是旧教程起点，不直接当作最优配置。
- 训练后的 names/cfg/weights 需配套部署到 detection 包，调整 launch，查看推理结果图；当前 2025 视频另有训练流程，需单独核对。

## 6. 超声波、寻线传感器与固件（第 54–65 页）

- `/sonar` 示例采用 `riki_msgs/Sonar` 的 `distance`，手册说单位 cm；四路循线传感器 `/trackline` 为 `std_msgs/UInt8MultiArray`，四个 0/1 值及实际极性需要按现场调节确认。两者都属于 ROS 1 示例。
- 驱动板固件升级章节列 ST-LINK V2、SWD 接线 GND/SWCLK/SWDIO/VCC 3.3 V、Windows 烧写工具。先核对实际板型、电压、固件、备份及接线，不能仅因教程存在就烧写。

## 7. 本项目下一步核对清单

1. 读取实际车载 OS、ROS 版本、工作区和现存 `deepbot` 包，不假设 PDF 中路径与 2025 镜像相同。
2. 明确底盘是否真为麦轮；获取实际 `/cmd_vel` 消息类型、速度方向、横移、超时停车与急停机制，架空车轮逐项验证。ricecar 阿克曼 PWM/舵机语义不等同。
3. 找到 2025 视觉导航源码、模型、类别文件及推理入口；实际跑一帧并确认输出话题、性能和后处理。
4. 在真实赛道检查摄像机角度、光照、车道线、人行道、红绿灯和挡板识别，逐项测试两圈、3 秒与播报。
5. 地图矢量图只是示意；按官方地图和现场测量建立比例、原点和坐标变换。单一线感知不能自动提供全局定位。
