# Hikrobot Camera ROS 2 Driver

基于 Hikrobot MVS SDK 的 ROS 2 Humble 工业相机驱动。

本项目实现：

- MVS 相机枚举
- 相机自动连接
- MVS 图像取流
- RGB8 图像转换
- ROS 2 `sensor_msgs/msg/Image` 发布
- `/image_raw` 图像话题
- Exposure Time 参数
- Gain 参数
- Frame Rate 参数
- Pixel Format 参数
- ROS 2 动态参数修改
- USB/相机断线检测
- 自动重新连接
- 重新 `StartGrabbing`
- 断线重连后恢复相机参数
- RViz2 图像显示

---

## 1. 环境

测试环境：

- Ubuntu 22.04
- ROS 2 Humble
- C++17
- Hikrobot MVS SDK 4.8.1

---

## 2. MVS SDK

本项目依赖 Hikrobot MVS SDK。

MVS SDK 需要从海康机器人官方渠道下载安装。

MVS SDK 不是普通 ROS 2 软件包，因此不能依赖 `rosdep` 自动安装。

本机 MVS SDK 路径：

```text
/opt/MVS
```

头文件：

```text
/opt/MVS/include/MvCameraControl.h
```

64 位动态库：

```text
/opt/MVS/lib/64/libMvCameraControl.so
```

项目的 CMakeLists.txt 已配置上述路径。

---

## 3. 创建工作空间

```zsh
mkdir -p ~/hik_camera_ws/src
cd ~/hik_camera_ws
```

将 hik_camera package 放入：

```text
~/hik_camera_ws/src/hik_camera
```

---

## 4. 编译

首先加载 ROS 2 Humble：

```zsh
source /opt/ros/humble/setup.zsh
```

如果使用本项目之前的本机测试环境：

```zsh
export ROS_LOCALHOST_ONLY=1
```

然后编译：

```zsh
cd ~/hik_camera_ws
colcon build --symlink-install
```

编译完成后：

```zsh
source install/setup.zsh
```

---

## 5. 使用 Launch 启动

运行：

```zsh
ros2 launch hik_camera hik_camera.launch.py
```

Launch 文件会启动：

```text
hik_camera_node
```

并自动进行相机初始化和图像取流。

---

## 6. 图像话题

默认图像话题：

```text
/image_raw
```

消息类型：

```text
sensor_msgs/msg/Image
```

图像编码：

```text
rgb8
```

坐标系：

```text
camera_frame
```

检查：

```zsh
ros2 topic info /image_raw --verbose
```

查看消息：

```zsh
ros2 topic echo /image_raw --once --field header
```

---

## 7. 查看图像

### RViz2

运行：

```zsh
rviz2
```

设置：

```text
Fixed Frame = camera_frame
```

添加：

```text
Image
```

Topic：

```text
/image_raw
```

确认 Image 状态为：

```text
Status: Ok
```

并显示实时相机画面。

### rqt_image_view

也可以使用：

```zsh
ros2 run rqt_image_view rqt_image_view
```

选择：

```text
/image_raw
```

查看实时图像。

---

## 8. ROS 2 参数

本节点支持以下 ROS 2 参数：

```text
exposure_time
gain
frame_rate
pixel_format
```

查看参数：

```zsh
ros2 param list /hik_camera
```

---

## 9. Exposure Time

读取：

```zsh
ros2 param get /hik_camera exposure_time
```

修改：

```zsh
ros2 param set /hik_camera exposure_time <value>
```

修改结果由节点参数回调处理，并调用 MVS SDK 设置相机曝光时间。

---

## 10. Gain

读取：

```zsh
ros2 param get /hik_camera gain
```

修改：

```zsh
ros2 param set /hik_camera gain <value>
```

参数修改时会进行范围检查，并检查 MVS SDK 返回结果。

---

## 11. Frame Rate

读取：

```zsh
ros2 param get /hik_camera frame_rate
```

修改：

```zsh
ros2 param set /hik_camera frame_rate 30.0
```

需要注意：

`frame_rate` 是相机配置的目标帧率，而：

```zsh
ros2 topic hz /image_raw
```

测量的是 ROS 2 图像消息实际到达频率。

两者可能存在差异。

实际频率可以使用：

```zsh
ros2 topic hz /image_raw
```

进行测量。

---

## 12. Pixel Format

读取：

```zsh
ros2 param get /hik_camera pixel_format
```

Pixel Format 使用 MVS SDK 的枚举参数。

部分相机在修改 Pixel Format 时可能需要停止图像采集，再重新启动采集。

因此修改 Pixel Format 时必须使用当前相机实际支持的枚举值。

---

## 13. 动态参数修改

本节点使用 ROS 2 参数回调处理运行期间的参数修改。

例如：

```zsh
ros2 param set /hik_camera frame_rate 10.0
```

修改后：

```zsh
ros2 param get /hik_camera frame_rate
```

检查新值。

同时：

```zsh
ros2 topic echo /image_raw --once --field header
```

确认图像仍正常发布。

---

## 14. 断线检测与自动重连

运行过程中如果相机 USB 连接断开，节点会检测取流异常。

检测到相机断线后：

- 停止当前采集
- 关闭当前相机连接
- 重新枚举设备
- 重新连接相机
- 重新设置相机参数
- 重新执行 `StartGrabbing`
- 恢复 `/image_raw` 发布

因此相机重新连接后可以继续进行 ROS 2 图像发布。

---

## 15. 检查图像发布频率

运行：

```zsh
ros2 topic hz /image_raw
```

例如：

```text
average rate: ...
```

该值代表 ROS 2 接收端实际测得的图像消息频率。

---

## 16. 检查图像带宽

可以使用：

```zsh
ros2 topic bw /image_raw
```

观察图像话题的数据带宽。

---

## 17. 常见问题

### 17.1 找不到 MVS SDK

检查：

```zsh
ls -l /opt/MVS/include/MvCameraControl.h
```

以及：

```zsh
ls -l /opt/MVS/lib/64/libMvCameraControl.so
```

如果文件不存在，需要重新安装 MVS SDK。

### 17.2 No Hikvision camera found

检查：

- 相机 USB 是否连接
- MVS GUI 是否正在独占使用相机
- 是否同时启动了多个 `hik_camera` 节点
- 相机是否已经掉线

可以重新插拔 USB，然后重新启动节点。

### 17.3 RViz2 显示 No Image

检查：

```zsh
ros2 topic info /image_raw --verbose
```

确认存在：

```text
Publisher: hik_camera
Subscriber: rviz2
```

然后在 RViz2 中：

```text
Fixed Frame = camera_frame
```

Image Topic：

```text
/image_raw
```

也可以使用：

```zsh
ros2 run rqt_image_view rqt_image_view
```

进行交叉验证。

---

## 18. 最终验收命令

构建：

```zsh
cd ~/hik_camera_ws
colcon build --symlink-install
```

启动：

```zsh
source install/setup.zsh
ros2 launch hik_camera hik_camera.launch.py
```

节点：

```zsh
ros2 node list
```

图像话题：

```zsh
ros2 topic info /image_raw --verbose
```

图像消息：

```zsh
ros2 topic echo /image_raw --once --field header
```

图像频率：

```zsh
ros2 topic hz /image_raw
```

参数：

```zsh
ros2 param list /hik_camera
```

---

## 19. 项目状态

当前版本已经实现：

- MVS 相机枚举
- 相机连接
- MVS 取流
- RGB8 转换
- ROS 2 `/image_raw`
- RViz2 图像显示
- Exposure Time 参数
- Gain 参数
- Frame Rate 参数
- Pixel Format 参数
- 参数动态修改
- 断线检测
- 自动重连
- 参数恢复
- ROS 2 Humble 构建
- Launch 启动
