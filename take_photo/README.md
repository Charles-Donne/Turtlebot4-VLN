## PC-Server
### 0.加载环境（初始化）
#### 启动ros2
```plain
source /opt/ros/humble/setup.bash
```

```plain
source install/setup.bash
```

##### 1️⃣ `/opt/ros/humble/setup.bash`
+ 这是 **系统安装的 ROS2 Humble 基础环境**。
+ 它包含：
    - ROS2 的命令行工具 (`ros2`, `colcon`, `ros2 topic` 等)
    - 系统自带的包（比如 `rclcpp`, `sensor_msgs`）
    - 基础环境变量（`ROS_DISTRO`, `AMENT_PREFIX_PATH` 等）
+ **作用**：先让 shell 能识别 ROS2 基础命令和系统包。

---

##### 2️⃣ `~/ros2_ws/install/setup.bash`
+ 这是你 **自定义工作空间 build 后的环境**。
+ 它包含：
    - 你自己编译的 package（比如 `depthai_examples`、`depthai_bridge`）
    - 将你的工作空间加入到 `AMENT_PREFIX_PATH`，覆盖系统默认包（如果有同名包）
+ **作用**：让 shell 能找到你自己编译的包，并覆盖系统包（如果必要）。

---

##### 3️⃣ 正确顺序
通常做法：

```plain
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

+ 先加载系统 ROS2，保证基础工具可用。
+ 再加载你自己的工作空间，保证自定义包被找到。

注意顺序不能反，先加载工作空间再加载系统 ROS 会导致你的自定义包可能被系统包覆盖。

---

##### 4️⃣ 小技巧
+ 每次打开新的 terminal，建议用：

```plain
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

+ 可以把它加到 `~/.bashrc`：

```plain
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

这样每次开新的 shell 就自动生效。

### 1.启动程序
```bash
ros2 launch depthai_examples photo_save.launch.py
```

### <font style="color:rgb(0, 0, 0);">2.终端拍照</font>
<font style="color:rgba(0, 0, 0, 0.85) !important;">根据代码中定义的 </font>**<font style="color:rgb(0, 0, 0) !important;">服务名称（</font>**`**<font style="color:rgb(0, 0, 0);">capture_photo</font>**`**<font style="color:rgb(0, 0, 0) !important;">）</font>**<font style="color:rgba(0, 0, 0, 0.85) !important;"> 和 </font>**<font style="color:rgb(0, 0, 0) !important;">服务类型（</font>**`**<font style="color:rgb(0, 0, 0);">std_srvs/srv/Trigger</font>**`**<font style="color:rgb(0, 0, 0) !important;">）</font>**<font style="color:rgba(0, 0, 0, 0.85) !important;">，触发拍照的命令是：</font>

```bash
ros2 service call /capture_photo std_srvs/srv/Trigger
```

##### <font style="color:rgb(0, 0, 0);">命令解析</font>
+ `<font style="color:rgb(0, 0, 0);">/capture_photo</font>`<font style="color:rgba(0, 0, 0, 0.85) !important;">：服务的完整名称（代码中未加节点名前缀，是全局服务）；</font>
+ `<font style="color:rgb(0, 0, 0);">std_srvs/srv/Trigger</font>`<font style="color:rgba(0, 0, 0, 0.85) !important;">：服务类型，与代码中</font><font style="color:rgba(0, 0, 0, 0.85) !important;"> </font>`<font style="color:rgb(0, 0, 0);">std_srvs::srv::Trigger</font>`<font style="color:rgba(0, 0, 0, 0.85) !important;"> </font><font style="color:rgba(0, 0, 0, 0.85) !important;">完全匹配。</font>

##### <font style="color:rgb(0, 0, 0);">调用成功的预期结果</font>
1. **<font style="color:rgb(0, 0, 0) !important;">终端响应</font>**<font style="color:rgba(0, 0, 0, 0.85) !important;">：</font>**<font style="color:rgba(0, 0, 0, 0.85);">plaintext</font>**

```plain
waiting for service /capture_photo...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Photo saved to: /home/ubuntu/oak_photos/photo_1756036000000000000.jpg')
```

2. **<font style="color:rgb(0, 0, 0) !important;">节点日志</font>**<font style="color:rgba(0, 0, 0, 0.85) !important;">（在启动</font><font style="color:rgba(0, 0, 0, 0.85) !important;"> </font>`<font style="color:rgba(0, 0, 0, 0.85) !important;">photo_capture.launch.py</font>`<font style="color:rgba(0, 0, 0, 0.85) !important;"> </font><font style="color:rgba(0, 0, 0, 0.85) !important;">的终端中）：</font>**<font style="color:rgba(0, 0, 0, 0.85);">plaintext</font>**

```plain
[photo_capture_node-1] [INFO] [1756036001.123456789] [photo_capture_node]: Photo saved: /home/ubuntu/oak_photos/photo_1756036000000000000.jpg
```

3. **<font style="color:rgb(0, 0, 0) !important;">照片保存路径</font>**<font style="color:rgba(0, 0, 0, 0.85) !important;">：默认保存在 </font>`<font style="color:rgba(0, 0, 0, 0.85) !important;">~/oak_photos</font>`<font style="color:rgba(0, 0, 0, 0.85) !important;"> 目录下，文件名以时间戳命名（如 </font>`<font style="color:rgba(0, 0, 0, 0.85) !important;">photo_1756036000000000000.jpg</font>`<font style="color:rgba(0, 0, 0, 0.85) !important;">）。</font>

##### <font style="color:rgb(0, 0, 0);">验证服务是否存在</font>
<font style="color:rgba(0, 0, 0, 0.85) !important;">若不确定服务是否正常注册，可在新终端中执行以下命令查看：</font>

```bash
# 检查 capture_photo 服务是否存在
ros2 service list | grep capture_photo
```

+ <font style="color:rgba(0, 0, 0, 0.85) !important;">正常输出：</font>`<font style="color:rgb(0, 0, 0);">/capture_photo</font>`<font style="color:rgba(0, 0, 0, 0.85) !important;">（说明服务已成功注册，可调用）。</font>

## Turtlebot4
### 0.加载环境（初始化）
#### 启动ros2
```plain
source /opt/ros/humble/setup.bash
```

```plain
source install/setup.bash
```

##### 1️⃣ `/opt/ros/humble/setup.bash`
+ 这是 **系统安装的 ROS2 Humble 基础环境**。
+ 它包含：
    - ROS2 的命令行工具 (`ros2`, `colcon`, `ros2 topic` 等)
    - 系统自带的包（比如 `rclcpp`, `sensor_msgs`）
    - 基础环境变量（`ROS_DISTRO`, `AMENT_PREFIX_PATH` 等）
+ **作用**：先让 shell 能识别 ROS2 基础命令和系统包。

---

##### 2️⃣ `~/ros2_ws/install/setup.bash`
+ 这是你 **自定义工作空间 build 后的环境**。
+ 它包含：
    - 你自己编译的 package（比如 `depthai_examples`、`depthai_bridge`）
    - 将你的工作空间加入到 `AMENT_PREFIX_PATH`，覆盖系统默认包（如果有同名包）
+ **作用**：让 shell 能找到你自己编译的包，并覆盖系统包（如果必要）。

---

##### 3️⃣ 正确顺序
通常做法：

```plain
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

+ 先加载系统 ROS2，保证基础工具可用。
+ 再加载你自己的工作空间，保证自定义包被找到。

注意顺序不能反，先加载工作空间再加载系统 ROS 会导致你的自定义包可能被系统包覆盖。

---

##### 4️⃣ 小技巧
+ 每次打开新的 terminal，建议用：

```plain
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

+ 可以把它加到 `~/.bashrc`：

```plain
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

这样每次开新的 shell 就自动生效。

### 1.启动程序
```bash
ros2 launch depthai_examples photo_capture.launch.py
```

