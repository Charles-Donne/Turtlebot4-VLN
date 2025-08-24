cmake_minimum_required(VERSION 3.10.2)  # 适配 Ubuntu 18.04 LTS
project(depthai_examples VERSION 2.11.2 LANGUAGES CXX C)

# 编译配置（新增默认 Build Type）
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)  # 保留调试信息（Debug 模式生效）
# 新增：默认 Release 模式，兼顾速度和体积
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# 处理 CMake 政策，确保兼容性
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

# OpenCV 查找（优先 4，若找不到则用 3，避免版本冲突）
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()
message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")

# ROS 2 基础依赖
find_package(ament_cmake REQUIRED)
message(STATUS "------------------------------------------")
message(STATUS "Depthai Bridge is being built using AMENT.")
message(STATUS "------------------------------------------")

# 核心依赖查找（去重，仅保留必要项）
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 可选依赖（按需启用，如立体视觉、视觉消息）
# find_package(stereo_msgs REQUIRED)
# find_package(vision_msgs REQUIRED)

# 定义节点依赖变量（精简，仅包含必要依赖）
set(dependencies
  camera_info_manager
  cv_bridge
  depthai_ros_msgs
  depthai_bridge
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  # stereo_msgs  # 若不使用立体视觉，注释掉
  # vision_msgs   # 若不使用视觉检测消息，注释掉
)

# 自定义宏：统一编译节点（包含链接逻辑，新增头文件路径和统一 OpenCV 链接）
macro(dai_add_node_ros2 node_name node_src)
  add_executable("${node_name}" "${node_src}")
  
  # 新增：显式包含 ROS 2 依赖的头文件路径，避免隐式包含警告
  target_include_directories("${node_name}"
    PUBLIC
    ${ament_INCLUDE_DIRS}
  )
  
  # 修改：用 OpenCV_LIBS 统一链接 OpenCV 库，适配不同版本
  target_link_libraries("${node_name}"
    depthai::core
    ${OpenCV_LIBS}
  )
  
  # 关联 ROS 2 依赖
  ament_target_dependencies("${node_name}"
    ${dependencies}
  )
endmacro()

# 编译节点（通过宏统一管理，避免重复）
dai_add_node_ros2(crop_control_service src/crop_control_service.cpp)
dai_add_node_ros2(rgb_stereo_node src/rgb_stereo_node.cpp)
dai_add_node_ros2(photo_capture_node src/photo_capture_node.cpp)

# 针对特定 ROS 版本的编译选项（如 galactic）
if($ENV{ROS_DISTRO} STREQUAL "galactic")
  target_compile_definitions(rgb_stereo_node PRIVATE IS_GALACTIC)
endif()

# 安装文件与目标
install(DIRECTORY params rviz resources DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# 安装节点可执行文件
install(TARGETS
  crop_control_service
  rgb_stereo_node
  photo_capture_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()