cmake_minimum_required(VERSION 3.10.2)
project(depthai_examples VERSION 2.11.2 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

# OpenCV 依赖
set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()

# ROS 2 基础依赖
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 核心依赖列表
set(dependencies
  cv_bridge
  depthai_bridge
  rclcpp
  sensor_msgs
)

# -------------------------- 关键修改：添加头文件目录 --------------------------
# 让编译器能找到 include/depthai_examples 下的 .hpp 文件
include_directories(
  include  # 新增：对应 include/depthai_examples 目录
  ${ament_INCLUDE_DIRS}
)

# 节点编译宏（不变）
macro(dai_add_node_ros2 node_name node_src)
  add_executable("${node_name}" "${node_src}")
  target_link_libraries("${node_name}"
                        depthai::core
                        opencv_imgproc
                        opencv_highgui)
  ament_target_dependencies("${node_name}"
                            ${dependencies})
endmacro()

# 编译图像保存节点（不变）
dai_add_node_ros2(photo_save_node src/photo_save_node.cpp)

# -------------------------- 新增：安装头文件（供其他程序引用） --------------------------
install(DIRECTORY include/
        DESTINATION include/
        FILES_MATCHING PATTERN "*.hpp")  # 仅安装 .hpp 头文件

# 安装 launch 文件与节点（修改后：匹配实际文件名 photo_save.launch.py）
install(DIRECTORY launch/ 
        DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "photo_save.launch.py")  # 关键：把“_”改成“.”

install(TARGETS
        photo_save_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()