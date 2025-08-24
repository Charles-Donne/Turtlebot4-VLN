// 1. 包含自定义头文件（必须放在最前面，确保声明与实现一致）
#include "depthai_examples/photo_save_node.hpp"

// 2. 包含内部实现依赖的头文件（不对外暴露，仅在源文件中包含）
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <filesystem>
#include <chrono>
#include <cstdlib>  // 用于 expanduser（解析 ~ 为用户主目录）

// 3. 使用命名空间（与头文件一致）
namespace depthai_examples {
namespace fs = std::filesystem;

// -------------------------- 构造函数实现 --------------------------
ImageSaverNode::ImageSaverNode(
    const std::string& node_name,
    const std::string& image_topic,
    const std::string& save_directory
) : Node(node_name),  // 初始化父类 rclcpp::Node
    image_topic_(image_topic) {

    // 解析保存目录（处理 ~ 符号，转为绝对路径）
    std::string expanded_dir = save_directory;
    if (!expanded_dir.empty() && expanded_dir[0] == '~') {
        const char* home = std::getenv("HOME");
        if (home != nullptr) {
            expanded_dir.replace(0, 1, home);
        }
    }
    current_save_dir_ = fs::absolute(expanded_dir).string();

    // 创建保存目录
    if (create_save_directory_(current_save_dir_)) {
        RCLCPP_INFO(this->get_logger(), "Current save directory: %s", current_save_dir_.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize save directory: %s", current_save_dir_.c_str());
    }

    // 创建图像订阅者（订阅话题，触发自动保存）
    img_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        10,  // 消息队列大小
        std::bind(&ImageSaverNode::image_callback_, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "ImageSaverNode initialized (subscribe topic: %s)", image_topic_.c_str());
}

// -------------------------- 公共接口实现 --------------------------
bool ImageSaverNode::set_save_directory(const std::string& new_dir) {
    // 解析新目录（处理 ~ 符号）
    std::string expanded_new_dir = new_dir;
    if (!expanded_new_dir.empty() && expanded_new_dir[0] == '~') {
        const char* home = std::getenv("HOME");
        if (home != nullptr) {
            expanded_new_dir.replace(0, 1, home);
        }
    }
    std::string new_abs_dir = fs::absolute(expanded_new_dir).string();

    // 创建新目录并更新当前目录
    if (create_save_directory_(new_abs_dir)) {
        current_save_dir_ = new_abs_dir;
        RCLCPP_INFO(this->get_logger(), "Save directory updated to: %s", current_save_dir_.c_str());
        return true;
    }
    return false;
}

std::string ImageSaverNode::get_current_save_directory() const {
    return current_save_dir_;
}

bool ImageSaverNode::save_image_manually(const sensor_msgs::msg::Image::SharedPtr img_msg) {
    if (!img_msg) {
        RCLCPP_WARN(this->get_logger(), "Manual save failed: empty image message");
        return false;
    }

    // 调用核心保存逻辑（与回调函数复用同一套逻辑）
    try {
        // ROS 图像 → OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
            img_msg, sensor_msgs::image_encodings::BGR8
        );

        // 生成文件名并保存
        std::string filename = generate_unique_filename_();
        if (cv::imwrite(filename, cv_ptr->image)) {
            RCLCPP_INFO(this->get_logger(), "Manually saved image: %s", filename.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Manual save failed: cannot write to %s", filename.c_str());
            return false;
        }
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Manual save failed (cv_bridge error): %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Manual save failed: %s", e.what());
        return false;
    }
}

// -------------------------- 私有成员函数实现 --------------------------
bool ImageSaverNode::create_save_directory_(const std::string& dir) {
    try {
        if (fs::exists(dir)) {
            if (fs::is_directory(dir)) {
                return true;  // 目录已存在且是合法目录
            } else {
                RCLCPP_ERROR(this->get_logger(), "Path exists but is not a directory: %s", dir.c_str());
                return false;
            }
        }

        // 创建目录（支持多级目录）
        if (fs::create_directories(dir)) {
            RCLCPP_INFO(this->get_logger(), "Created new directory: %s", dir.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s", dir.c_str());
            return false;
        }
    } catch (const fs::filesystem_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Directory error: %s (path: %s)", e.what(), dir.c_str());
        return false;
    }
}

void ImageSaverNode::image_callback_(const sensor_msgs::msg::Image::SharedPtr msg) {
    // 复用手动保存的逻辑，避免代码冗余
    save_image_manually(msg);
}

std::string ImageSaverNode::generate_unique_filename_() const {
    // 基于当前时间戳生成唯一文件名（精确到纳秒，避免并发保存冲突）
    auto timestamp = std::chrono::system_clock::now().time_since_epoch().count();
    return current_save_dir_ + "/" + "received_image_" + std::to_string(timestamp) + ".jpg";
}

}  // namespace depthai_examples

// -------------------------- 独立节点入口（兼容原启动方式） --------------------------
// 保留原有的 main 函数，确保通过 launch 文件启动时仍能正常运行
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // 创建 ImageSaverNode 对象（使用默认参数，可通过 ROS 2 参数重写）
    auto node = std::make_shared<depthai_examples::ImageSaverNode>();
    
    // 自旋运行节点
    rclcpp::spin(node);
    
    // 资源清理
    rclcpp::shutdown();
    return 0;
}
