#ifndef DEPTHAI_EXAMPLES_PHOTO_SAVE_NODE_HPP_
#define DEPTHAI_EXAMPLES_PHOTO_SAVE_NODE_HPP_

// 1. 包含必需的系统/ROS/第三方库头文件（对外暴露时需明确依赖）
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <string>

namespace depthai_examples {  // 用命名空间封装，避免命名冲突

/**
 * @class ImageSaverNode
 * @brief 图像保存节点类（对外暴露可复用接口）
 * 功能：订阅 sensor_msgs/msg/Image 话题，将图像保存到本地目录
 * 可复用场景：其他程序可直接创建该类对象，或继承扩展功能
 */
class ImageSaverNode : public rclcpp::Node {
public:
    /**
     * @brief 构造函数（支持自定义参数，提高复用灵活性）
     * @param node_name 节点名称（默认：image_saver_node）
     * @param image_topic 订阅的图像话题（默认：color/preview/image）
     * @param save_directory 图像保存目录（默认：~/received_images）
     */
    explicit ImageSaverNode(
        const std::string& node_name = "image_saver_node",
        const std::string& image_topic = "color/preview/image",
        const std::string& save_directory = "~/received_images"
    );

    /**
     * @brief 析构函数（释放资源，确保优雅退出）
     */
    ~ImageSaverNode() = default;

    // -------------------------- 对外暴露的公共接口 --------------------------
    /**
     * @brief 设置新的图像保存目录
     * @param new_dir 新的保存目录路径（绝对路径或相对路径均可）
     * @return bool：true=目录设置成功（或已存在），false=目录创建失败
     */
    bool set_save_directory(const std::string& new_dir);

    /**
     * @brief 获取当前的图像保存目录
     * @return std::string：当前保存目录的绝对路径
     */
    std::string get_current_save_directory() const;

    /**
     * @brief 手动触发一次图像保存（支持外部程序主动调用）
     * @param img_msg 待保存的 ROS 图像消息
     * @return bool：true=保存成功，false=保存失败
     */
    bool save_image_manually(const sensor_msgs::msg::Image::SharedPtr img_msg);

private:
    // -------------------------- 内部私有成员（不对外暴露） --------------------------
    /**
     * @brief 创建保存目录（若不存在）
     * @param dir 目标目录路径
     * @return bool：true=创建成功/已存在，false=创建失败
     */
    bool create_save_directory_(const std::string& dir);

    /**
     * @brief 图像话题回调函数（订阅到图像时自动触发保存）
     * @param msg 订阅到的 ROS 图像消息
     */
    void image_callback_(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief 生成唯一的图像文件名（基于时间戳，避免覆盖）
     * @return std::string：完整的图像文件名（含路径）
     */
    std::string generate_unique_filename_() const;

    // 私有成员变量（通过公共接口访问，不直接暴露）
    std::string image_topic_;          // 当前订阅的图像话题
    std::string current_save_dir_;     // 当前的图像保存目录（绝对路径）
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscriber_;  // 图像订阅者
};

}  // namespace depthai_examples

#endif  // DEPTHAI_EXAMPLES_PHOTO_SAVE_NODE_HPP_