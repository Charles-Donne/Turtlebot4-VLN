#include <chrono>
#include <cstdio>
#include <functional>
#include <iostream>
#include <string>
#include <memory>
#include <deque>  // 确保包含 deque 头文件（std::deque 依赖）

// ROS 2 核心头文件
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"  // 新增：用于 sensor_msgs::image_encodings::BGR8
#include "std_srvs/srv/trigger.hpp"

// DepthAI 核心头文件（去重，仅保留必要项）
#include "depthai/device/Device.hpp"
#include "depthai/device/DataQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"  // 若不用单目相机可删除，此处保留兼容扩展
#include "depthai/pipeline/node/StereoDepth.hpp"  // 若不用深度图可删除，此处保留兼容扩展
#include "depthai/pipeline/node/XLinkOut.hpp"

// DepthAI-ROS 桥接与 OpenCV 头文件
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/BridgePublisher.hpp"  // 若不用实时发布可删除，此处保留兼容扩展
#include <opencv2/opencv.hpp>  
#include <cv_bridge/cv_bridge.h>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

// 修改：接收参数配置相机
std::tuple<dai::Pipeline, std::string> createPipeline(
    const std::string& color_resolution, 
    int preview_width, 
    int preview_height) {
    
    dai::Pipeline pipeline;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("video");

    // 根据参数设置分辨率
    if(color_resolution == "720p") {
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
        colorCam->setVideoSize(1280, 720);
    } else if(color_resolution == "4k") {
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
        colorCam->setVideoSize(3840, 2160);
    } else {  // 默认1080p
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        colorCam->setVideoSize(1920, 1080);
    }

    // 设置预览尺寸
    colorCam->setPreviewSize(preview_width, preview_height);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(30);

    colorCam->video.link(xlinkOut->input);

    return std::make_tuple(pipeline, "video");
}

class PhotoCaptureNode : public rclcpp::Node {
public:
    PhotoCaptureNode() : Node("photo_capture_node") {
        // 声明并获取参数
        declare_parameters();
        get_parameters();

        // 创建 pipeline 时使用参数
        auto pipeline = createPipeline(color_resolution_, preview_width_, preview_height_);
        dai::Pipeline daiPipeline;
        std::string streamName;
        std::tie(daiPipeline, streamName) = pipeline;

        device = std::make_unique<dai::Device>(daiPipeline);
        videoQueue = device->getOutputQueue(streamName, 30, false);

        // 创建服务
        capture_service = this->create_service<std_srvs::srv::Trigger>(
            "capture_photo",
            std::bind(&PhotoCaptureNode::capturePhotoCallback, this, 
                      std::placeholders::_1, std::placeholders::_2));

        // 创建图像发布者
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            image_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "Photo capture node initialized.");
        RCLCPP_INFO(this->get_logger(), "Saving photos to: %s", save_directory_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing images to: %s", image_topic_.c_str());
    }

private:
    // 声明参数
    void declare_parameters() {
        this->declare_parameter<std::string>("save_directory", 
            fs::path(fs::path(getenv("HOME")) / "oak_photos").string());
        this->declare_parameter<std::string>("color_resolution", "1080p");
        this->declare_parameter<int>("preview_width", 300);
        this->declare_parameter<int>("preview_height", 300);
        this->declare_parameter<std::string>("image_topic", "color/preview/image");
    }

    // 获取参数
    void get_parameters() {
        this->get_parameter("save_directory", save_directory_);
        this->get_parameter("color_resolution", color_resolution_);
        this->get_parameter("preview_width", preview_width_);
        this->get_parameter("preview_height", preview_height_);
        this->get_parameter("image_topic", image_topic_);

        // 创建保存目录
        try {
            if(!fs::exists(save_directory_)) {
                fs::create_directories(save_directory_);
                RCLCPP_INFO(this->get_logger(), "Created save directory: %s", save_directory_.c_str());
            }
        } catch (const fs::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s", e.what());
        }
    }

    void capturePhotoCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        auto frame = videoQueue->tryGet<dai::ImgFrame>();
        if(frame) {
            dai::rosBridge::ImageConverter converter("camera_frame", true);
            std::deque<sensor_msgs::msg::Image> ros_img_queue;
            converter.toRosMsg(frame, ros_img_queue);
            
            if(!ros_img_queue.empty()) {
                sensor_msgs::msg::Image ros_img = ros_img_queue.front();
                // 发布图像
                image_pub_->publish(ros_img);

                // 转换为OpenCV格式并保存
                try {
                    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
                        ros_img, sensor_msgs::image_encodings::BGR8);
                    cv::Mat mat = cv_ptr->image;

                    // 生成唯一文件名
                    auto timestamp = std::chrono::system_clock::now().time_since_epoch().count();
                    std::string filename = save_directory_ + "/" + "photo_" + 
                                          std::to_string(timestamp) + ".jpg";
                    
                    if(cv::imwrite(filename, mat)) {
                        RCLCPP_INFO(this->get_logger(), "Photo saved: %s", filename.c_str());
                        response->success = true;
                        response->message = "Photo saved to: " + filename;
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to save photo to: %s", filename.c_str());
                        response->success = false;
                        response->message = "Failed to save photo";
                    }
                } catch (cv_bridge::Exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
                    response->success = false;
                    response->message = "Image conversion failed";
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Converted image queue is empty");
                response->success = false;
                response->message = "No converted image available";
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "No frame available in queue");
            response->success = false;
            response->message = "No camera frame available";
        }
    }

    // 参数存储
    std::string save_directory_;
    std::string color_resolution_;
    int preview_width_;
    int preview_height_;
    std::string image_topic_;

    // 设备与队列
    std::unique_ptr<dai::Device> device;
    std::shared_ptr<dai::DataOutputQueue> videoQueue;
    
    // 服务与发布者
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PhotoCaptureNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
