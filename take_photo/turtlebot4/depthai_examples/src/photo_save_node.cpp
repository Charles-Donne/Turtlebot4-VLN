#include <chrono>
#include <cstdio>
#include <functional>
#include <iostream>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImageConverter.hpp"

using namespace std::chrono_literals;

std::tuple<dai::Pipeline, std::string> createPipeline() {
    dai::Pipeline pipeline;

    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("video");

    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setVideoSize(1920, 1080);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(30);

    colorCam->video.link(xlinkOut->input);

    return std::make_tuple(pipeline, "video");
}

class PhotoSaveNode : public rclcpp::Node {
public:
    PhotoSaveNode() : Node("photo_save_node") {
        auto pipeline = createPipeline();
        dai::Pipeline daiPipeline;
        std::string streamName;
        std::tie(daiPipeline, streamName) = pipeline;

        device = std::make_unique<dai::Device>(daiPipeline);
        videoQueue = device->getOutputQueue(streamName, 30, false);

        save_service = this->create_service<std_srvs::srv::Trigger>(
            "save_photo",
            std::bind(&PhotoSaveNode::savePhotoCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Photo save node initialized.");
    }

private:
    void savePhotoCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        auto frame = videoQueue->tryGet<dai::ImgFrame>();
        if(frame) {
            dai::rosBridge::ImageConverter converter("camera_frame", true);
            cv::Mat mat = converter.toCvMat(frame);

            std::string filename = "photo_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".jpg";
            cv::imwrite(filename, mat);

            RCLCPP_INFO(this->get_logger(), "Photo saved as %s", filename.c_str());
            response->success = true;
            response->message = "Photo saved successfully.";
        } else {
            RCLCPP_WARN(this->get_logger(), "No frame available in the queue.");
            response->success = false;
            response->message = "Failed to save photo. No frame available.";
        }
    }

    std::unique_ptr<dai::Device> device;
    std::shared_ptr<dai::DataOutputQueue> videoQueue;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PhotoSaveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
