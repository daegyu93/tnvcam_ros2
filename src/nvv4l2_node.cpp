#include <rclcpp/rclcpp.hpp>
#include "camera_pipeline.hpp"
#include "nvv4l2_node.hpp"

using namespace ros2_nvv4l2;

NvV4l2Node::NvV4l2Node(const rclcpp::NodeOptions & node_options) :
    Node("nvv4l2_node")
{
    RCLCPP_INFO(get_logger(), "[NvV4l2Node] start.");
    init();
}

NvV4l2Node::~NvV4l2Node()
{
    RCLCPP_INFO(get_logger(), "[NvV4l2Node] pipeline stop().");
    camera_pipeline_->stop();
}

void NvV4l2Node::init()
{
    std::string node_name = get_name();
    RCLCPP_INFO(get_logger(), "[NvV4l2Node] %s.", node_name.c_str());
    camera_id_ = declare_parameter<int>("camera_id", 0);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/image_raw", 1);

    std::string device = "/dev/video" + std::to_string(camera_id_);
    RCLCPP_INFO(get_logger(), "[NvV4l2Node] %s pipeline start.", device.c_str());

    camera_pipeline_ = std::make_unique<CameraPipeline>(device
        , std::bind(&NvV4l2Node::publish_frame, this, std::placeholders::_1)
        , get_logger()
        , 1920
        , 1080);
    
    camera_pipeline_->start();

    RCLCPP_INFO(get_logger(), "[NvV4l2Node] pipeline start().");
}


void NvV4l2Node::publish_frame(const sensor_msgs::msg::Image::SharedPtr& msg) 
{
    image_pub_->publish(*msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NvV4l2Node>());
    rclcpp::shutdown();
    return 0;
}

