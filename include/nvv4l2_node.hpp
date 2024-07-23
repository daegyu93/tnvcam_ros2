#include <rclcpp/rclcpp.hpp>
#include "camera_pipeline.hpp"

namespace ros2_nvv4l2 {
class NvV4l2Node : public rclcpp::Node {
public:
    NvV4l2Node(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()) ;
    ~NvV4l2Node();
private:
    void init();
    void publish_frame(const sensor_msgs::msg::Image::SharedPtr& msg);
    std::unique_ptr<CameraPipeline> camera_pipeline_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    int camera_id_;
};

} // namespace nvv4l2