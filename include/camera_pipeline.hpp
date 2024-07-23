#ifndef CAMERA_PIPELINE_HPP
#define CAMERA_PIPELINE_HPP

#include <gstreamermm.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <functional>

class CameraPipeline {
public:
    using FrameCallback = std::function<void(const sensor_msgs::msg::Image::SharedPtr&)>;

    CameraPipeline(const std::string& device
                    , FrameCallback callback
                    , rclcpp::Logger logger
                    , const int width
                    , const int height);

    ~CameraPipeline();

    void start();
    void stop();

private:
    bool on_bus_message(const Glib::RefPtr<Gst::Bus>&, const Glib::RefPtr<Gst::Message>& message);
    Gst::FlowReturn on_new_sample();

    void init(const std::string& device
                            , const int width
                            , const int height
                            );

    rclcpp::Logger logger_;

    const int idx_;
    Glib::RefPtr<Gst::Pipeline> pipeline_;
    Glib::RefPtr<Gst::Bus> bus_;
    Glib::RefPtr<Gst::Element> cam_;
    Glib::RefPtr<Gst::Element> nvvidconv_;
    Glib::RefPtr<Gst::Element> viddeoconvert_;
    Glib::RefPtr<Gst::AppSink> appsink_;
    Glib::RefPtr<Gst::Caps> nvvidconv_caps_;
    Glib::RefPtr<Gst::CapsFilter> nvvidconv_capsfilter_;
    Glib::RefPtr<Gst::Caps> viddeoconvert_caps_;
    Glib::RefPtr<Gst::CapsFilter> viddeoconvert_capsfilter_;
    FrameCallback frame_callback_;
};

#endif // CAMERA_PIPELINE_HPP
