#include "camera_pipeline.hpp"
#include <gst/app/gstappsink.h>
#include <rclcpp/rclcpp.hpp>

CameraPipeline::CameraPipeline(const std::string& device
                            , FrameCallback callback
                            , rclcpp::Logger logger
                            , const int width
                            , const int height
                            )
: frame_callback_(callback)
, logger_(logger)
, idx_(0)
{
    init(device, width, height);
}

void CameraPipeline::init(const std::string& device
                            , const int width
                            , const int height
                            )
{
    RCLCPP_INFO(logger_, " Init CameraPipeline");
    Gst::init();

    pipeline_ = Gst::Pipeline::create();
    if (!pipeline_) {
        throw std::runtime_error("Failed to create pipeline.");
    }
    cam_ = Gst::ElementFactory::create_element("nvv4l2camerasrc");
    nvvidconv_ = Gst::ElementFactory::create_element("nvvidconv");
    viddeoconvert_ = Gst::ElementFactory::create_element("videoconvert");
    appsink_ = Gst::AppSink::create("appsink");

    std::string nvvidconv_caps_str = "video/x-raw, format=RGBA";
    nvvidconv_caps_str += ", width=" + std::to_string(width);
    nvvidconv_caps_str += ", height=" + std::to_string(height);
    
    nvvidconv_caps_ = Gst::Caps::create_from_string(nvvidconv_caps_str);
    nvvidconv_capsfilter_ = Gst::CapsFilter::create("capsfilter_rgba");
    nvvidconv_capsfilter_->set_property("caps", nvvidconv_caps_);

    std::string viddeoconvert_caps_str = R"(
        video/x-raw, format=RGB
    )";
    viddeoconvert_caps_ = Gst::Caps::create_from_string(viddeoconvert_caps_str);
    viddeoconvert_capsfilter_ = Gst::CapsFilter::create("capsfilter_rgb8");
    viddeoconvert_capsfilter_->set_property("caps", viddeoconvert_caps_);

    if (!cam_ || !nvvidconv_ || !viddeoconvert_ || !appsink_) {
        RCLCPP_ERROR(logger_, " Failed to create elements.");
        throw std::runtime_error("Failed to create elements.");
    }

    cam_->set_property("device", device);

    appsink_->property_emit_signals() = true;
    appsink_->set_sync(false);
    appsink_->signal_new_sample().connect(
        sigc::mem_fun(*this, &CameraPipeline::on_new_sample)
    );

    pipeline_->add(cam_)
        ->add(nvvidconv_)->add(nvvidconv_capsfilter_)
        ->add(viddeoconvert_)->add(viddeoconvert_capsfilter_)
        ->add(appsink_);
    cam_->link(nvvidconv_)->link(nvvidconv_capsfilter_)
        ->link(viddeoconvert_)->link(viddeoconvert_capsfilter_)
        ->link(appsink_);
    
    bus_ = pipeline_->get_bus();
    bus_->add_watch(sigc::mem_fun(*this, &CameraPipeline::on_bus_message));
    
}

CameraPipeline::~CameraPipeline() {
    stop();
    RCLCPP_FATAL(logger_, "CameraPipeline terminated.");
}
bool CameraPipeline::on_bus_message(const Glib::RefPtr<Gst::Bus>&, const Glib::RefPtr<Gst::Message>& message)
{
    switch (message->get_message_type())
    {
        case Gst::MESSAGE_ERROR:
            std::cerr << "Error received from element " << message->get_source()->get_name() << std::endl;
            break;

        case Gst::MESSAGE_EOS:
            std::cout << "End of stream" << std::endl;
            break;

        default:
            break;
    }

    return true;
}


void CameraPipeline::start() {
    pipeline_->set_state(Gst::STATE_PLAYING);
}

void CameraPipeline::stop() {
    pipeline_->set_state(Gst::STATE_NULL);
}

Gst::FlowReturn CameraPipeline::on_new_sample() {
    auto sample = appsink_->pull_sample();
    if (!sample) return Gst::FLOW_ERROR;

    auto buffer = sample->get_buffer();
    auto caps = sample->get_caps();
    Gst::MapInfo map;
    if (!buffer->map(map, Gst::MAP_READ)) return Gst::FLOW_ERROR;

    Gst::Structure struct_ = caps->get_structure(0);
    int width=1920, height=1080;
    struct_.get_field("width", width);
    struct_.get_field("height", height);

    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->header.stamp = rclcpp::Clock().now();
    msg->header.frame_id = "tf_camera";
    msg->height = height;
    msg->width = width;
    msg->encoding = "rgb8";
    msg->is_bigendian = false;
    msg->step = width * 3;
    msg->data.assign(map.get_data(), map.get_data() + map.get_size());
    buffer->unmap(map);

    if (frame_callback_) 
    {
        frame_callback_(std::move(msg));
        return Gst::FLOW_OK;
    }
    return Gst::FLOW_ERROR;
}
