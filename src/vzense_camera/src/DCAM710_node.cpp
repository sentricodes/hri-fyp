#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cstdint>
#include <memory>
#include <chrono>
#include <functional>
#include <vector>
#include <cstring>
#include "DCAM710/Vzense_api_710.h"

class VzenseCameraNode : public rclcpp::Node
{
public:
  VzenseCameraNode() : Node("DCAM710_node")
  {
    RCLCPP_INFO(this->get_logger(), "Starting Vzense camera node...");

    if (!initialize_sdk()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize SDK.");
      return;
    }

    if (!open_first_device()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open device.");
      return;
    }

    if (!start_stream()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start stream.");
      return;
    }

    if (!enable_mapped_depth()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to enable mapped depth.");
      return;
    }

    if (!set_rgb_resolution()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set RGB resolution.");
      return;
    }

    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/vzense/depth/image_raw", 10);

    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/vzense/rgb/image_raw", 10);

    depth_registered_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/vzense/depth_registered/image_raw", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&VzenseCameraNode::capture_frame, this));

    RCLCPP_INFO(this->get_logger(), "Vzense camera node ready.");
  }

  ~VzenseCameraNode()
  {
    cleanup();
  }

private:
  bool initialize_sdk()
  {
    PsReturnStatus status = Ps2_Initialize();
    if (status != PsRetOK) {
      RCLCPP_ERROR(this->get_logger(), "Ps2_Initialize failed: %d", status);
      return false;
    }

    sdk_initialized_ = true;
    return true;
  }

  bool set_rgb_resolution()
  {
    PsResolution resolution = PsRGB_Resolution_640_480;
    PsReturnStatus status = Ps2_SetRGBResolution(device_handle_, session_index_, resolution);

    if (status != PsRetOK) {
      RCLCPP_ERROR(this->get_logger(), "Ps2_SetRGBResolution failed: %d", status);
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "RGB resolution set to 640x480.");
    return true;
  }

  bool open_first_device()
  {
    uint32_t device_count = 0;
    PsReturnStatus status = Ps2_GetDeviceCount(&device_count);
    if (status != PsRetOK) {
      RCLCPP_ERROR(this->get_logger(), "Ps2_GetDeviceCount failed: %d", status);
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Detected devices: %u", device_count);

    if (device_count == 0) {
      RCLCPP_WARN(this->get_logger(), "No Vzense device connected.");
      return false;
    }

    std::unique_ptr<PsDeviceInfo[]> device_list(new PsDeviceInfo[device_count]);

    status = Ps2_GetDeviceListInfo(device_list.get(), device_count);
    if (status != PsRetOK) {
      RCLCPP_ERROR(this->get_logger(), "Ps2_GetDeviceListInfo failed: %d", status);
      return false;
    }

    if (device_list[0].status != Connected) {
      RCLCPP_ERROR(this->get_logger(), "First device is not in Connected state: %d", device_list[0].status);
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Opening device: uri=%s alias=%s",
      device_list[0].uri, device_list[0].alias);

    status = Ps2_OpenDevice(device_list[0].uri, &device_handle_);
    if (status != PsRetOK) {
      RCLCPP_ERROR(this->get_logger(), "Ps2_OpenDevice failed: %d", status);
      return false;
    }

    device_open_ = true;
    return true;
  }

  bool start_stream()
  {
    PsReturnStatus status = Ps2_StartStream(device_handle_, session_index_);
    if (status != PsRetOK) {
      RCLCPP_ERROR(this->get_logger(), "Ps2_StartStream failed: %d", status);
      return false;
    }

    stream_started_ = true;
    return true;
  }

  bool enable_mapped_depth()
  {
    bool enable = true;
    PsReturnStatus status = Ps2_SetMapperEnabledRGBToDepth(
      device_handle_, session_index_, enable);

    if (status != PsRetOK) {
      RCLCPP_ERROR(this->get_logger(),
        "Ps2_SetMapperEnabledRGBToDepth failed: %d", status);
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Mapped depth in RGB space enabled.");
    return true;
  }

void capture_frame()
{
  PsFrameReady frame_ready{};
  PsReturnStatus status = Ps2_ReadNextFrame(device_handle_, session_index_, &frame_ready);
  if (status != PsRetOK) {
    RCLCPP_WARN(this->get_logger(), "Ps2_ReadNextFrame failed: %d", status);
    return;
  }

  if (frame_ready.depth == 1) {
    PsFrame depth_frame{};
    status = Ps2_GetFrame(device_handle_, session_index_, PsDepthFrame, &depth_frame);

    if (status == PsRetOK && depth_frame.pFrameData != nullptr) {
      sensor_msgs::msg::Image msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = "vzense_depth_frame";

      msg.height = depth_frame.height;
      msg.width = depth_frame.width;
      msg.encoding = "mono16";
      msg.is_bigendian = false;
      msg.step = depth_frame.width * 2;

      const size_t data_size = depth_frame.dataLen;
      msg.data.resize(data_size);
      std::memcpy(msg.data.data(), depth_frame.pFrameData, data_size);

      depth_pub_->publish(msg);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Published depth frame: index=%u size=%u %ux%u",
        depth_frame.frameIndex,
        depth_frame.dataLen,
        depth_frame.width,
        depth_frame.height);
    }
  }

  if (frame_ready.rgb == 1) {
    PsFrame rgb_frame{};
    status = Ps2_GetFrame(device_handle_, session_index_, PsRGBFrame, &rgb_frame);

    if (status == PsRetOK && rgb_frame.pFrameData != nullptr) {
      sensor_msgs::msg::Image msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = "vzense_rgb_frame";

      msg.height = rgb_frame.height;
      msg.width = rgb_frame.width;

      // BRG8
      msg.encoding = "bgr8";
      msg.is_bigendian = false;
      msg.step = rgb_frame.width * 3;

      const size_t data_size = rgb_frame.dataLen;
      msg.data.resize(data_size);
      std::memcpy(msg.data.data(), rgb_frame.pFrameData, data_size);

      rgb_pub_->publish(msg);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Published RGB frame: index=%u size=%u %ux%u pixelFormat=%d",
        rgb_frame.frameIndex,
        rgb_frame.dataLen,
        rgb_frame.width,
        rgb_frame.height,
        rgb_frame.pixelFormat);
    }
  }

  if (frame_ready.mappedDepth == 1) {
  PsFrame mapped_depth_frame{};
  status = Ps2_GetFrame(device_handle_, session_index_, PsMappedDepthFrame, &mapped_depth_frame);

  if (status == PsRetOK && mapped_depth_frame.pFrameData != nullptr) {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "vzense_depth_registered_frame";

    msg.height = mapped_depth_frame.height;
    msg.width = mapped_depth_frame.width;
    msg.encoding = "mono16";
    msg.is_bigendian = false;
    msg.step = mapped_depth_frame.width * 2;

    const size_t data_size = mapped_depth_frame.dataLen;
    msg.data.resize(data_size);
    std::memcpy(msg.data.data(), mapped_depth_frame.pFrameData, data_size);

    depth_registered_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Published mapped depth frame: index=%u size=%u %ux%u",
      mapped_depth_frame.frameIndex,
      mapped_depth_frame.dataLen,
      mapped_depth_frame.width,
      mapped_depth_frame.height);
  }
}
}

  void cleanup()
  {
    if (stream_started_) {
      PsReturnStatus status = Ps2_StopStream(device_handle_, session_index_);
      if (status != PsRetOK) {
        RCLCPP_WARN(this->get_logger(), "Ps2_StopStream failed: %d", status);
      }
      stream_started_ = false;
    }

    if (device_open_) {
      PsReturnStatus status = Ps2_CloseDevice(&device_handle_);
      if (status != PsRetOK) {
        RCLCPP_WARN(this->get_logger(), "Ps2_CloseDevice failed: %d", status);
      }
      device_open_ = false;
    }

    if (sdk_initialized_) {
      PsReturnStatus status = Ps2_Shutdown();
      if (status != PsRetOK) {
        RCLCPP_WARN(this->get_logger(), "Ps2_Shutdown failed: %d", status);
      }
      sdk_initialized_ = false;
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_registered_pub_;

  PsDeviceHandle device_handle_ = 0;
  uint32_t session_index_ = 0;

  bool sdk_initialized_ = false;
  bool device_open_ = false;
  bool stream_started_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VzenseCameraNode>());
  rclcpp::shutdown();
  return 0;
}