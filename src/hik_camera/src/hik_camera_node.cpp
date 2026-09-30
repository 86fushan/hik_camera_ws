#include <chrono>
#include <memory>
#include <iostream>
#include <vector>
#include <cstring>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "MvCameraControl.h"

using namespace std::chrono_literals;

class HikCameraNode : public rclcpp::Node
{
public:
    HikCameraNode()
        : Node("hik_camera")
    {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/image_raw", 10);

        if (!init_camera())
        {
            RCLCPP_ERROR(this->get_logger(), "Camera initialization failed.");
            return;
        }

        /*
         * 参数默认值使用相机当前实际值。
         * 这样启动节点时不会擅自修改相机参数。
         */

        MVCC_FLOATVALUE stFloatValue = {};

        // ExposureTime
        double current_exposure = 0.0;
        if (MV_CC_GetFloatValue(
                camera_handle_,
                "ExposureTime",
                &stFloatValue) == MV_OK)
        {
            current_exposure = stFloatValue.fCurValue;
        }

        // Gain
        double current_gain = 0.0;
        if (MV_CC_GetFloatValue(
                camera_handle_,
                "Gain",
                &stFloatValue) == MV_OK)
        {
            current_gain = stFloatValue.fCurValue;
        }

        // AcquisitionFrameRate
        double current_frame_rate = 0.0;
        if (MV_CC_GetFloatValue(
                camera_handle_,
                "AcquisitionFrameRate",
                &stFloatValue) == MV_OK)
        {
            current_frame_rate = stFloatValue.fCurValue;
        }

        // PixelFormat
        MVCC_ENUMVALUE stEnumValue = {};
        int64_t current_pixel_format = 0;

        if (MV_CC_GetEnumValue(
                camera_handle_,
                "PixelFormat",
                &stEnumValue) == MV_OK)
        {
            current_pixel_format =
                static_cast<int64_t>(stEnumValue.nCurValue);
        }

        /*
         * 声明 ROS 2 参数
         */
        this->declare_parameter<double>(
            "exposure_time",
            current_exposure);

        this->declare_parameter<double>(
            "gain",
            current_gain);

        this->declare_parameter<double>(
            "frame_rate",
            current_frame_rate);

        this->declare_parameter<int64_t>(
            "pixel_format",
            current_pixel_format);

        /*
         * 读取 ROS 2 参数
         */
        double exposure_time =
            this->get_parameter("exposure_time").as_double();

        double gain =
            this->get_parameter("gain").as_double();

        double frame_rate =
            this->get_parameter("frame_rate").as_double();

        int64_t pixel_format =
            this->get_parameter("pixel_format").as_int();

        /*
         * 应用参数
         */
        if (!set_camera_parameters(
                exposure_time,
                gain,
                frame_rate,
                pixel_format))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Some camera parameters failed to apply.");
        }

        if (!start_camera())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to start camera grabbing.");
            return;
        }

        /*
         * 参数动态修改回调
         */
        parameter_callback_handle_ =
            this->add_on_set_parameters_callback(
                std::bind(
                    &HikCameraNode::parameters_callback,
                    this,
                    std::placeholders::_1));

        /*
         * 100 Hz timer。
         * GetImageBuffer 本身会等待图像，因此实际发布频率
         * 由相机采集速度决定。
         */
        timer_ = this->create_wall_timer(
            10ms,
            std::bind(
                &HikCameraNode::timer_callback,
                this));

        RCLCPP_INFO(
            this->get_logger(),
            "Hik camera node started.");

        RCLCPP_INFO(
            this->get_logger(),
            "ExposureTime: %.3f us",
            exposure_time);

        RCLCPP_INFO(
            this->get_logger(),
            "Gain: %.3f dB",
            gain);

        RCLCPP_INFO(
            this->get_logger(),
            "AcquisitionFrameRate: %.3f fps",
            frame_rate);

        RCLCPP_INFO(
            this->get_logger(),
            "PixelFormat enum value: %ld",
            static_cast<long>(pixel_format));
    }

    ~HikCameraNode()
    {
        stop_camera();
    }

private:

    bool init_camera()
    {
        MV_CC_DEVICE_INFO_LIST device_list = {};
        int nRet = MV_CC_EnumDevices(
            MV_USB_DEVICE | MV_GIGE_DEVICE,
            &device_list);

        if (nRet != MV_OK)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "MV_CC_EnumDevices failed: 0x%x",
                nRet);
            return false;
        }

        if (device_list.nDeviceNum == 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "No Hikvision camera found.");
            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Found %u camera(s).",
            device_list.nDeviceNum);

        MV_CC_DEVICE_INFO* device_info =
            device_list.pDeviceInfo[0];

        /*
         * 打印设备基本信息
         */
        if (device_info->nTLayerType == MV_USB_DEVICE)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Camera type: USB");
        }
        else if (device_info->nTLayerType == MV_GIGE_DEVICE)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Camera type: GigE");
        }

        /*
         * Create Handle
         */
        nRet = MV_CC_CreateHandle(
            &camera_handle_,
            device_info);

        if (nRet != MV_OK)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "MV_CC_CreateHandle failed: 0x%x",
                nRet);
            camera_handle_ = nullptr;
            return false;
        }

        /*
         * Open Device
         */
        nRet = MV_CC_OpenDevice(
            camera_handle_);

        if (nRet != MV_OK)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "MV_CC_OpenDevice failed: 0x%x",
                nRet);

            MV_CC_DestroyHandle(camera_handle_);
            camera_handle_ = nullptr;

            return false;
        }

        /*
         * 读取 Width
         */
        MVCC_INTVALUE stWidth = {};
        nRet = MV_CC_GetIntValue(
            camera_handle_,
            "Width",
            &stWidth);

        if (nRet == MV_OK)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Width: %u",
                stWidth.nCurValue);
        }

        /*
         * 读取 Height
         */
        MVCC_INTVALUE stHeight = {};
        nRet = MV_CC_GetIntValue(
            camera_handle_,
            "Height",
            &stHeight);

        if (nRet == MV_OK)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Height: %u",
                stHeight.nCurValue);
        }

        /*
         * 读取 PixelFormat
         */
        MVCC_ENUMVALUE stPixelFormat = {};

        nRet = MV_CC_GetEnumValue(
            camera_handle_,
            "PixelFormat",
            &stPixelFormat);

        if (nRet == MV_OK)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Current PixelFormat: %ld",
                static_cast<long>(stPixelFormat.nCurValue));
        }

        bool frame_rate_enable = false;

        nRet = MV_CC_GetBoolValue(
            camera_handle_,
            "AcquisitionFrameRateEnable",
            &frame_rate_enable);

        if (nRet == MV_OK)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "AcquisitionFrameRateEnable = %s",
                frame_rate_enable ? "true" : "false");
        }
        else
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Get AcquisitionFrameRateEnable failed! nRet [0x%X]",
                nRet);
        }

        return true;
    }

    bool start_camera()
    {
        if (camera_handle_ == nullptr)
        {
            return false;
        }

        const int nRet = MV_CC_StartGrabbing(
            camera_handle_);

        if (nRet != MV_OK)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "MV_CC_StartGrabbing failed: 0x%x",
                nRet);
            return false;
        }

        grabbing_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Camera grabbing started.");

        return true;
    }

    bool set_camera_parameters(
        double exposure_time,
        double gain,
        double frame_rate,
        int64_t pixel_format)
    {
        if (camera_handle_ == nullptr)
        {
            return false;
        }

        bool success = true;

        int nRet;

        /*
         * 关闭自动曝光
         * 官方 SetParam / BasicDemo 已确认
         */
        nRet = MV_CC_SetEnumValue(
            camera_handle_,
            "ExposureAuto",
            0);

        if (nRet != MV_OK)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Set ExposureAuto failed: 0x%x",
                nRet);
            success = false;
        }

        /*
         * ExposureTime
         */
        nRet = MV_CC_SetFloatValue(
            camera_handle_,
            "ExposureTime",
            static_cast<float>(exposure_time));

        if (nRet != MV_OK)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Set ExposureTime failed: 0x%x",
                nRet);
            success = false;
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "ExposureTime set to %.3f",
                exposure_time);
        }

        /*
         * 关闭自动增益
         * 官方 BasicDemo 已确认
         */
        nRet = MV_CC_SetEnumValue(
            camera_handle_,
            "GainAuto",
            0);

        if (nRet != MV_OK)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Set GainAuto failed: 0x%x",
                nRet);
            success = false;
        }

        /*
         * Gain
         */
        nRet = MV_CC_SetFloatValue(
            camera_handle_,
            "Gain",
            static_cast<float>(gain));

        if (nRet != MV_OK)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Set Gain failed: 0x%x",
                nRet);
            success = false;
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Gain set to %.3f",
                gain);
        }

        /*
         * AcquisitionFrameRate
         */
        const bool enable_frame_rate = true;

        nRet = MV_CC_SetBoolValue(
            camera_handle_,
            "AcquisitionFrameRateEnable",
            enable_frame_rate);

        if (nRet != MV_OK)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Set AcquisitionFrameRateEnable failed: 0x%x",
                nRet);
            success = false;
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "AcquisitionFrameRateEnable = true");
        }

        nRet = MV_CC_SetFloatValue(
            camera_handle_,
            "AcquisitionFrameRate",
            static_cast<float>(frame_rate));

        if (nRet != MV_OK)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Set AcquisitionFrameRate failed: 0x%x",
                nRet);
            success = false;
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "AcquisitionFrameRate set to %.3f",
                frame_rate);
        }

        /*
         * PixelFormat
         *
         * 这里使用相机当前读取到的枚举值。
         * 如果 ROS 参数改变了它，则尝试设置。
         */
        MVCC_ENUMVALUE current_pixel_format = {};

        nRet = MV_CC_GetEnumValue(
            camera_handle_,
            "PixelFormat",
            &current_pixel_format);

        if (nRet == MV_OK)
        {
            if (static_cast<int64_t>(
                    current_pixel_format.nCurValue) != pixel_format)
            {
                /*
                 * PixelFormat 修改可能要求停止取流。
                 */
                if (grabbing_)
                {
                    nRet = MV_CC_StopGrabbing(
                        camera_handle_);

                    if (nRet == MV_OK)
                    {
                        grabbing_ = false;
                    }
                    else
                    {
                        RCLCPP_WARN(
                            this->get_logger(),
                            "Stop grabbing for PixelFormat failed: 0x%x",
                            nRet);
                        success = false;
                    }
                }

                if (success)
                {
                    nRet = MV_CC_SetEnumValue(
                        camera_handle_,
                        "PixelFormat",
                        static_cast<unsigned int>(
                            pixel_format));

                    if (nRet != MV_OK)
                    {
                        RCLCPP_WARN(
                            this->get_logger(),
                            "Set PixelFormat failed: 0x%x",
                            nRet);
                        success = false;
                    }
                    else
                    {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "PixelFormat set to %ld",
                            static_cast<long>(pixel_format));
                    }
                }

                /*
                 * 恢复取流
                 */
                if (!grabbing_ && camera_handle_ != nullptr)
                {
                    nRet = MV_CC_StartGrabbing(
                        camera_handle_);

                    if (nRet == MV_OK)
                    {
                        grabbing_ = true;
                    }
                    else
                    {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "Restart grabbing failed: 0x%x",
                            nRet);
                        success = false;
                    }
                }
            }
        }

        return success;
    }

    rcl_interfaces::msg::SetParametersResult
    parameters_callback(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        std::lock_guard<std::mutex> lock(parameter_mutex_);

        auto result =
            rcl_interfaces::msg::SetParametersResult();

        result.successful = true;
        result.reason = "success";

        for (const auto& parameter : parameters)
        {
            if (parameter.get_name() == "exposure_time")
            {
                double value = parameter.as_double();

                int nRet = MV_CC_SetEnumValue(
                    camera_handle_,
                    "ExposureAuto",
                    0);

                if (nRet != MV_OK)
                {
                    result.successful = false;
                    result.reason =
                        "Failed to disable ExposureAuto";
                    return result;
                }

                nRet = MV_CC_SetFloatValue(
                    camera_handle_,
                    "ExposureTime",
                    static_cast<float>(value));

                if (nRet != MV_OK)
                {
                    result.successful = false;
                    result.reason =
                        "Failed to set ExposureTime";
                    return result;
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "Runtime ExposureTime = %.3f",
                    value);
            }
            else if (parameter.get_name() == "gain")
            {
                double value = parameter.as_double();

                int nRet = MV_CC_SetEnumValue(
                    camera_handle_,
                    "GainAuto",
                    0);

                if (nRet != MV_OK)
                {
                    result.successful = false;
                    result.reason =
                        "Failed to disable GainAuto";
                    return result;
                }

                nRet = MV_CC_SetFloatValue(
                    camera_handle_,
                    "Gain",
                    static_cast<float>(value));

                if (nRet != MV_OK)
                {
                    result.successful = false;
                    result.reason =
                        "Failed to set Gain";
                    return result;
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "Runtime Gain = %.3f",
                    value);
            }
            else if (parameter.get_name() == "frame_rate")
            {
                double value = parameter.as_double();

                const bool enable_frame_rate = true;

                int nRet = MV_CC_SetBoolValue(
                    camera_handle_,
                    "AcquisitionFrameRateEnable",
                    enable_frame_rate);

                if (nRet != MV_OK)
                {
                    result.successful = false;
                    result.reason =
                        "Failed to enable AcquisitionFrameRate";
                    return result;
                }

                nRet = MV_CC_SetFloatValue(
                    camera_handle_,
                    "AcquisitionFrameRate",
                    static_cast<float>(value));

                if (nRet != MV_OK)
                {
                    result.successful = false;
                    result.reason =
                        "Failed to set AcquisitionFrameRate";
                    return result;
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "Runtime AcquisitionFrameRate = %.3f",
                    value);
            }
            else if (parameter.get_name() == "pixel_format")
            {
                int64_t value = parameter.as_int();

                bool was_grabbing = grabbing_;

                if (was_grabbing)
                {
                    int nRet = MV_CC_StopGrabbing(
                        camera_handle_);

                    if (nRet != MV_OK)
                    {
                        result.successful = false;
                        result.reason =
                            "Failed to stop grabbing for PixelFormat";
                        return result;
                    }

                    grabbing_ = false;
                }

                int nRet = MV_CC_SetEnumValue(
                    camera_handle_,
                    "PixelFormat",
                    static_cast<unsigned int>(value));

                if (nRet != MV_OK)
                {
                    if (was_grabbing)
                    {
                        MV_CC_StartGrabbing(camera_handle_);
                        grabbing_ = true;
                    }

                    result.successful = false;
                    result.reason =
                        "Failed to set PixelFormat";
                    return result;
                }

                if (was_grabbing)
                {
                    nRet = MV_CC_StartGrabbing(
                        camera_handle_);

                    if (nRet != MV_OK)
                    {
                        grabbing_ = false;
                        result.successful = false;
                        result.reason =
                            "Failed to restart grabbing";
                        return result;
                    }

                    grabbing_ = true;
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "Runtime PixelFormat = %ld",
                    static_cast<long>(value));
            }
        }

        return result;
    }

    void timer_callback()
    {
        if (camera_handle_ == nullptr || !grabbing_)
        {
            return;
        }

        MV_FRAME_OUT stImageInfo = {};

        int nRet = MV_CC_GetImageBuffer(
            camera_handle_,
            &stImageInfo,
            1000);

        if (nRet != MV_OK)
        {
            return;
        }

        MVCC_FLOATVALUE stResultingFrameRate = {};

        //int fps_ret = MV_CC_GetFloatValue(
         //   camera_handle_,
          //  "ResultingFrameRate",
           // &stResultingFrameRate);
//             if (fps_ret == MV_OK)
// {
//     RCLCPP_INFO(
//         this->get_logger(),
//         "ResultingFrameRate: %.2f FPS",
//         stResultingFrameRate.fCurValue);
// }
// else
// {
//     RCLCPP_WARN(
//         this->get_logger(),
//         "Get ResultingFrameRate failed! nRet [0x%x]",
//         fps_ret);
// }

    
        const unsigned int width =
            stImageInfo.stFrameInfo.nExtendWidth;

        const unsigned int height =
            stImageInfo.stFrameInfo.nExtendHeight;

        /*
         * RGB8 输出缓冲区
         */
        std::vector<unsigned char> rgb_buffer(
            static_cast<size_t>(width) *
            static_cast<size_t>(height) *
            4 +
            2048);

        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {};

        stConvertParam.nWidth = width;
        stConvertParam.nHeight = height;
        stConvertParam.pSrcData =
            stImageInfo.pBufAddr;

        stConvertParam.nSrcDataLen =
            stImageInfo.stFrameInfo.nFrameLenEx;

        stConvertParam.enSrcPixelType =
            stImageInfo.stFrameInfo.enPixelType;

        stConvertParam.enDstPixelType =
            PixelType_Gvsp_RGB8_Packed;

        stConvertParam.pDstBuffer =
            rgb_buffer.data();

        stConvertParam.nDstBufferSize =
            static_cast<unsigned int>(
                rgb_buffer.size());

        nRet = MV_CC_ConvertPixelType(
            camera_handle_,
            &stConvertParam);

        if (nRet == MV_OK)
        {
            auto msg =
                sensor_msgs::msg::Image();

            msg.header.stamp =
                this->get_clock()->now();

            msg.header.frame_id =
                "camera_frame";

            msg.height = height;
            msg.width = width;

            msg.encoding = "rgb8";
            msg.is_bigendian = 0;
            msg.step = width * 3;

            msg.data.resize(
                stConvertParam.nDstLen);

            std::memcpy(
                msg.data.data(),
                rgb_buffer.data(),
                stConvertParam.nDstLen);

            image_pub_->publish(msg);
        }

        /*
         * 官方 API：
         * GetImageBuffer 后必须 FreeImageBuffer
         */
        MV_CC_FreeImageBuffer(
            camera_handle_,
            &stImageInfo);
    }

    void stop_camera()
    {
        if (camera_handle_ == nullptr)
        {
            return;
        }

        if (grabbing_)
        {
            MV_CC_StopGrabbing(
                camera_handle_);

            grabbing_ = false;
        }

        MV_CC_CloseDevice(
            camera_handle_);

        MV_CC_DestroyHandle(
            camera_handle_);

        camera_handle_ = nullptr;
    }

private:

    void* camera_handle_ = nullptr;

    bool grabbing_ = false;

    rclcpp::Publisher<
        sensor_msgs::msg::Image>::SharedPtr image_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    OnSetParametersCallbackHandle::SharedPtr
        parameter_callback_handle_;

    std::mutex parameter_mutex_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<HikCameraNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}