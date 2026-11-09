#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>
#include <algorithm>

class VrHudNode : public rclcpp::Node {
public:
    VrHudNode() : Node("vr_hud_node") {
        auto video_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        
        image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/image_raw/compressed", video_qos,
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) { image_callback(msg); });

        speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/robot/telemetry/speed", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) { current_speed_ = msg->data; });

        gear_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/robot/telemetry/gear", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) { current_gear_ = msg->data; });
    }

    cv::Mat render_eye(bool is_left, float ipd_center_shift) {
        cv::Mat eye_canvas = cv::Mat::zeros(1200, 1080, CV_8UC3);

        if (latest_frame_.empty()) {
            cv::putText(eye_canvas, "WAITING FOR CAMERA...", cv::Point(150, 600), 
                        cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 3);
            return eye_canvas;
        }

        float scale_w = (1080.0f / latest_frame_.cols) * 1.85f;
        float scale_h = (1200.0f / latest_frame_.rows) * 1.10f;
        float max_scale = std::max(scale_w, scale_h);

        int target_w = static_cast<int>(latest_frame_.cols * max_scale);
        int target_h = static_cast<int>(latest_frame_.rows * max_scale);
        
        cv::Mat resized_camera;
        cv::resize(latest_frame_, resized_camera, cv::Size(target_w, target_h));

        int texture_shift = -50; 
        int start_x = (target_w - 1080) / 2 + (is_left ? texture_shift : -texture_shift);
        int start_y = (target_h - 1200) / 2;

        start_x = std::max(0, std::min(start_x, target_w - 1080));
        start_y = std::max(0, std::min(start_y, target_h - 1200));

        cv::Rect crop_roi(start_x, start_y, 1080, 1200);
        resized_camera(crop_roi).copyTo(eye_canvas);
        
        std::string speed_str = "SPEED: " + cv::format("%.1f", current_speed_) + " km/h";
        std::string gear_str = "GEAR: " + (current_gear_ == 0 ? "R" : (current_gear_ == 1 ? "N" : std::to_string(current_gear_ - 1)));
        
        int hud_x = is_left ? 180 : 120; 
        cv::putText(eye_canvas, speed_str, cv::Point(hud_x, 240), cv::FONT_HERSHEY_SIMPLEX, 1.4, cv::Scalar(0, 255, 0), 4, cv::LINE_AA);
        cv::putText(eye_canvas, gear_str, cv::Point(hud_x, 320), cv::FONT_HERSHEY_SIMPLEX, 1.4, cv::Scalar(0, 255, 0), 4, cv::LINE_AA);

        cv::Mat map_x, map_y;
        generate_lens_maps(is_left, ipd_center_shift, map_x, map_y);

        cv::Mat distorted_eye;
        cv::remap(eye_canvas, distorted_eye, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        float optical_offset = 45.0f + (is_left ? ipd_center_shift : -ipd_center_shift);
        cv::Point lens_center((1080 / 2) + (is_left ? optical_offset : -optical_offset), 1200 / 2);
        
        int mask_radius = 535; 

        cv::Mat mask = cv::Mat::zeros(1200, 1080, CV_8UC1);
        cv::circle(mask, lens_center, mask_radius, cv::Scalar(255), -1, cv::LINE_AA);

        cv::Mat masked_eye = cv::Mat::zeros(1200, 1080, CV_8UC3);
        distorted_eye.copyTo(masked_eye, mask);

        return masked_eye;
    }

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (!msg->data.empty()) {
            cv::Mat decoded = cv::imdecode(msg->data, cv::IMREAD_COLOR);
            if (!decoded.empty()) latest_frame_ = decoded;
        }
    }

    void generate_lens_maps(bool is_left, float ipd_center_shift, cv::Mat& map_x, cv::Mat& map_y) {
        int w = 1080;
        int h = 1200;
        map_x.create(h, w, CV_32FC1);
        map_y.create(h, w, CV_32FC1);

        float k1 = 0.26f;
        float k2 = 0.16f;

        float optical_offset = 45.0f + (is_left ? ipd_center_shift : -ipd_center_shift);
        float cx = (w / 2.0f) + (is_left ? optical_offset : -optical_offset);
        float cy = h / 2.0f;

        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                float nx = (x - cx) / (w / 2.0f);
                float ny = (y - cy) / (h / 2.0f);
                float r2 = nx * nx + ny * ny;

                float distortion = 1.0f + k1 * r2 + k2 * r2 * r2;

                float ox = nx * distortion * (w / 2.0f) + cx;
                float oy = ny * distortion * (h / 2.0f) + cy;

                map_x.at<float>(y, x) = ox;
                map_y.at<float>(y, x) = oy;
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gear_sub_;
    cv::Mat latest_frame_;
    std::mutex frame_mutex_;
    float current_speed_ = 0.0f;
    int current_gear_ = 1;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VrHudNode>();
    std::thread ros_thread([&]() { rclcpp::spin(node); });

    std::string window_name = "VR HUD Extended Window";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::setWindowProperty(window_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);

    float ipd_center_shift = 0.0f; 

    RCLCPP_INFO(node->get_logger(), "VR HUD Oculus-Mask style started. Use 'A' and 'D' to tweak!");

    while (rclcpp::ok()) {
        cv::Mat left_eye = node->render_eye(true, ipd_center_shift);
        cv::Mat right_eye = node->render_eye(false, ipd_center_shift);

        cv::Mat sbs_frame;
        cv::hconcat(left_eye, right_eye, sbs_frame);
        cv::imshow(window_name, sbs_frame);

        int key = cv::waitKey(11);
        if (key == 27) { // ESC
            break;
        } else if (key == 121 || key == 201) { // F11
            double prop = cv::getWindowProperty(window_name, cv::WND_PROP_FULLSCREEN);
            if (prop == cv::WINDOW_FULLSCREEN) {
                cv::setWindowProperty(window_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);
            } else {
                cv::setWindowProperty(window_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
            }
        } 
        else if (key == 'a' || key == 'A' || key == 'а' || key == 'А') {
            ipd_center_shift -= 1.0f; 
            RCLCPP_INFO(node->get_logger(), "Optical Shift: %.1f", ipd_center_shift);
        } else if (key == 'd' || key == 'D' || key == 'в' || key == 'В') {
            ipd_center_shift += 1.0f;
            RCLCPP_INFO(node->get_logger(), "Optical Shift: %.1f", ipd_center_shift);
        }
    }

    cv::destroyAllWindows();
    rclcpp::shutdown();
    ros_thread.join();
    return 0;
}
