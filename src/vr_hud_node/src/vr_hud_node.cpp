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

        // Инициализируем карты дисторсии при старте
        update_lens_maps(0.0f);
    }

    // Метод для обновления карт (вызывается при изменении IPD)
    void update_lens_maps(float ipd_center_shift) {
        std::lock_guard<std::mutex> lock(maps_mutex_);
        generate_lens_maps(true, ipd_center_shift, left_map_x_, left_map_y_);
        generate_lens_maps(false, ipd_center_shift, right_map_x_, right_map_y_);
    }

    cv::Mat render_eye(bool is_left, float ipd_center_shift) {
        cv::Mat eye_canvas = cv::Mat::zeros(1200, 1080, CV_8UC3);

        // БЕЗОПАСНО КОПИРУЕМ КАДР ПОД МЬЮТЕКСОМ
        cv::Mat local_frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (!latest_frame_.empty()) {
                local_frame = latest_frame_.clone(); // clone гарантирует независимость памяти
            }
        }

        if (local_frame.empty()) {
            cv::putText(eye_canvas, "WAITING FOR CAMERA...", cv::Point(150, 600), 
                        cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 3);
            return eye_canvas;
        }

        // 1. МАСШТАБИРОВАНИЕ
        float scale_w = (1080.0f / local_frame.cols) * 1.0f;
        float scale_h = (1200.0f / local_frame.rows) * 1.0f;
        float max_scale = std::max(scale_w, scale_h);

        int target_w = static_cast<int>(local_frame.cols * max_scale);
        int target_h = static_cast<int>(local_frame.rows * max_scale);
        
        cv::Mat resized_camera;
        cv::resize(local_frame, resized_camera, cv::Size(target_w, target_h));

        // СБОРКА СТЕРЕОПАРЫ
        int texture_shift = -50; 
        int start_x = (target_w - 1080) / 2 + (is_left ? texture_shift : -texture_shift);
        int start_y = (target_h - 1200) / 2;

        start_x = std::max(0, std::min(start_x, target_w - 1080));
        start_y = std::max(0, std::min(start_y, target_h - 1200));

        cv::Rect crop_roi(start_x, start_y, 1080, 1200);
        resized_camera(crop_roi).copyTo(eye_canvas);
        
        // 2. ОТРИСОВКА HUD
        std::string speed_str = "SPEED: " + cv::format("%.1f", current_speed_.load()) + " km/h";
        int gear = current_gear_.load();
        std::string gear_str = "GEAR: " + (gear == 0 ? "R" : (gear == 1 ? "N" : std::to_string(gear - 1)));
        
        int hud_x = is_left ? 215 : 120; 
        cv::putText(eye_canvas, speed_str, cv::Point(hud_x, 240), cv::FONT_HERSHEY_SIMPLEX, 1.4, cv::Scalar(0, 255, 0), 4, cv::LINE_AA);
        cv::putText(eye_canvas, gear_str, cv::Point(hud_x, 320), cv::FONT_HERSHEY_SIMPLEX, 1.4, cv::Scalar(0, 255, 0), 4, cv::LINE_AA);
        
        // 3. ПРИМЕНЕНИЕ ДИСТОРСИИ ИЗ КЭША
        cv::Mat distorted_eye;
        {
            std::lock_guard<std::mutex> lock(maps_mutex_);
            cv::remap(eye_canvas, distorted_eye, 
                      is_left ? left_map_x_ : right_map_x_, 
                      is_left ? left_map_y_ : right_map_y_, 
                      cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        }

        // 4. НАЛОЖЕНИЕ МАСКИ
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
        if (!msg->data.empty()) {
            cv::Mat decoded = cv::imdecode(msg->data, cv::IMREAD_COLOR);
            if (!decoded.empty()) {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_frame_ = decoded;
            }
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
    std::mutex maps_mutex_;

    // Кэш для карт ремаппинга
    cv::Mat left_map_x_, left_map_y_;
    cv::Mat right_map_x_, right_map_y_;

    // Использование std::atomic защищает примитивные типы от race conditions без мьютексов
    std::atomic<float> current_speed_{0.0f};
    std::atomic<int> current_gear_{1};
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
            node->update_lens_maps(ipd_center_shift); // Пересчитываем карты только при изменении!
            RCLCPP_INFO(node->get_logger(), "Optical Shift: %.1f", ipd_center_shift);
        } else if (key == 'd' || key == 'D' || key == 'в' || key == 'В') {
            ipd_center_shift += 1.0f;
            node->update_lens_maps(ipd_center_shift); // Пересчитываем карты только при изменении!
            RCLCPP_INFO(node->get_logger(), "Optical Shift: %.1f", ipd_center_shift);
        }
    }

    cv::destroyAllWindows();
    rclcpp::shutdown();
    ros_thread.join();
    return 0;
}
