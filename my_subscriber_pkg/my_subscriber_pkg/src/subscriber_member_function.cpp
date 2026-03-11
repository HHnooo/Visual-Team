#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <memory>
#include <cmath>
#include <functional>
#include <algorithm>
#include <cstring>

constexpr int IMAGE_WIDTH = 1152;
constexpr int IMAGE_HEIGHT = 648;
constexpr double BULLET_SPEED = 600.0;
constexpr double MAX_ANGLE = 180.0;
constexpr double MIN_ANGLE = -180.0;
constexpr int FIRE_CD_MS = 10;
constexpr int MAX_MISSED_FRAMES = 60;
const int ROI_Y_START = IMAGE_HEIGHT - 80;
const int ROI_Y_END = IMAGE_HEIGHT;
const int ROI_X_START = IMAGE_WIDTH / 2 - 80;
const int ROI_X_END = IMAGE_WIDTH / 2 + 80;

std::vector<uint8_t> angleToBytes(double angle_deg) {
    float f = static_cast<float>(angle_deg);
    uint8_t *bytes = reinterpret_cast<uint8_t *>(&f);
    return {bytes[0], bytes[1], bytes[2], bytes[3]};
}

class SerialPort {
public:
    SerialPort() : fd_(-1) {
    }

    ~SerialPort() { if (fd_ >= 0) close(fd_); }

    bool open(const std::string &port, int baudrate) {
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) return false;

        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) return false;

        cfsetospeed(&tty, baudrate);
        cfsetispeed(&tty, baudrate);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) return false;
        return true;
    }

    bool sendTurnCmd(double angle_deg) {
        uint8_t cmd[5] = {0x01};
        float angle_f = static_cast<float>(angle_deg);
        memcpy(cmd + 1, &angle_f, 4);
        return writeData(cmd, 5);
    }

    bool sendFireCmd() {
        uint8_t cmd = 0x02;
        return writeData(&cmd, 1);
    }

private:
    int fd_;

    bool writeData(const uint8_t *data, size_t size) {
        if (fd_ < 0) return false;
        ssize_t written = ::write(fd_, data, size);
        if (written != static_cast<ssize_t>(size)) {
            return false;
        }
        usleep(size * 500);
        return true;
    }
};

class ArmorTarget {
public:
    int id;
    double last_update_time;
    bool is_alive;
    int missed_frames;
    cv::KalmanFilter kf;
    cv::Point2f pos, vel, acc;

    ArmorTarget(int id, const cv::Point2f &init_pos, double time,
                double process_noise_pos, double process_noise_vel, double process_noise_acc,
                double measurement_noise)
        : id(id), last_update_time(time), is_alive(true), missed_frames(0) {
        const int state_size = 6;
        const int meas_size = 2;
        kf.init(state_size, meas_size, 0, CV_32F);

        cv::setIdentity(kf.transitionMatrix, cv::Scalar(1));

        kf.measurementMatrix = cv::Mat::zeros(meas_size, state_size, CV_32F);
        kf.measurementMatrix.at<float>(0, 0) = 1.0f;
        kf.measurementMatrix.at<float>(1, 1) = 1.0f;

        cv::setIdentity(kf.processNoiseCov, cv::Scalar(0));
        kf.processNoiseCov.at<float>(0, 0) = process_noise_pos;
        kf.processNoiseCov.at<float>(1, 1) = process_noise_pos;
        kf.processNoiseCov.at<float>(2, 2) = process_noise_vel;
        kf.processNoiseCov.at<float>(3, 3) = process_noise_vel;
        kf.processNoiseCov.at<float>(4, 4) = process_noise_acc;
        kf.processNoiseCov.at<float>(5, 5) = process_noise_acc;

        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(measurement_noise));

        cv::Mat state(state_size, 1, CV_32F);
        state.at<float>(0) = init_pos.x;
        state.at<float>(1) = init_pos.y;
        state.at<float>(2) = 0;
        state.at<float>(3) = 0;
        state.at<float>(4) = 0;
        state.at<float>(5) = 0;
        kf.statePost = state;


        cv::setIdentity(kf.errorCovPost, cv::Scalar(100));
        kf.errorCovPost.at<float>(2, 2) = 500;
        kf.errorCovPost.at<float>(3, 3) = 500;
        kf.errorCovPost.at<float>(4, 4) = 1000;
        kf.errorCovPost.at<float>(5, 5) = 1000;

        pos = init_pos;
        vel = cv::Point2f(0, 0);
        acc = cv::Point2f(0, 0);
    }

    cv::Point2f predict(double current_time) {
        double dt = current_time - last_update_time;
        if (dt < 0) dt = 0;

        kf.transitionMatrix.at<float>(0, 2) = dt;
        kf.transitionMatrix.at<float>(0, 4) = 0.5f * dt * dt;
        kf.transitionMatrix.at<float>(1, 3) = dt;
        kf.transitionMatrix.at<float>(1, 5) = 0.5f * dt * dt;
        kf.transitionMatrix.at<float>(2, 4) = dt;
        kf.transitionMatrix.at<float>(3, 5) = dt;

        cv::Mat pred = kf.predict();
        pos.x = pred.at<float>(0);
        pos.y = pred.at<float>(1);
        vel.x = pred.at<float>(2);
        vel.y = pred.at<float>(3);
        acc.x = pred.at<float>(4);
        acc.y = pred.at<float>(5);
        return pos;
    }

    void correct(const cv::Point2f &meas_pos, double time) {
        cv::Mat z(2, 1, CV_32F);
        z.at<float>(0) = meas_pos.x;
        z.at<float>(1) = meas_pos.y;
        cv::Mat est = kf.correct(z);
        pos.x = est.at<float>(0);
        pos.y = est.at<float>(1);
        vel.x = est.at<float>(2);
        vel.y = est.at<float>(3);
        acc.x = est.at<float>(4);
        acc.y = est.at<float>(5);
        last_update_time = time;
    }

    cv::Point2f predictFuture(double dt) {
        return pos + vel * dt + 0.5f * acc * dt * dt;
    }

    cv::Point2f predictFutureIterative(double initial_flight_time, const cv::Point2f &gun_pos, int iterations = 10) {
        double t = initial_flight_time;
        for (int i = 0; i < iterations; ++i) {
            cv::Point2f future = pos + vel * t + 0.5f * acc * t * t;
            double dist = cv::norm(future - gun_pos);
            t = dist / BULLET_SPEED;
        }
        return pos + vel * t + 0.5f * acc * t * t;
    }
};

class MainNode : public rclcpp::Node {
public:
    MainNode() : Node("main_node"),
                 last_fire_time_(0),
                 next_enemy_id_(0), next_own_id_(0),
                 color_initialized_(false),
                 launcher_pos_(576, 540) {
        this->declare_parameter<std::string>("difficulty", "中杯");
        std::string difficulty = this->get_parameter("difficulty").as_string();

        if (difficulty == "中杯") {
            process_noise_pos_ = 1e-5;
            measurement_noise_ = 1e-3;
        } else if (difficulty == "大杯") {
            process_noise_pos_ = 1e-4;
            measurement_noise_ = 1e-4;
        } else if (difficulty == "超大杯") {
            process_noise_pos_ = 1e-3;
            measurement_noise_ = 1e-1;
        }
        process_noise_vel_ = process_noise_pos_ * 10;
        process_noise_acc_ = process_noise_pos_ * 100;

        this->declare_parameter<std::string>("serial_port", "/dev/pts/1");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<std::string>("own_color", "auto");

        std::string port = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("baudrate").as_int();
        own_color_param_ = this->get_parameter("own_color").as_string();

        if (!serial_.open(port, baud)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port.c_str());
        }

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&MainNode::imageCallback, this, std::placeholders::_1));
    }

private:
    std::string detectOwnColor(const cv::Mat &frame) {
        cv::Rect roi(ROI_X_START, ROI_Y_START, ROI_X_END - ROI_X_START, ROI_Y_END - ROI_Y_START);
        cv::Mat roi_img = frame(roi);
        cv::Mat hsv;
        cv::cvtColor(roi_img, hsv, cv::COLOR_BGR2HSV);
        cv::Mat red_mask1, red_mask2, red_mask;
        cv::inRange(hsv, cv::Scalar(0, 30, 30), cv::Scalar(20, 255, 255), red_mask1);
        cv::inRange(hsv, cv::Scalar(160, 30, 30), cv::Scalar(180, 255, 255), red_mask2);
        cv::bitwise_or(red_mask1, red_mask2, red_mask);
        cv::Mat blue_mask;
        cv::inRange(hsv, cv::Scalar(90, 30, 30), cv::Scalar(130, 255, 255), blue_mask);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_OPEN, kernel);

        int red_pixels = cv::countNonZero(red_mask);
        int blue_pixels = cv::countNonZero(blue_mask);

        const int MIN_PIXELS = 100;
        if (red_pixels > blue_pixels && red_pixels > MIN_PIXELS) {
            return "red";
        } else if (blue_pixels > red_pixels && blue_pixels > MIN_PIXELS) {
            return "blue";
        } else {
            return "unknown";
        }
    }

    void updateColorTargets(const cv::Mat &frame,
                            const cv::Scalar &lower, const cv::Scalar &upper,
                            std::vector<std::shared_ptr<ArmorTarget> > &targets,
                            int &next_id, double current_time) {
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, lower, upper, mask);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(50, 1));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point2f> detections;
        for (const auto &contour: contours) {
            double area = cv::contourArea(contour);
            if (area < 30 || area > 8000) continue;
            cv::RotatedRect rect = cv::minAreaRect(contour);
            float width = rect.size.width;
            float height = rect.size.height;
            if (width < height) std::swap(width, height);
            if (width == 0 || height == 0) continue;
            float aspect = width / height;
            if (aspect < 1.5 || aspect > 7.0) continue;
            if (rect.center.y < 540) {
                detections.push_back(rect.center);
            }
        }
        std::vector<bool> matched_detections(detections.size(), false);

        for (size_t i = 0; i < targets.size(); ++i) {
            cv::Point2f pred = targets[i]->predict(current_time);

            double min_dist = 100.0;
            int best_j = -1;
            for (size_t j = 0; j < detections.size(); ++j) {
                if (matched_detections[j]) continue;
                double dist = cv::norm(pred - detections[j]);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_j = j;
                }
            }

            if (best_j >= 0) {
                matched_detections[best_j] = true;
                targets[i]->correct(detections[best_j], current_time);
                targets[i]->missed_frames = 0;
            } else {
                targets[i]->missed_frames++;
                if (targets[i]->missed_frames > MAX_MISSED_FRAMES) {
                    targets[i]->is_alive = false;
                }
            }
        }
        for (size_t j = 0; j < detections.size(); ++j) {
            if (!matched_detections[j]) {
                auto new_target = std::make_shared<ArmorTarget>(
                    next_id++, detections[j], current_time,
                    process_noise_pos_, process_noise_vel_, process_noise_acc_, measurement_noise_);
                targets.push_back(new_target);
            }
        }
        targets.erase(
            std::remove_if(
                targets.begin(), targets.end(),
                [](const std::shared_ptr<ArmorTarget> &t) { return !t->is_alive; }
            ),
            targets.end()
        );
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        double current_time = rclcpp::Time(msg->header.stamp).seconds();
        double now = this->now().seconds();

        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }
        if (!color_initialized_) {
            std::string own_color;
            if (own_color_param_ != "auto") {
                own_color = own_color_param_;
            } else {
                own_color = detectOwnColor(frame);
            }

            if (own_color != "unknown") {
                if (own_color == "red") {
                    enemy_lower_ = cv::Scalar(100, 50, 50);
                    enemy_upper_ = cv::Scalar(130, 255, 255);
                    own_lower_ = cv::Scalar(0, 50, 50);
                    own_upper_ = cv::Scalar(10, 255, 255);
                } else {
                    enemy_lower_ = cv::Scalar(0, 50, 50);
                    enemy_upper_ = cv::Scalar(10, 255, 255);
                    own_lower_ = cv::Scalar(100, 50, 50);
                    own_upper_ = cv::Scalar(130, 255, 255);
                }
                color_initialized_ = true;
            } else {
                return;
            }
        }

        updateColorTargets(frame, enemy_lower_, enemy_upper_, enemy_targets_, next_enemy_id_, current_time);
        updateColorTargets(frame, own_lower_, own_upper_, own_targets_, next_own_id_, current_time);

        if (!enemy_targets_.empty()) {
            auto best = enemy_targets_[0];
            double best_dist = cv::norm(best->predict(current_time) - launcher_pos_);
            for (auto &t: enemy_targets_) {
                double d = cv::norm(t->predict(current_time) - launcher_pos_);
                if (d < best_dist) {
                    best_dist = d;
                    best = t;
                }
            }

            cv::Point2f pos = best->predict(current_time);
            double dx = pos.x - launcher_pos_.x;
            double dy = pos.y - launcher_pos_.y;
            double pixel_dist = std::sqrt(dx * dx + dy * dy);
            double flight_time_guess = pixel_dist / BULLET_SPEED;
            cv::Point2f future_pos = best->predictFutureIterative(flight_time_guess, launcher_pos_, 10);
            double angle_rad = std::atan2(future_pos.y - launcher_pos_.y, future_pos.x - launcher_pos_.x);
            double angle_deg = angle_rad * 180.0 / CV_PI;
            if (angle_deg > MAX_ANGLE) angle_deg = MAX_ANGLE;
            if (angle_deg < MIN_ANGLE) angle_deg = MIN_ANGLE;
            serial_.sendTurnCmd(-angle_deg);
            if (now - last_fire_time_ >= FIRE_CD_MS / 1000.0) {
                if (canFire(best, future_pos, current_time)) {
                    if (serial_.sendFireCmd()) {
                        last_fire_time_ = now;
                    }
                }
            }
        }
    }

    double pointToLineDist(const cv::Point2f &P, const cv::Point2f &A, const cv::Point2f &B) {
        cv::Point2f AB = B - A;
        cv::Point2f AP = P - A;
        double ab2 = AB.x * AB.x + AB.y * AB.y;
        double t = (AP.x * AB.x + AP.y * AB.y) / ab2;
        t = std::max(0.0, std::min(1.0, t));
        cv::Point2f projection = A + t * AB;
        return cv::norm(P - projection);
    }

    bool canFire(const std::shared_ptr<ArmorTarget> &enemy, const cv::Point2f &enemy_future_pos, double current_time) {
        (void) enemy;
        for (auto &own: own_targets_) {
            if (!own->is_alive) continue;
            cv::Point2f own_pos = own->predict(current_time);
            double dist = pointToLineDist(own_pos, launcher_pos_, enemy_future_pos);
            const double safe_dist = 100.0;
            if (dist < safe_dist) {
                return false;
            }
        }
        return true;
    }

    double process_noise_pos_;
    double process_noise_vel_;
    double process_noise_acc_;
    double measurement_noise_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    SerialPort serial_;
    double last_fire_time_;
    std::vector<std::shared_ptr<ArmorTarget> > enemy_targets_;
    std::vector<std::shared_ptr<ArmorTarget> > own_targets_;
    int next_enemy_id_;
    int next_own_id_;
    cv::Scalar enemy_lower_, enemy_upper_;
    cv::Scalar own_lower_, own_upper_;
    bool color_initialized_;
    std::string own_color_param_;
    cv::Point2f launcher_pos_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MainNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}