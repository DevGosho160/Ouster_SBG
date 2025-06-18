#include <rclcpp/rclcpp.hpp>
#include <sbg_driver/msg/sbg_imu_data.hpp>
#include <sbg_driver/msg/sbg_ship_motion.hpp>
#include <sbg_driver/msg/sbg_ekf_euler.hpp>
#include <sbg_driver/msg/sbg_ekf_nav.hpp>
#include <fstream>
#include <filesystem>
#include <iomanip>

const std::string SESSION_FILE = "/mnt/ssd/session_path.txt";
std::string session_folder;
std::string save_path;

class SbgLoggerNode : public rclcpp::Node {
public:
    SbgLoggerNode() : Node("sbg_logger_node") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliability(rclcpp::ReliabilityPolicy::BestEffort);

        // Read session path
        std::ifstream infile(SESSION_FILE);
        if (!infile.good()) {
            RCLCPP_WARN(this->get_logger(), "Session file not found, using fallback path.");
            session_folder = "/mnt/ssd/metadata/";
        } else {
            std::getline(infile, session_folder);
        }
        save_path = session_folder + "/sbg_log/";
        std::filesystem::create_directories(save_path);
        RCLCPP_INFO(this->get_logger(), "Logging to: %s", save_path.c_str());

        // Subscriptions
        imu_sub_ = this->create_subscription<sbg_driver::msg::SbgImuData>(
            "/sbg/imu_data", qos, std::bind(&SbgLoggerNode::imuCallback, this, std::placeholders::_1));
        ship_sub_ = this->create_subscription<sbg_driver::msg::SbgShipMotion>(
            "/sbg/ship_motion", qos, std::bind(&SbgLoggerNode::shipCallback, this, std::placeholders::_1));
        euler_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfEuler>(
            "/sbg/ekf_euler", qos, std::bind(&SbgLoggerNode::eulerCallback, this, std::placeholders::_1));
        nav_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfNav>(
            "/sbg/ekf_nav", qos, std::bind(&SbgLoggerNode::navCallback, this, std::placeholders::_1));
    }

private:
    // Log each type to a single file (append mode)
    void imuCallback(const sbg_driver::msg::SbgImuData::SharedPtr msg) {
        std::ofstream out(save_path + "imu_data.txt", std::ios::app);
        out << std::unitbuf;
        out << "IMU Timestamp (sec,nsec): " << msg->header.stamp.sec << ", " << msg->header.stamp.nanosec << "\n";
        out << "IMU Data: "
            << msg->accel.x << ", " << msg->accel.y << ", " << msg->accel.z << ", "
            << msg->gyro.x  << ", " << msg->gyro.y  << ", " << msg->gyro.z  << "\n\n";
    }

    void shipCallback(const sbg_driver::msg::SbgShipMotion::SharedPtr msg) {
        std::ofstream out(save_path + "ship_motion.txt", std::ios::app);
        out << std::unitbuf;
        out << "Ship Motion Timestamp: " << msg->header.stamp.sec << ", " << msg->header.stamp.nanosec << "\n";
        out << "Ship Motion Data: "
            << msg->heave_period << ", "
            << msg->ship_motion.x << ", " << msg->ship_motion.y << ", " << msg->ship_motion.z << ", "
            << msg->acceleration.x << ", " << msg->acceleration.y << ", " << msg->acceleration.z << ", "
            << msg->velocity.x    << ", " << msg->velocity.y    << ", " << msg->velocity.z << "\n\n";
    }

    void eulerCallback(const sbg_driver::msg::SbgEkfEuler::SharedPtr msg) {
        std::ofstream out(save_path + "ekf_euler.txt", std::ios::app);
        out << std::unitbuf;
        out << "Euler Timestamp: " << msg->header.stamp.sec << ", " << msg->header.stamp.nanosec << "\n";
        out << "Euler Data: "
            << msg->angle.x << ", " << msg->angle.y << ", " << msg->angle.z << ", "
            << msg->accuracy.x << ", " << msg->accuracy.y << ", " << msg->accuracy.z << "\n\n";
    }

    void navCallback(const sbg_driver::msg::SbgEkfNav::SharedPtr msg) {
        std::ofstream out(save_path + "ekf_nav.txt", std::ios::app);
        out << std::unitbuf;
        out << "Nav Timestamp: " << msg->header.stamp.sec << ", " << msg->header.stamp.nanosec << "\n";
        out << "Nav Data: "
            << msg->velocity.x << ", " << msg->velocity.y << ", " << msg->velocity.z << ", "
            << msg->velocity_accuracy.x << ", " << msg->velocity_accuracy.y << ", " << msg->velocity_accuracy.z << ", "
            << msg->latitude << ", " << msg->longitude << ", " << msg->altitude << ", " << msg->undulation << ", "
            << msg->position_accuracy.x << ", " << msg->position_accuracy.y << ", " << msg->position_accuracy.z << "\n\n";
    }

    rclcpp::Subscription<sbg_driver::msg::SbgImuData>::SharedPtr imu_sub_;
    rclcpp::Subscription<sbg_driver::msg::SbgShipMotion>::SharedPtr ship_sub_;
    rclcpp::Subscription<sbg_driver::msg::SbgEkfEuler>::SharedPtr euler_sub_;
    rclcpp::Subscription<sbg_driver::msg::SbgEkfNav>::SharedPtr nav_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SbgLoggerNode>());
    rclcpp::shutdown();
    return 0;
}
