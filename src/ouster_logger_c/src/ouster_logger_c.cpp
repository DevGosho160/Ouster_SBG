// ROS2, PCL, and Open3D C++ includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <open3d/Open3D.h>
#include <chrono>
#include <fstream>
#include <mutex>
#include <thread>
#include <queue>
#include <filesystem>

#define SESSION_FILE "/home/imsel/data/session_path.txt"

using namespace std::chrono_literals;
using std::placeholders::_1;
using rclcpp::QoS;
using rclcpp::SensorDataQoS;

struct LidarData {
  sensor_msgs::msg::Imu imu;
  sensor_msgs::msg::PointCloud2 lidar;
};

class OusterDataNode : public rclcpp::Node {
public:
  OusterDataNode() : Node("ouster_log") {
    session_folder_ = read_session_path();
    session_folder_lidar_ = session_folder_ + "/lidar";

    std::filesystem::create_directories(session_folder_);
    std::filesystem::create_directories(session_folder_lidar_);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/ouster/imu", SensorDataQoS(), std::bind(&OusterDataNode::imu_callback, this, _1));
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/ouster/points", SensorDataQoS(), std::bind(&OusterDataNode::lidar_callback, this, _1));

    std::filesystem::create_directories(session_folder_ + "/metadata");
    csv_file_.open(session_folder_ + "/metadata/ouster_log.csv", std::ios::app);
    csv_file_ << "timestamp_sec,timestamp_nsec,orientation_x,orientation_y,orientation_z,orientation_w,";
    csv_file_ << "angular_velocity_x,angular_velocity_y,angular_velocity_z,";
    csv_file_ << "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z,";
    csv_file_ << "height,width,point_step,row_step\n";

    logger_thread_ = std::thread(&OusterDataNode::logger_worker, this);
    vis_thread_ = std::thread(&OusterDataNode::visualize_pointcloud, this);

    RCLCPP_INFO(this->get_logger(), "Saving LiDAR data to: %s", session_folder_.c_str());
  }

  ~OusterDataNode() {
    run_logger_ = false;
    if (logger_thread_.joinable()) logger_thread_.join();
    if (vis_thread_.joinable()) vis_thread_.join();
    if (csv_file_.is_open()) csv_file_.close();
  }

private:
  std::string read_session_path() {
    std::ifstream file(SESSION_FILE);
    std::string path;
    if (file.is_open()) {
      std::getline(file, path);
      if (!path.empty()) {
        std::filesystem::create_directories(path);
        return path;
      }
    }
    return "/mnt/ssd/metadata";
  }

  void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_imu_ = msg;
  }

  void lidar_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    LidarData data{*latest_imu_, *msg};
    lidar_queue_.push(data);
    latest_vis_lidar_ = msg;
  }


  void logger_worker() {
    while (rclcpp::ok() && run_logger_) {
      LidarData data;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (lidar_queue_.empty()) {
          std::this_thread::sleep_for(100ms);
          if (!rclcpp::ok()) break;
          continue;
        }
        data = lidar_queue_.front();
        lidar_queue_.pop();
      }

      auto now = std::chrono::system_clock::now();
      auto epoch = now.time_since_epoch();
      auto secs = std::chrono::duration_cast<std::chrono::seconds>(epoch);
      auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch - secs);

      csv_file_ << secs.count() << "," << nsecs.count() << ",";
      const auto& imu = data.imu;
      const auto& lidar = data.lidar;
      csv_file_ << imu.orientation.x << "," << imu.orientation.y << "," << imu.orientation.z << "," << imu.orientation.w << ",";
      csv_file_ << imu.angular_velocity.x << "," << imu.angular_velocity.y << "," << imu.angular_velocity.z << ",";
      csv_file_ << imu.linear_acceleration.x << "," << imu.linear_acceleration.y << "," << imu.linear_acceleration.z << ",";
      csv_file_ << lidar.height << "," << lidar.width << "," << lidar.point_step << "," << lidar.row_step << "\n";
      csv_file_.flush();

      std::string ply_path = session_folder_lidar_ + "/lidar_cap_" + std::to_string(index_++) + ".ply";
      save_pointcloud2_to_ply(data.lidar, ply_path);
    }
  }

  void save_pointcloud2_to_ply(const sensor_msgs::msg::PointCloud2& msg, const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(msg, *cloud);

    // Convert to Open3D format
    open3d::geometry::PointCloud o3d_cloud;
    for (const auto& pt : cloud->points) {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
        o3d_cloud.points_.emplace_back(pt.x, pt.y, pt.z);
        float intensity = std::min(pt.intensity / 255.0f, 0.9f);
        intensity = std::max(intensity, 0.1f);
        o3d_cloud.colors_.emplace_back(intensity, intensity, intensity);
      }
    }
    if (o3d_cloud.points_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Skipping write: point cloud is empty.");
      return;
    }
    open3d::io::WritePointCloud(filename, o3d_cloud, 
      open3d::io::WritePointCloudOption(/* write_ascii */ true));
  }

  void visualize_pointcloud() {
    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow("Live PointCloud", 1000, 750);
    auto pcd_ptr = std::make_shared<open3d::geometry::PointCloud>();
    bool added = false;

    while (rclcpp::ok() && run_logger_) {
      std::shared_ptr<sensor_msgs::msg::PointCloud2> msg;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        msg = latest_vis_lidar_;
      }
      if (!msg) {
        std::this_thread::sleep_for(100ms);
        continue;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*msg, *pcl_cloud);

      pcd_ptr->Clear();
      for (const auto& pt : pcl_cloud->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
          pcd_ptr->points_.emplace_back(pt.x, pt.y, pt.z);
          float intensity = std::min(pt.intensity / 255.0f, 0.9f);
          pcd_ptr->colors_.emplace_back(intensity, intensity, intensity);
        }
      }

      if (!added) {
        vis.AddGeometry(pcd_ptr);
        added = true;
      } else {
        vis.UpdateGeometry(pcd_ptr);
      }

      vis.PollEvents();
      vis.UpdateRender();
      std::this_thread::sleep_for(100ms);
    }

    vis.DestroyVisualizerWindow();
  }


  std::string session_folder_;
  std::string session_folder_lidar_;
  std::ofstream csv_file_;
  std::atomic<bool> run_logger_ = true;
  std::thread logger_thread_;
  std::thread vis_thread_;
  std::mutex mutex_;
  std::queue<LidarData> lidar_queue_;
  std::shared_ptr<sensor_msgs::msg::Imu> latest_imu_;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> latest_vis_lidar_;
  size_t index_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OusterDataNode>());
  rclcpp::shutdown();
  return 0;
}
