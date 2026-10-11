#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "indoor_slam_sim/csv_reader.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace indoor_slam_sim {
struct PatternRay { double azimuth; double elevation; };

class Mid360GpuPlugin final : public gazebo::SensorPlugin {
 public:
  void Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf) override {
    sensor_ = std::dynamic_pointer_cast<gazebo::sensors::GpuRaySensor>(parent);
    if (!sensor_) {
      gzerr << "Indoor Mid-360 GPU plugin requires a Gazebo GpuRaySensor\n";
      return;
    }
    node_ = gazebo_ros::Node::Get(sdf);
    const auto topic = sdf->Get<std::string>("topic", "/livox/lidar").first;
    frame_id_ = sdf->Get<std::string>("frame_id", "livox_frame").first;
    samples_ = std::max(1, sdf->Get<int>("samples", 2000).first);
    min_range_ = sdf->Get<double>("min_range", 0.15).first;
    max_range_ = sdf->Get<double>("max_range", 30.0).first;
    publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS());

    std::vector<std::vector<double>> values;
    const auto csv = sdf->Get<std::string>("csv_file_name");
    if (!CsvReader::ReadCsvFile(csv, values)) {
      RCLCPP_ERROR(node_->get_logger(), "Cannot load Mid-360 CSV: %s", csv.c_str());
      return;
    }
    for (const auto &row : values) {
      if (row.size() >= 3) {
        pattern_.push_back({row[1] * M_PI / 180.0,
                            row[2] * M_PI / 180.0 - M_PI_2});
      }
    }
    if (pattern_.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Mid-360 CSV has no valid pattern rows");
      return;
    }

    sensor_->SetActive(true);
    connection_ = sensor_->ConnectNewLaserFrame(
        [this](const float *, unsigned int, unsigned int, unsigned int,
               const std::string &) { PublishCloud(); });
    RCLCPP_INFO(node_->get_logger(),
        "MID360_GPU_SIM=READY topic=%s frame=%s pattern_points=%zu output_points=%d "
        "gpu_grid=%dx%d range_m=[%.2f,%.1f] update_hz=10 point_timing=global_shutter",
        publisher_->get_topic_name(), frame_id_.c_str(), pattern_.size(), samples_,
        sensor_->RangeCount(), sensor_->VerticalRangeCount(), min_range_, max_range_);
  }

 private:
  void PublishCloud() {
    if (!sensor_ || !publisher_) return;
    const auto sim_time = sensor_->LastUpdateTime();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp.sec = static_cast<int32_t>(sim_time.sec);
    cloud.header.stamp.nanosec = static_cast<uint32_t>(sim_time.nsec);
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(static_cast<size_t>(samples_));
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");

    const double h_min = sensor_->AngleMin().Radian();
    const double v_min = sensor_->VerticalAngleMin().Radian();
    const double h_step = sensor_->AngleResolution();
    const double v_step = sensor_->VerticalAngleResolution();
    const int width = sensor_->RangeCount();
    const int height = sensor_->VerticalRangeCount();
    for (int i = 0; i < samples_; ++i) {
      const auto pattern_index = static_cast<size_t>(i) * pattern_.size() / samples_;
      const auto &entry = pattern_[pattern_index];
      int h = h_step > 0 ? static_cast<int>(std::lround((entry.azimuth - h_min) / h_step)) : 0;
      int v = v_step > 0 ? static_cast<int>(std::lround((entry.elevation - v_min) / v_step)) : 0;
      h = std::clamp(h, 0, width - 1);
      v = std::clamp(v, 0, height - 1);
      const int index = v * width + h;
      const double range = sensor_->Range(index);
      if (range < min_range_ || range >= max_range_ || !std::isfinite(range)) {
        *x = *y = *z = std::numeric_limits<float>::quiet_NaN();
        *intensity = 0.0F;
      } else {
        const double horizontal = std::cos(entry.elevation) * range;
        *x = static_cast<float>(horizontal * std::cos(entry.azimuth));
        *y = static_cast<float>(horizontal * std::sin(entry.azimuth));
        *z = static_cast<float>(range * std::sin(entry.elevation));
        *intensity = static_cast<float>(sensor_->Retro(index));
      }
      ++x; ++y; ++z; ++intensity;
    }
    cloud.is_dense = false;
    publisher_->publish(cloud);
  }

  gazebo::sensors::GpuRaySensorPtr sensor_;
  gazebo::event::ConnectionPtr connection_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::vector<PatternRay> pattern_;
  std::string frame_id_{"livox_frame"};
  int samples_{2000};
  double min_range_{0.15};
  double max_range_{30.0};
};
}  // namespace indoor_slam_sim

GZ_REGISTER_SENSOR_PLUGIN(indoor_slam_sim::Mid360GpuPlugin)
