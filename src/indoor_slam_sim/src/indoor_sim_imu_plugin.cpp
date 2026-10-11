#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cstdint>
#include <memory>
#include <random>
#include <string>

class SimImuPlugin final : public gazebo::ModelPlugin {
 public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    link_ = model->GetLink(sdf->Get<std::string>("link_name", "base_link").first);
    if (!link_) {
      gzerr << "Indoor IMU link not found\n";
      return;
    }
    rate_hz_ = sdf->Get<double>("update_rate", 200.0).first;
    frame_ = sdf->Get<std::string>("frame_name", "imu_link").first;
    world_ = model->GetWorld();
    node_ = gazebo_ros::Node::Get(sdf);
    publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>(
        "/imu/data", rclcpp::SensorDataQoS());
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&SimImuPlugin::OnUpdate, this));
    gzmsg << "Indoor simulated IMU publishing /imu/data at " << rate_hz_ << " Hz\n";
  }

 private:
  void OnUpdate() {
    if (!publisher_ || !world_ || !link_) return;
    const auto stamp = world_->SimTime();
    const double now = stamp.Double();
    if (now - last_publish_ + 1e-9 < 1.0 / rate_hz_) return;
    last_publish_ = now;

    const auto rotation = link_->WorldPose().Rot();
    const auto angular = rotation.Inverse() * link_->WorldAngularVel();
    const auto specific_force = rotation.Inverse() *
        (link_->WorldLinearAccel() - world_->Gravity());

    sensor_msgs::msg::Imu msg;
    msg.header.stamp.sec = static_cast<int32_t>(stamp.sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>(stamp.nsec);
    msg.header.frame_id = frame_;
    msg.orientation.w = rotation.W();
    msg.orientation.x = rotation.X();
    msg.orientation.y = rotation.Y();
    msg.orientation.z = rotation.Z();
    msg.angular_velocity.x = angular.X();
    msg.angular_velocity.y = angular.Y();
    msg.angular_velocity.z = angular.Z();
    msg.linear_acceleration.x = specific_force.X();
    msg.linear_acceleration.y = specific_force.Y();
    msg.linear_acceleration.z = specific_force.Z();
    msg.angular_velocity.x += gyro_noise_(random_);
    msg.angular_velocity.y += gyro_noise_(random_);
    msg.angular_velocity.z += gyro_noise_(random_);
    msg.linear_acceleration.x += accel_noise_(random_);
    msg.linear_acceleration.y += accel_noise_(random_);
    msg.linear_acceleration.z += accel_noise_(random_);
    msg.orientation_covariance[0] = msg.orientation_covariance[4] =
        msg.orientation_covariance[8] = 1e-4;
    msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] =
        msg.angular_velocity_covariance[8] = 4e-8;
    msg.linear_acceleration_covariance[0] = msg.linear_acceleration_covariance[4] =
        msg.linear_acceleration_covariance[8] = 4e-6;
    publisher_->publish(msg);
  }

  gazebo::physics::LinkPtr link_;
  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  double rate_hz_{200.0};
  double last_publish_{-1.0};
  std::string frame_{"imu_link"};
  std::mt19937 random_{360};
  std::normal_distribution<double> gyro_noise_{0.0, 0.0002};
  std::normal_distribution<double> accel_noise_{0.0, 0.002};
};

GZ_REGISTER_MODEL_PLUGIN(SimImuPlugin)
