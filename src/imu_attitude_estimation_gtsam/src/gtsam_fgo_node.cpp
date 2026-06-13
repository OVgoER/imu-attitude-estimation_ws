#include <cmath>
#include <deque>
#include <memory>
#include <string>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <Eigen/Dense>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

double stampToSec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

Eigen::Vector3d vec3(double x, double y, double z)
{
  return Eigen::Vector3d(x, y, z);
}

gtsam::Rot3 rotFromAccel(const Eigen::Vector3d & accel)
{
  const double roll = std::atan2(accel.y(), accel.z());
  const double pitch = std::atan2(-accel.x(), std::sqrt(accel.y() * accel.y() + accel.z() * accel.z()));
  return gtsam::Rot3::RzRyRx(roll, pitch, 0.0);
}

void fillOdom(
  nav_msgs::msg::Odometry & msg,
  const sensor_msgs::msg::Imu & imu,
  const gtsam::Pose3 & pose,
  const Eigen::Vector3d & velocity,
  const gtsam::imuBias::ConstantBias & bias)
{
  msg.header.stamp = imu.header.stamp;
  msg.header.frame_id = "world";
  msg.child_frame_id = "imu_estimator_fgo_gtsam";
  const auto t = pose.translation();
  msg.pose.pose.position.x = t.x();
  msg.pose.pose.position.y = t.y();
  msg.pose.pose.position.z = t.z();
  const auto q = pose.rotation().toQuaternion();
  msg.pose.pose.orientation.w = q.w();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.twist.twist.linear.x = velocity.x();
  msg.twist.twist.linear.y = velocity.y();
  msg.twist.twist.linear.z = velocity.z();
  const auto gyro_bias = bias.gyroscope();
  msg.twist.twist.angular.x = gyro_bias.x();
  msg.twist.twist.angular.y = gyro_bias.y();
  msg.twist.twist.angular.z = gyro_bias.z();
  msg.pose.covariance[0] = 0.02;
  msg.pose.covariance[7] = 0.02;
  msg.pose.covariance[14] = 0.02;
  msg.pose.covariance[21] = 0.01;
  msg.pose.covariance[28] = 0.01;
  msg.pose.covariance[35] = 0.01;
  msg.twist.covariance = msg.pose.covariance;
}

}  // namespace

class GtsamFgoNode : public rclcpp::Node
{
public:
  GtsamFgoNode()
  : Node("gtsam_fgo_node")
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/raw");
    keyframe_dt_ = declare_parameter<double>("keyframe_dt", 0.10);
    gravity_ = declare_parameter<double>("gravity", 9.80665);
    gyro_noise_ = declare_parameter<double>("gyro_noise", 0.004);
    accel_noise_ = declare_parameter<double>("accel_noise", 0.08);
    gyro_bias_rw_ = declare_parameter<double>("gyro_bias_rw", 0.0006);
    accel_bias_rw_ = declare_parameter<double>("accel_bias_rw", 0.01);
    static_gyro_threshold_ = declare_parameter<double>("static_gyro_threshold", 0.045);
    static_accel_threshold_ = declare_parameter<double>("static_accel_threshold", 0.22);
    enable_loop_closure_ = declare_parameter<bool>("enable_loop_closure", false);
    loop_closure_after_ = declare_parameter<double>("loop_closure_after", 45.0);

    params_ = gtsam::PreintegrationParams::MakeSharedU(gravity_);
    params_->setGyroscopeCovariance(Eigen::Matrix3d::Identity() * gyro_noise_ * gyro_noise_);
    params_->setAccelerometerCovariance(Eigen::Matrix3d::Identity() * accel_noise_ * accel_noise_);
    params_->setIntegrationCovariance(Eigen::Matrix3d::Identity() * 1e-6);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/attitude/fgo", 20);
    zupt_pub_ = create_publisher<std_msgs::msg::Bool>("/imu/fgo_zupt_active", 20);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 100, std::bind(&GtsamFgoNode::onImu, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Native GTSAM FGO node ready; subscribed to %s", imu_topic_.c_str());
  }

private:
  struct Keyframe
  {
    double stamp{};
    gtsam::Pose3 pose;
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    gtsam::imuBias::ConstantBias bias;
    gtsam::PreintegratedImuMeasurements pim;
    bool is_static{false};
  };

  void initialize(const sensor_msgs::msg::Imu & msg)
  {
    const Eigen::Vector3d accel = vec3(
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    const gtsam::Rot3 rot = rotFromAccel(accel);
    current_pose_ = gtsam::Pose3(rot, gtsam::Point3(0.0, 0.0, 0.0));
    current_velocity_.setZero();
    current_bias_ = gtsam::imuBias::ConstantBias();
    reference_pose_ = current_pose_;
    reference_velocity_ = current_velocity_;
    last_stamp_ = stampToSec(msg.header.stamp);
    last_keyframe_stamp_ = last_stamp_;
    current_pim_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(params_, current_bias_);
    initialized_ = true;
    addKeyframe(last_stamp_, true);
    publish(msg, false);
  }

  bool isStatic(const sensor_msgs::msg::Imu & msg) const
  {
    const Eigen::Vector3d gyro = vec3(
      msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    const Eigen::Vector3d accel = vec3(
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    return gyro.norm() < static_gyro_threshold_ && std::abs(accel.norm() - gravity_) < static_accel_threshold_;
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!initialized_) {
      initialize(*msg);
      return;
    }
    const double stamp = stampToSec(msg->header.stamp);
    double dt = stamp - last_stamp_;
    if (dt <= 0.0 || dt > 1.0) {
      last_stamp_ = stamp;
      return;
    }
    const Eigen::Vector3d accel = vec3(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    const Eigen::Vector3d gyro = vec3(
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    current_pim_->integrateMeasurement(accel, gyro, dt);
    const gtsam::NavState predicted =
      current_pim_->predict(gtsam::NavState(current_pose_, current_velocity_), current_bias_);
    current_pose_ = predicted.pose();
    current_velocity_ = predicted.velocity();

    const bool stationary = isStatic(*msg);
    if (stationary) {
      const Eigen::Vector3d acc_bias = 0.98 * current_bias_.accelerometer() +
        0.02 * (accel - Eigen::Vector3d(0.0, 0.0, gravity_));
      const Eigen::Vector3d gyro_bias = 0.98 * current_bias_.gyroscope() + 0.02 * gyro;
      current_bias_ = gtsam::imuBias::ConstantBias(acc_bias, gyro_bias);
      current_velocity_ *= 0.20;
    }

    if (stamp - last_keyframe_stamp_ >= keyframe_dt_) {
      addKeyframe(stamp, stationary);
      optimize(stamp);
      current_pim_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(params_, current_bias_);
      last_keyframe_stamp_ = stamp;
    }
    last_stamp_ = stamp;
    publish(*msg, stationary);
  }

  void addKeyframe(double stamp, bool stationary)
  {
    Keyframe frame{
      stamp,
      current_pose_,
      current_velocity_,
      current_bias_,
      *current_pim_,
      stationary,
    };
    frames_.push_back(frame);
    constexpr std::size_t max_frames = 18;
    while (frames_.size() > max_frames) {
      frames_.pop_front();
    }
  }

  void optimize(double stamp)
  {
    if (frames_.size() < 3) {
      return;
    }
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.02, 0.02, 0.02, 0.08, 0.08, 0.08).finished());
    auto velocity_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);
    auto bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.03);
    auto zupt_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.01);
    auto loop_pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.005, 0.005, 0.005, 0.02, 0.02, 0.02).finished());

    for (std::size_t i = 0; i < frames_.size(); ++i) {
      values.insert(X(i), frames_[i].pose);
      values.insert(V(i), frames_[i].velocity);
      values.insert(B(i), frames_[i].bias);
    }
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), frames_[0].pose, pose_noise));
    graph.add(gtsam::PriorFactor<Eigen::Vector3d>(V(0), frames_[0].velocity, velocity_noise));
    graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), frames_[0].bias, bias_noise));

    for (std::size_t i = 1; i < frames_.size(); ++i) {
      graph.add(gtsam::ImuFactor(X(i - 1), V(i - 1), X(i), V(i), B(i - 1), frames_[i].pim));
      graph.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
        B(i - 1), B(i), gtsam::imuBias::ConstantBias(),
        gtsam::noiseModel::Isotropic::Sigma(6, 0.02)));
      if (frames_[i].is_static) {
        graph.add(gtsam::PriorFactor<Eigen::Vector3d>(V(i), Eigen::Vector3d::Zero(), zupt_noise));
      }
    }
    if (enable_loop_closure_ && stamp >= loop_closure_after_) {
      const std::size_t last = frames_.size() - 1;
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(last), reference_pose_, loop_pose_noise));
      graph.add(gtsam::PriorFactor<Eigen::Vector3d>(V(last), reference_velocity_, zupt_noise));
    }

    try {
      gtsam::LevenbergMarquardtParams lm_params;
      lm_params.setMaxIterations(8);
      lm_params.setVerbosityLM("SILENT");
      gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
      const gtsam::Values result = optimizer.optimize();
      const std::size_t last = frames_.size() - 1;
      current_pose_ = result.at<gtsam::Pose3>(X(last));
      current_velocity_ = result.at<Eigen::Vector3d>(V(last));
      current_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(last));
      frames_[last].pose = current_pose_;
      frames_[last].velocity = current_velocity_;
      frames_[last].bias = current_bias_;
    } catch (const std::exception & exc) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "GTSAM optimize failed: %s", exc.what());
    }
  }

  void publish(const sensor_msgs::msg::Imu & imu, bool stationary)
  {
    nav_msgs::msg::Odometry odom;
    fillOdom(odom, imu, current_pose_, current_velocity_, current_bias_);
    odom_pub_->publish(odom);
    std_msgs::msg::Bool msg;
    msg.data = stationary;
    zupt_pub_->publish(msg);
  }

  std::string imu_topic_;
  double keyframe_dt_{0.1};
  double gravity_{9.80665};
  double gyro_noise_{0.004};
  double accel_noise_{0.08};
  double gyro_bias_rw_{0.0006};
  double accel_bias_rw_{0.01};
  double static_gyro_threshold_{0.045};
  double static_accel_threshold_{0.22};
  bool enable_loop_closure_{false};
  double loop_closure_after_{45.0};
  bool initialized_{false};
  double last_stamp_{0.0};
  double last_keyframe_stamp_{0.0};
  gtsam::Pose3 current_pose_;
  Eigen::Vector3d current_velocity_{Eigen::Vector3d::Zero()};
  gtsam::imuBias::ConstantBias current_bias_;
  gtsam::Pose3 reference_pose_;
  Eigen::Vector3d reference_velocity_{Eigen::Vector3d::Zero()};
  boost::shared_ptr<gtsam::PreintegrationParams> params_;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> current_pim_;
  std::deque<Keyframe> frames_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr zupt_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GtsamFgoNode>());
  rclcpp::shutdown();
  return 0;
}
