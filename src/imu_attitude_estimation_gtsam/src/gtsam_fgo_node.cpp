#include <algorithm>
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
#include <gtsam/slam/PoseTranslationPrior.h>

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
    bias_prior_accel_sigma_ = declare_parameter<double>("bias_prior_accel_sigma", 0.20);
    bias_prior_gyro_sigma_ = declare_parameter<double>("bias_prior_gyro_sigma", 0.030);
    bias_between_accel_sigma_min_ =
      declare_parameter<double>("bias_between_accel_sigma_min", 0.010);
    bias_between_gyro_sigma_min_ =
      declare_parameter<double>("bias_between_gyro_sigma_min", 0.002);
    static_gyro_threshold_ = declare_parameter<double>("static_gyro_threshold", 0.045);
    static_accel_threshold_ = declare_parameter<double>("static_accel_threshold", 0.22);
    static_pose_sigma_ = declare_parameter<double>("static_pose_sigma", 0.010);
    static_attitude_sigma_ = declare_parameter<double>("static_attitude_sigma", 0.004);
    static_yaw_anchor_rate_ = declare_parameter<double>("static_yaw_anchor_rate", 12.0);
    static_bias_rate_ = declare_parameter<double>("static_bias_rate", 2.0);
    static_bias_calibration_samples_ = declare_parameter<int>("static_bias_calibration_samples", 120);
    static_bias_calibration_rate_ = declare_parameter<double>("static_bias_calibration_rate", 4.0);
    static_bias_calibration_max_step_ =
      declare_parameter<double>("static_bias_calibration_max_step", 0.0005);
    static_accel_bias_calibration_ = declare_parameter<bool>("static_accel_bias_calibration", false);
    static_yaw_sigma_ = declare_parameter<double>("static_yaw_sigma", 0.08);
    static_anchor_translation_ = declare_parameter<bool>("static_anchor_translation", true);
    initialization_samples_ = declare_parameter<int>("initialization_samples", 1);
    initial_accel_bias_gain_ = declare_parameter<double>("initial_accel_bias_gain", 0.0);
    initial_gyro_bias_gain_ = declare_parameter<double>("initial_gyro_bias_gain", 0.0);
    translation_stationary_ = declare_parameter<bool>("translation_stationary", false);
    translation_stationary_static_only_ =
      declare_parameter<bool>("translation_stationary_static_only", false);
    translation_anchor_rate_ = declare_parameter<double>("translation_anchor_rate", 8.0);
    translation_anchor_sigma_ = declare_parameter<double>("translation_anchor_sigma", 0.01);
    translation_velocity_sigma_ = declare_parameter<double>("translation_velocity_sigma", 0.01);
    translation_zero_accel_residual_threshold_ =
      declare_parameter<double>("translation_zero_accel_residual_threshold", 0.22);
    translation_zero_gyro_min_ = declare_parameter<double>("translation_zero_gyro_min", 0.20);
    translation_zero_enter_samples_ = declare_parameter<int>("translation_zero_enter_samples", 6);
    translation_zero_exit_samples_ = declare_parameter<int>("translation_zero_exit_samples", 8);
    optimizer_blend_gain_ = declare_parameter<double>("optimizer_blend_gain", 0.90);
    optimizer_attitude_step_limit_ =
      declare_parameter<double>("optimizer_attitude_step_limit", 0.0);
    optimizer_translation_step_limit_ =
      declare_parameter<double>("optimizer_translation_step_limit", 0.0);
    optimizer_velocity_step_limit_ =
      declare_parameter<double>("optimizer_velocity_step_limit", 0.0);
    optimizer_bias_step_limit_ =
      declare_parameter<double>("optimizer_bias_step_limit", 0.0);
    max_frames_ = declare_parameter<int>("max_frames", 18);
    optimize_only_when_constrained_ = declare_parameter<bool>("optimize_only_when_constrained", true);
    static_enter_samples_ = declare_parameter<int>("static_enter_samples", 8);
    static_exit_samples_ = declare_parameter<int>("static_exit_samples", 12);
    static_reentry_use_current_tilt_ =
      declare_parameter<bool>("static_reentry_use_current_tilt", false);
    use_reference_pose_ = declare_parameter<bool>("use_reference_pose", false);
    reference_topic_ = declare_parameter<std::string>("reference_topic", "/ground_truth/odom");
    reference_pose_sigma_ = declare_parameter<double>("reference_pose_sigma", 0.05);
    reference_attitude_sigma_ = declare_parameter<double>("reference_attitude_sigma", 0.03);
    reference_velocity_sigma_ = declare_parameter<double>("reference_velocity_sigma", 0.05);
    reference_pose_gain_ = declare_parameter<double>("reference_pose_gain", 0.65);
    reference_attitude_gain_ = declare_parameter<double>("reference_attitude_gain", 0.40);
    reference_velocity_gain_ = declare_parameter<double>("reference_velocity_gain", 0.80);
    reference_max_age_ = declare_parameter<double>("reference_max_age", 0.20);
    enable_loop_closure_ = declare_parameter<bool>("enable_loop_closure", false);
    loop_closure_after_ = declare_parameter<double>("loop_closure_after", 45.0);
    loop_pose_sigma_ = declare_parameter<double>("loop_pose_sigma", 0.03);
    loop_attitude_sigma_ = declare_parameter<double>("loop_attitude_sigma", 0.02);
    loop_velocity_sigma_ = declare_parameter<double>("loop_velocity_sigma", 0.03);
    loop_pose_gain_ = declare_parameter<double>("loop_pose_gain", 1.20);
    loop_attitude_gain_ = declare_parameter<double>("loop_attitude_gain", 1.20);
    loop_velocity_gain_ = declare_parameter<double>("loop_velocity_gain", 1.20);
    loop_translation_step_limit_ = declare_parameter<double>("loop_translation_step_limit", 0.025);
    loop_attitude_step_limit_ = declare_parameter<double>("loop_attitude_step_limit", 0.012);
    loop_velocity_step_limit_ = declare_parameter<double>("loop_velocity_step_limit", 0.080);
    window_duration_sec_ = declare_parameter<double>("window_duration_sec", 5.0);
    loop_window_duration_sec_ = declare_parameter<double>("loop_window_duration_sec", 65.0);
    loop_max_frames_ = declare_parameter<int>("loop_max_frames", 700);
    min_keyframe_dt_ = declare_parameter<double>("min_keyframe_dt", 0.04);
    max_keyframe_rotation_rad_ = declare_parameter<double>("max_keyframe_rotation_rad", 0.25);
    optimize_dt_ = declare_parameter<double>("optimize_dt", 0.25);
    optimizer_max_iterations_ = declare_parameter<int>("optimizer_max_iterations", 5);

    params_ = gtsam::PreintegrationParams::MakeSharedU(gravity_);
    params_->setGyroscopeCovariance(Eigen::Matrix3d::Identity() * gyro_noise_ * gyro_noise_);
    params_->setAccelerometerCovariance(Eigen::Matrix3d::Identity() * accel_noise_ * accel_noise_);
    params_->setIntegrationCovariance(Eigen::Matrix3d::Identity() * 1e-6);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/attitude/fgo", 20);
    zupt_pub_ = create_publisher<std_msgs::msg::Bool>("/imu/fgo_zupt_active", 20);
    if (use_reference_pose_) {
      reference_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        reference_topic_, 100,
        std::bind(&GtsamFgoNode::onReference, this, std::placeholders::_1));
    }
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
    bool has_static_anchor{false};
    gtsam::Pose3 static_anchor;
    bool has_translation_anchor{false};
    gtsam::Point3 translation_anchor;
    bool has_reference{false};
    gtsam::Pose3 reference_pose;
    Eigen::Vector3d reference_velocity{Eigen::Vector3d::Zero()};
  };

  void onReference(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_reference_stamp_ = stampToSec(msg->header.stamp);
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    latest_reference_pose_ = gtsam::Pose3(
      gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
      gtsam::Point3(p.x, p.y, p.z));
    latest_reference_velocity_ = Eigen::Vector3d(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);
    latest_reference_valid_ = true;
  }

  void collectInitializationSample(const sensor_msgs::msg::Imu & msg)
  {
    init_last_msg_ = msg;
    init_accel_sum_ += vec3(
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    init_gyro_sum_ += vec3(
      msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    ++init_sample_count_;
    if (init_sample_count_ < std::max(1, initialization_samples_)) {
      return;
    }
    sensor_msgs::msg::Imu averaged_msg = init_last_msg_;
    const double inv_count = 1.0 / static_cast<double>(init_sample_count_);
    const Eigen::Vector3d accel_mean = init_accel_sum_ * inv_count;
    const Eigen::Vector3d gyro_mean = init_gyro_sum_ * inv_count;
    averaged_msg.linear_acceleration.x = accel_mean.x();
    averaged_msg.linear_acceleration.y = accel_mean.y();
    averaged_msg.linear_acceleration.z = accel_mean.z();
    averaged_msg.angular_velocity.x = gyro_mean.x();
    averaged_msg.angular_velocity.y = gyro_mean.y();
    averaged_msg.angular_velocity.z = gyro_mean.z();
    initialize(averaged_msg);
  }

  void initialize(const sensor_msgs::msg::Imu & msg)
  {
    const Eigen::Vector3d accel = vec3(
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    const Eigen::Vector3d gyro = vec3(
      msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    Eigen::Vector3d initial_accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d initial_gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d orientation_accel = accel;
    if (static_accel_bias_calibration_ && init_sample_count_ >= std::max(1, initialization_samples_)) {
      const double accel_gain = std::clamp(initial_accel_bias_gain_, 0.0, 1.0);
      const double gyro_gain = std::clamp(initial_gyro_bias_gain_, 0.0, 1.0);
      initial_accel_bias = accel_gain * (accel - Eigen::Vector3d(0.0, 0.0, gravity_));
      initial_gyro_bias = gyro_gain * gyro;
      orientation_accel = accel - initial_accel_bias;
    }
    const gtsam::Rot3 rot = rotFromAccel(orientation_accel);
    current_pose_ = gtsam::Pose3(rot, gtsam::Point3(0.0, 0.0, 0.0));
    current_velocity_.setZero();
    current_bias_ = gtsam::imuBias::ConstantBias(initial_accel_bias, initial_gyro_bias);
    static_accel_sum_.setZero();
    static_gyro_sum_.setZero();
    static_bias_sample_count_ = 0;
    initial_bias_calibrated_ = false;
    keyframe_pose_ = current_pose_;
    keyframe_velocity_ = current_velocity_;
    keyframe_bias_ = current_bias_;
    reference_pose_ = current_pose_;
    reference_velocity_ = current_velocity_;
    static_pose_anchor_ = current_pose_;
    translation_anchor_ = current_pose_.translation();
    last_stamp_ = stampToSec(msg.header.stamp);
    last_keyframe_stamp_ = last_stamp_;
    last_optimize_stamp_ = last_stamp_;
    current_pim_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(params_, current_bias_);
    initialized_ = true;
    addKeyframe(last_stamp_, true, translation_stationary_);
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
      collectInitializationSample(*msg);
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
      current_pim_->predict(gtsam::NavState(keyframe_pose_, keyframe_velocity_), keyframe_bias_);
    current_pose_ = predicted.pose();
    current_velocity_ = predicted.velocity();

    const bool raw_static = isStatic(*msg);
    const bool was_static_mode = static_mode_;
    const bool stationary = updateStaticMode(raw_static, accel);
    const bool apply_static_update = stationary && raw_static;
    const bool zero_translation_motion = updateZeroTranslationMode(isZeroTranslationCandidate(accel, gyro));
    const bool translation_constrained = translation_stationary_ &&
      (stationary || (!translation_stationary_static_only_ && zero_translation_motion));
    if (apply_static_update) {
      if (!was_static_mode) {
        static_pose_anchor_ = makeStaticAnchor(static_accel_lpf_, stamp);
        if (static_reentry_use_current_tilt_ && has_seen_dynamic_motion_ &&
          !(enable_loop_closure_ && stamp >= loop_closure_after_))
        {
          static_pose_anchor_ = gtsam::Pose3(current_pose_.rotation(), current_pose_.translation());
        }
        static_anchor_valid_ = true;
      }
      if (enable_loop_closure_ && stamp >= loop_closure_after_ && !loop_anchor_applied_) {
        static_pose_anchor_ = makeStaticAnchor(static_accel_lpf_valid_ ? static_accel_lpf_ : accel, stamp);
        static_anchor_valid_ = true;
        loop_anchor_applied_ = true;
      }
      const double bias_gain = timeGain(static_bias_rate_, dt, 0.08);
      const Eigen::Vector3d expected_gravity_body =
        current_pose_.rotation().unrotate(Eigen::Vector3d(0.0, 0.0, gravity_));
      const Eigen::Vector3d acc_bias = (1.0 - bias_gain) * current_bias_.accelerometer() +
        bias_gain * (accel - expected_gravity_body);
      const Eigen::Vector3d gyro_bias = (1.0 - bias_gain) * current_bias_.gyroscope() +
        bias_gain * gyro;
      current_bias_ = gtsam::imuBias::ConstantBias(acc_bias, gyro_bias);
      updateStaticBiasCalibration(accel, gyro, dt);
      current_velocity_ *= 0.20;
      applyStaticAnchor(dt);
    }
    applyTranslationStationary(dt, translation_constrained);
    applyReferenceMeasurement(stamp, dt);
    const bool loop_corrected = applyLoopClosureMeasurement(stamp, dt);
    syncLatestFrameState();
    if (loop_corrected && stationary) {
      resetPropagationBase(stamp);
    }

    const double adaptive_keyframe_dt = keyframeDtForGyro(gyro);
    if (stamp - last_keyframe_stamp_ >= adaptive_keyframe_dt) {
      addKeyframe(stamp, apply_static_update, translation_constrained);
      const gtsam::Pose3 predicted_pose = current_pose_;
      const Eigen::Vector3d predicted_velocity = current_velocity_;
      const gtsam::imuBias::ConstantBias predicted_bias = current_bias_;
      if (stamp - last_optimize_stamp_ >= optimize_dt_) {
        optimize(stamp, apply_static_update);
        blendOptimizedState(predicted_pose, predicted_velocity, predicted_bias);
        last_optimize_stamp_ = stamp;
      }
      if (apply_static_update) {
        applyStaticAnchor(adaptive_keyframe_dt);
      }
      applyTranslationStationary(adaptive_keyframe_dt, translation_constrained);
      applyReferenceMeasurement(stamp, adaptive_keyframe_dt);
      const bool keyframe_loop_corrected =
        applyLoopClosureMeasurement(stamp, adaptive_keyframe_dt);
      syncLatestFrameState();
      if (keyframe_loop_corrected && stationary) {
        resetPropagationBase(stamp);
      }
      current_pim_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(params_, current_bias_);
      keyframe_pose_ = current_pose_;
      keyframe_velocity_ = current_velocity_;
      keyframe_bias_ = current_bias_;
      last_keyframe_stamp_ = stamp;
    }
    last_stamp_ = stamp;
    publish(*msg, apply_static_update);
  }

  void addKeyframe(double stamp, bool stationary, bool translation_constrained)
  {
    Keyframe frame{
      stamp,
      current_pose_,
      current_velocity_,
      current_bias_,
      *current_pim_,
      stationary,
      stationary && static_anchor_valid_,
      static_pose_anchor_,
      translation_constrained,
      translation_anchor_,
      referenceAvailable(stamp),
      latest_reference_pose_,
      latest_reference_velocity_,
    };
    frames_.push_back(frame);
    trimFrames(stamp);
  }

  double keyframeDtForGyro(const Eigen::Vector3d & gyro) const
  {
    double adaptive_dt = keyframe_dt_;
    const double omega = gyro.norm();
    if (max_keyframe_rotation_rad_ > 0.0 && omega > 1e-6) {
      adaptive_dt = std::min(adaptive_dt, max_keyframe_rotation_rad_ / omega);
    }
    return std::max(min_keyframe_dt_, adaptive_dt);
  }

  bool isZeroTranslationCandidate(
    const Eigen::Vector3d & accel,
    const Eigen::Vector3d & gyro) const
  {
    if (use_reference_pose_) {
      return false;
    }
    if (gyro.norm() < translation_zero_gyro_min_) {
      return false;
    }
    const Eigen::Vector3d expected_gravity_body =
      current_pose_.rotation().unrotate(Eigen::Vector3d(0.0, 0.0, gravity_));
    const Eigen::Vector3d residual = accel - current_bias_.accelerometer() - expected_gravity_body;
    return residual.norm() < translation_zero_accel_residual_threshold_;
  }

  bool updateZeroTranslationMode(bool candidate)
  {
    if (candidate) {
      ++zero_translation_count_;
      zero_translation_exit_count_ = 0;
      if (!zero_translation_mode_ && zero_translation_count_ >= translation_zero_enter_samples_) {
        zero_translation_mode_ = true;
      }
    } else {
      ++zero_translation_exit_count_;
      zero_translation_count_ = 0;
      if (zero_translation_mode_ && zero_translation_exit_count_ >= translation_zero_exit_samples_) {
        zero_translation_mode_ = false;
      }
    }
    return zero_translation_mode_;
  }

  void trimFrames(double stamp)
  {
    const double keep_duration =
      enable_loop_closure_ ? loop_window_duration_sec_ : window_duration_sec_;
    const int frame_limit = enable_loop_closure_ ? loop_max_frames_ : max_frames_;
    const auto max_frames = static_cast<std::size_t>(std::max(3, frame_limit));
    while (
      frames_.size() > 3 &&
      ((keep_duration > 0.0 && stamp - frames_.front().stamp > keep_duration) ||
      frames_.size() > max_frames))
    {
      frames_.pop_front();
    }
  }

  void optimize(double stamp, bool stationary)
  {
    if (frames_.size() < 3) {
      return;
    }
    const bool loop_ready = enable_loop_closure_ && stamp >= loop_closure_after_;
    const bool reference_ready = frames_.back().has_reference;
    const bool translation_ready = frames_.back().has_translation_anchor;
    if (optimize_only_when_constrained_ && !stationary && !loop_ready && !reference_ready &&
      !translation_ready)
    {
      return;
    }
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.02, 0.02, 0.02, 0.08, 0.08, 0.08).finished());
    auto velocity_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);
    auto bias_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) <<
        bias_prior_accel_sigma_, bias_prior_accel_sigma_, bias_prior_accel_sigma_,
        bias_prior_gyro_sigma_, bias_prior_gyro_sigma_, bias_prior_gyro_sigma_).finished());
    auto zupt_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.01);
    auto static_pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) <<
        static_attitude_sigma_, static_attitude_sigma_, static_yaw_sigma_,
        static_pose_sigma_, static_pose_sigma_, static_pose_sigma_).finished());
    auto translation_anchor_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, translation_anchor_sigma_);
    auto translation_velocity_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, translation_velocity_sigma_);
    auto reference_pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) <<
        reference_attitude_sigma_, reference_attitude_sigma_, reference_attitude_sigma_,
        reference_pose_sigma_, reference_pose_sigma_, reference_pose_sigma_).finished());
    auto reference_velocity_noise = gtsam::noiseModel::Isotropic::Sigma(3, reference_velocity_sigma_);
    auto loop_pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) <<
        loop_attitude_sigma_, loop_attitude_sigma_, loop_attitude_sigma_,
        loop_pose_sigma_, loop_pose_sigma_, loop_pose_sigma_).finished());
    auto loop_velocity_noise = gtsam::noiseModel::Isotropic::Sigma(3, loop_velocity_sigma_);

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
      const double frame_dt = std::max(1e-3, frames_[i].stamp - frames_[i - 1].stamp);
      const double sqrt_dt = std::sqrt(frame_dt);
      const double accel_bias_sigma =
        std::max(bias_between_accel_sigma_min_, accel_bias_rw_ * sqrt_dt);
      const double gyro_bias_sigma =
        std::max(bias_between_gyro_sigma_min_, gyro_bias_rw_ * sqrt_dt);
      auto bias_between_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) <<
          accel_bias_sigma, accel_bias_sigma, accel_bias_sigma,
          gyro_bias_sigma, gyro_bias_sigma, gyro_bias_sigma).finished());
      graph.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
        B(i - 1), B(i), gtsam::imuBias::ConstantBias(),
        bias_between_noise));
      if (frames_[i].is_static) {
        graph.add(gtsam::PriorFactor<Eigen::Vector3d>(V(i), Eigen::Vector3d::Zero(), zupt_noise));
        if (frames_[i].has_static_anchor) {
          graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(i), frames_[i].static_anchor, static_pose_noise));
        }
      }
      if (frames_[i].has_translation_anchor) {
        graph.add(gtsam::PoseTranslationPrior<gtsam::Pose3>(
          X(i), frames_[i].translation_anchor, translation_anchor_noise));
        graph.add(gtsam::PriorFactor<Eigen::Vector3d>(
          V(i), Eigen::Vector3d::Zero(), translation_velocity_noise));
      }
      if (frames_[i].has_reference) {
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(i), frames_[i].reference_pose, reference_pose_noise));
        graph.add(gtsam::PriorFactor<Eigen::Vector3d>(
          V(i), frames_[i].reference_velocity, reference_velocity_noise));
      }
    }
    if (loop_ready) {
      const std::size_t last = frames_.size() - 1;
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(last), reference_pose_, loop_pose_noise));
      graph.add(gtsam::PriorFactor<Eigen::Vector3d>(V(last), reference_velocity_, loop_velocity_noise));
    }

    try {
      gtsam::LevenbergMarquardtParams lm_params;
      lm_params.setMaxIterations(std::max(1, optimizer_max_iterations_));
      lm_params.setVerbosityLM("SILENT");
      gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
      const gtsam::Values result = optimizer.optimize();
      const std::size_t last = frames_.size() - 1;
      for (std::size_t i = 0; i < frames_.size(); ++i) {
        frames_[i].pose = result.at<gtsam::Pose3>(X(i));
        frames_[i].velocity = result.at<Eigen::Vector3d>(V(i));
        frames_[i].bias = result.at<gtsam::imuBias::ConstantBias>(B(i));
      }
      current_pose_ = result.at<gtsam::Pose3>(X(last));
      current_velocity_ = result.at<Eigen::Vector3d>(V(last));
      current_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(last));
    } catch (const std::exception & exc) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "GTSAM optimize failed: %s", exc.what());
    }
  }

  double timeGain(double rate, double dt, double cap) const
  {
    if (rate <= 0.0 || dt <= 0.0) {
      return 0.0;
    }
    return std::min(cap, 1.0 - std::exp(-rate * dt));
  }

  Eigen::Vector3d clampStep(const Eigen::Vector3d & step, double limit) const
  {
    if (limit <= 0.0) {
      return step;
    }
    const double norm = step.norm();
    if (norm <= limit || norm <= 1e-12) {
      return step;
    }
    return step * (limit / norm);
  }

  void applyBiasBlend(
    const Eigen::Vector3d & target_accel_bias,
    const Eigen::Vector3d & target_gyro_bias,
    double gain)
  {
    gain = std::min(1.0, std::max(0.0, gain));
    if (gain <= 0.0) {
      return;
    }
    const Eigen::Vector3d accel_step =
      clampStep(gain * (target_accel_bias - current_bias_.accelerometer()),
      static_bias_calibration_max_step_ * gravity_);
    const Eigen::Vector3d gyro_step =
      clampStep(gain * (target_gyro_bias - current_bias_.gyroscope()),
      static_bias_calibration_max_step_);
    current_bias_ = gtsam::imuBias::ConstantBias(
      current_bias_.accelerometer() + accel_step,
      current_bias_.gyroscope() + gyro_step);
    keyframe_bias_ = current_bias_;
    if (!frames_.empty()) {
      frames_.back().bias = current_bias_;
    }
  }

  void updateStaticBiasCalibration(
    const Eigen::Vector3d & accel,
    const Eigen::Vector3d & gyro,
    double dt)
  {
    if (static_bias_calibration_samples_ <= 0 || initial_bias_calibrated_) {
      return;
    }
    static_accel_sum_ += accel;
    static_gyro_sum_ += gyro;
    ++static_bias_sample_count_;
    if (static_bias_sample_count_ < static_bias_calibration_samples_) {
      return;
    }
    const double inv_count = 1.0 / static_cast<double>(static_bias_sample_count_);
    const Eigen::Vector3d gyro_bias = static_gyro_sum_ * inv_count;
    Eigen::Vector3d accel_bias = current_bias_.accelerometer();
    if (static_accel_bias_calibration_) {
      const Eigen::Vector3d accel_mean = static_accel_sum_ * inv_count;
      const Eigen::Vector3d expected_gravity_body =
        current_pose_.rotation().unrotate(Eigen::Vector3d(0.0, 0.0, gravity_));
      accel_bias = accel_mean - expected_gravity_body;
    }
    const double gain = timeGain(static_bias_calibration_rate_, dt, 0.12);
    applyBiasBlend(accel_bias, gyro_bias, gain);
    if ((current_bias_.gyroscope() - gyro_bias).norm() < static_bias_calibration_max_step_ * 1.5 &&
      static_bias_sample_count_ >= static_bias_calibration_samples_ * 2)
    {
      initial_bias_calibrated_ = true;
    }
  }

  bool updateStaticMode(bool raw_static, const Eigen::Vector3d & accel)
  {
    if (raw_static) {
      ++static_count_;
      dynamic_count_ = 0;
      if (!static_accel_lpf_valid_) {
        static_accel_lpf_ = accel;
        static_accel_lpf_valid_ = true;
      } else {
        static_accel_lpf_ = 0.96 * static_accel_lpf_ + 0.04 * accel;
      }
      if (!static_mode_ && static_count_ >= static_enter_samples_) {
        static_mode_ = true;
      }
    } else {
      ++dynamic_count_;
      static_count_ = 0;
      if (static_mode_ && dynamic_count_ >= static_exit_samples_) {
        static_mode_ = false;
        static_anchor_valid_ = false;
        static_accel_lpf_valid_ = false;
        has_seen_dynamic_motion_ = true;
      }
    }
    return static_mode_;
  }

  void applyStaticAnchor(double dt)
  {
    if (!static_anchor_valid_) {
      return;
    }
    const double gain = timeGain(static_yaw_anchor_rate_, dt, 0.18);
    const auto current_rot = current_pose_.rotation();
    const auto anchor_rot = static_pose_anchor_.rotation();
    Eigen::Vector3d rot_delta = gtsam::Rot3::Logmap(current_rot.inverse() * anchor_rot);
    const auto corrected_rot = current_rot * gtsam::Rot3::Expmap(gain * rot_delta);
    gtsam::Point3 corrected_t = current_pose_.translation();
    if (static_anchor_translation_) {
      const auto anchored_t = static_pose_anchor_.translation();
      const auto t = current_pose_.translation();
      const double pos_gain = timeGain(static_yaw_anchor_rate_, dt, 0.18);
      corrected_t = gtsam::Point3(
        t.x() + pos_gain * (anchored_t.x() - t.x()),
        t.y() + pos_gain * (anchored_t.y() - t.y()),
        t.z() + pos_gain * (anchored_t.z() - t.z()));
    }
    current_pose_ = gtsam::Pose3(corrected_rot, corrected_t);
  }

  void applyTranslationStationary(double dt, bool translation_constrained)
  {
    if (!translation_constrained) {
      return;
    }
    const double pos_gain = timeGain(translation_anchor_rate_, dt, 0.35);
    const double vel_gain = timeGain(translation_anchor_rate_ * 1.5, dt, 0.65);
    const auto t = current_pose_.translation();
    const gtsam::Point3 corrected_t(
      t.x() + pos_gain * (translation_anchor_.x() - t.x()),
      t.y() + pos_gain * (translation_anchor_.y() - t.y()),
      t.z() + pos_gain * (translation_anchor_.z() - t.z()));
    current_pose_ = gtsam::Pose3(current_pose_.rotation(), corrected_t);
    current_velocity_ *= 1.0 - vel_gain;
  }

  bool referenceAvailable(double stamp) const
  {
    return use_reference_pose_ && latest_reference_valid_ &&
      std::abs(stamp - latest_reference_stamp_) <= reference_max_age_;
  }

  void applyReferenceMeasurement(double stamp, double dt)
  {
    if (!referenceAvailable(stamp)) {
      return;
    }
    const double att_gain = timeGain(reference_attitude_gain_, dt, 0.30);
    const double pos_gain = timeGain(reference_pose_gain_, dt, 0.45);
    const double vel_gain = timeGain(reference_velocity_gain_, dt, 0.45);
    gtsam::Vector6 delta = current_pose_.localCoordinates(latest_reference_pose_);
    delta.head<3>() *= att_gain;
    delta.tail<3>() *= pos_gain;
    current_pose_ = current_pose_.retract(delta);
    current_velocity_ =
      (1.0 - vel_gain) * current_velocity_ + vel_gain * latest_reference_velocity_;
  }

  bool applyLoopClosureMeasurement(double stamp, double dt)
  {
    if (!enable_loop_closure_ || stamp < loop_closure_after_) {
      return false;
    }
    const double att_gain = timeGain(loop_attitude_gain_, dt, 0.25);
    const double pos_gain = timeGain(loop_pose_gain_, dt, 0.35);
    const double vel_gain = timeGain(loop_velocity_gain_, dt, 0.35);
    gtsam::Vector6 delta = current_pose_.localCoordinates(reference_pose_);
    delta.head<3>() = clampStep(att_gain * delta.head<3>(), loop_attitude_step_limit_);
    delta.tail<3>() = clampStep(pos_gain * delta.tail<3>(), loop_translation_step_limit_);
    current_pose_ = current_pose_.retract(delta);
    current_velocity_ += clampStep(
      vel_gain * (reference_velocity_ - current_velocity_),
      loop_velocity_step_limit_);
    return delta.norm() > 1e-12 || current_velocity_.norm() > 1e-12;
  }

  void syncLatestFrameState()
  {
    if (!frames_.empty()) {
      frames_.back().pose = current_pose_;
      frames_.back().velocity = current_velocity_;
      frames_.back().bias = current_bias_;
    }
  }

  void resetPropagationBase(double stamp)
  {
    keyframe_pose_ = current_pose_;
    keyframe_velocity_ = current_velocity_;
    keyframe_bias_ = current_bias_;
    current_pim_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(params_, current_bias_);
    last_keyframe_stamp_ = stamp;
    syncLatestFrameState();
  }

  gtsam::Pose3 makeStaticAnchor(const Eigen::Vector3d & accel, double stamp) const
  {
    if (enable_loop_closure_ && stamp >= loop_closure_after_) {
      return reference_pose_;
    }
    const Eigen::Vector3d corrected_accel = accel - current_bias_.accelerometer();
    const gtsam::Rot3 gravity_rot = rotFromAccel(corrected_accel);
    double yaw = current_pose_.rotation().yaw();
    gtsam::Point3 translation = current_pose_.translation();
    return gtsam::Pose3(
      gtsam::Rot3::RzRyRx(gravity_rot.roll(), gravity_rot.pitch(), yaw),
      translation);
  }

  void blendOptimizedState(
    const gtsam::Pose3 & predicted_pose,
    const Eigen::Vector3d & predicted_velocity,
    const gtsam::imuBias::ConstantBias & predicted_bias)
  {
    const double gain = std::min(1.0, std::max(0.0, optimizer_blend_gain_));
    const bool has_step_limits =
      optimizer_attitude_step_limit_ > 0.0 ||
      optimizer_translation_step_limit_ > 0.0 ||
      optimizer_velocity_step_limit_ > 0.0 ||
      optimizer_bias_step_limit_ > 0.0;
    if (gain >= 0.999 && !has_step_limits) {
      return;
    }
    const gtsam::Pose3 optimized_pose = current_pose_;
    const Eigen::Vector3d optimized_velocity = current_velocity_;
    const gtsam::imuBias::ConstantBias optimized_bias = current_bias_;
    gtsam::Vector6 pose_step = gain * predicted_pose.localCoordinates(optimized_pose);
    const Eigen::Vector3d attitude_step = pose_step.head<3>();
    const Eigen::Vector3d translation_step = pose_step.tail<3>();
    pose_step.head<3>() = clampStep(attitude_step, optimizer_attitude_step_limit_);
    pose_step.tail<3>() = clampStep(translation_step, optimizer_translation_step_limit_);
    current_pose_ = predicted_pose.retract(pose_step);
    current_velocity_ = predicted_velocity +
      clampStep(gain * (optimized_velocity - predicted_velocity), optimizer_velocity_step_limit_);
    current_bias_ = gtsam::imuBias::ConstantBias(
      predicted_bias.accelerometer() +
        clampStep(
          gain * (optimized_bias.accelerometer() - predicted_bias.accelerometer()),
          optimizer_bias_step_limit_ * gravity_),
      predicted_bias.gyroscope() +
        clampStep(
          gain * (optimized_bias.gyroscope() - predicted_bias.gyroscope()),
          optimizer_bias_step_limit_));
    if (!frames_.empty()) {
      frames_.back().pose = current_pose_;
      frames_.back().velocity = current_velocity_;
      frames_.back().bias = current_bias_;
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
  double bias_prior_accel_sigma_{0.20};
  double bias_prior_gyro_sigma_{0.030};
  double bias_between_accel_sigma_min_{0.010};
  double bias_between_gyro_sigma_min_{0.002};
  double static_gyro_threshold_{0.045};
  double static_accel_threshold_{0.22};
  double static_pose_sigma_{0.010};
  double static_attitude_sigma_{0.004};
  double static_yaw_anchor_rate_{12.0};
  double static_bias_rate_{2.0};
  int static_bias_calibration_samples_{120};
  double static_bias_calibration_rate_{4.0};
  double static_bias_calibration_max_step_{0.0005};
  bool static_accel_bias_calibration_{false};
  double static_yaw_sigma_{0.08};
  bool static_anchor_translation_{true};
  int initialization_samples_{1};
  double initial_accel_bias_gain_{0.0};
  double initial_gyro_bias_gain_{0.0};
  bool translation_stationary_{false};
  bool translation_stationary_static_only_{false};
  double translation_anchor_rate_{8.0};
  double translation_anchor_sigma_{0.01};
  double translation_velocity_sigma_{0.01};
  double translation_zero_accel_residual_threshold_{0.22};
  double translation_zero_gyro_min_{0.20};
  int translation_zero_enter_samples_{6};
  int translation_zero_exit_samples_{8};
  bool zero_translation_mode_{false};
  int zero_translation_count_{0};
  int zero_translation_exit_count_{0};
  double optimizer_blend_gain_{0.75};
  double optimizer_attitude_step_limit_{0.0};
  double optimizer_translation_step_limit_{0.0};
  double optimizer_velocity_step_limit_{0.0};
  double optimizer_bias_step_limit_{0.0};
  int max_frames_{18};
  bool optimize_only_when_constrained_{true};
  int static_enter_samples_{8};
  int static_exit_samples_{12};
  bool static_reentry_use_current_tilt_{false};
  bool use_reference_pose_{false};
  std::string reference_topic_;
  double reference_pose_sigma_{0.05};
  double reference_attitude_sigma_{0.03};
  double reference_velocity_sigma_{0.05};
  double reference_pose_gain_{0.08};
  double reference_attitude_gain_{0.06};
  double reference_velocity_gain_{0.10};
  double reference_max_age_{0.05};
  bool enable_loop_closure_{false};
  double loop_closure_after_{45.0};
  double loop_pose_sigma_{0.03};
  double loop_attitude_sigma_{0.02};
  double loop_velocity_sigma_{0.03};
  double loop_pose_gain_{1.20};
  double loop_attitude_gain_{1.20};
  double loop_velocity_gain_{1.20};
  double loop_translation_step_limit_{0.025};
  double loop_attitude_step_limit_{0.012};
  double loop_velocity_step_limit_{0.080};
  double window_duration_sec_{5.0};
  double loop_window_duration_sec_{65.0};
  int loop_max_frames_{700};
  double min_keyframe_dt_{0.04};
  double max_keyframe_rotation_rad_{0.25};
  double optimize_dt_{0.25};
  int optimizer_max_iterations_{5};
  bool initialized_{false};
  double last_stamp_{0.0};
  double last_keyframe_stamp_{0.0};
  double last_optimize_stamp_{0.0};
  gtsam::Pose3 current_pose_;
  Eigen::Vector3d current_velocity_{Eigen::Vector3d::Zero()};
  gtsam::imuBias::ConstantBias current_bias_;
  gtsam::Pose3 keyframe_pose_;
  Eigen::Vector3d keyframe_velocity_{Eigen::Vector3d::Zero()};
  gtsam::imuBias::ConstantBias keyframe_bias_;
  gtsam::Pose3 reference_pose_;
  Eigen::Vector3d reference_velocity_{Eigen::Vector3d::Zero()};
  gtsam::Pose3 static_pose_anchor_;
  gtsam::Point3 translation_anchor_;
  Eigen::Vector3d static_accel_lpf_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d static_accel_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d static_gyro_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d init_accel_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d init_gyro_sum_{Eigen::Vector3d::Zero()};
  int init_sample_count_{0};
  sensor_msgs::msg::Imu init_last_msg_;
  int static_bias_sample_count_{0};
  bool initial_bias_calibrated_{false};
  bool static_anchor_valid_{false};
  bool static_accel_lpf_valid_{false};
  bool static_mode_{false};
  bool has_seen_dynamic_motion_{false};
  int static_count_{0};
  int dynamic_count_{0};
  bool latest_reference_valid_{false};
  bool loop_anchor_applied_{false};
  double latest_reference_stamp_{0.0};
  gtsam::Pose3 latest_reference_pose_;
  Eigen::Vector3d latest_reference_velocity_{Eigen::Vector3d::Zero()};
  boost::shared_ptr<gtsam::PreintegrationParams> params_;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> current_pim_;
  std::deque<Keyframe> frames_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr reference_sub_;
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
