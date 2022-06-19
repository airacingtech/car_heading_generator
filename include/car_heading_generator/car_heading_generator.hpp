// Copyright 2022 Siddharth Saha

#ifndef CAR_HEADING_GENERATOR_HPP_
#define CAR_HEADING_GENERATOR_HPP_

#include "rclcpp/rclcpp.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "gps_msgs/msg/gps_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace car_heading_generator
{

class CarHeadingGenerator : public rclcpp::Node
{
public:
  explicit CarHeadingGenerator(const rclcpp::NodeOptions & options);
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

private:
  using GpsSyncPolicy = message_filters::sync_policies::ApproximateTime<gps_msgs::msg::GPSFix, gps_msgs::msg::GPSFix>;

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  void callback(const gps_msgs::msg::GPSFix::ConstSharedPtr front_msg, const gps_msgs::msg::GPSFix::ConstSharedPtr back_msg);
  void timer_callback();
  double calc_car_heading(double back_lat, double back_lon, double front_lat, double front_lon);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_heading_stamped_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_heading_;
  std::unique_ptr<message_filters::Subscriber<gps_msgs::msg::GPSFix>> gps_subscribers_[2];
  std::unique_ptr<message_filters::Synchronizer<GpsSyncPolicy>> gps_synchronizer_;

  gps_msgs::msg::GPSFix::ConstSharedPtr latest_front_msg_;
  gps_msgs::msg::GPSFix::ConstSharedPtr latest_back_msg_;

  std_msgs::msg::Float32::SharedPtr car_heading_msg_;
  geometry_msgs::msg::PoseStamped::SharedPtr car_heading_stamped_msg_;
  
  bool seen_gps_{false};
  double heading_scale_;
  double heading_offset_;
  tf2::Quaternion quat_;

  const double DEGREE_TO_RADIANS = M_PI/180.0;
  const double RADIANS_TO_DEGREE = 180.0/M_PI;
};

}  // namespace car_heading_generator

#endif  // CAR_HEADING_GENERATOR_HPP_
