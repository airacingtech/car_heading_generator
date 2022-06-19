// Copyright 2022 Siddharth Saha

#include <chrono>
#include <cmath>

#include "car_heading_generator/car_heading_generator.hpp"

namespace car_heading_generator
{

CarHeadingGenerator::CarHeadingGenerator(const rclcpp::NodeOptions & options)
: Node("car_heading_generator_node", options)
{
  this->declare_parameter<double>("dt");
  gps_subscribers_[0] = std::make_unique<message_filters::Subscriber<gps_msgs::msg::GPSFix>>(this, "gps_front", rmw_qos_profile_sensor_data);
  gps_subscribers_[1] = std::make_unique<message_filters::Subscriber<gps_msgs::msg::GPSFix>>(this, "gps_back", rmw_qos_profile_sensor_data);
  gps_synchronizer_ = std::make_unique<message_filters::Synchronizer<GpsSyncPolicy>>(
    GpsSyncPolicy(10), *gps_subscribers_[0], *gps_subscribers_[1]);
  gps_synchronizer_->registerCallback(
    std::bind(
      &CarHeadingGenerator::callback, this, std::placeholders::_1,
      std::placeholders::_2)
  );

  car_heading_msg_ = std::make_unique<std_msgs::msg::Float32>();

  pub_heading_ = this->create_publisher<std_msgs::msg::Float32>(
    "car_heading", rclcpp::SensorDataQoS());

  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<float>(this->get_parameter("dt").as_double()), [this] {
      timer_callback();
    });
}

void CarHeadingGenerator::callback(const gps_msgs::msg::GPSFix::ConstSharedPtr front_msg, const gps_msgs::msg::GPSFix::ConstSharedPtr back_msg)
{
  latest_front_msg_ = front_msg;
  latest_back_msg_ = back_msg;
  seen_gps_ = true;
}

void CarHeadingGenerator::timer_callback()
{
  if (!seen_gps_)
  {
    return;
  }
  double car_heading = calc_car_heading(latest_back_msg_->latitude, latest_back_msg_->longitude, latest_front_msg_->latitude, latest_front_msg_->longitude);
  car_heading_msg_->data = car_heading;
  pub_heading_->publish(*car_heading_msg_);
}

double CarHeadingGenerator::calc_car_heading(double back_lat, double back_lon, double front_lat, double front_lon)
{
    double back_theta = back_lat * DEGREE_TO_RADIANS;
    double front_theta = front_lat * DEGREE_TO_RADIANS;
    double lon_delta = (front_lon - back_lon) * DEGREE_TO_RADIANS;

    //==================Heading Formula Calculation================//

    double y = std::sin(lon_delta) * std::cos(front_theta);
    double x = (std::cos(back_theta) * std::sin(front_theta)) - (std::sin(back_theta) * std::cos(front_theta) * std::cos(lon_delta));
    double car_heading = std::atan2(y, x);
    car_heading = car_heading * RADIANS_TO_DEGREE;
    car_heading = std::fmod(90.0 - car_heading, 360.0);
    return car_heading * DEGREE_TO_RADIANS;
}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<car_heading_generator::CarHeadingGenerator>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}