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
  this->declare_parameter<std::string>("frame_id");
  this->declare_parameter<double>("heading_scale");
  heading_scale_ = this->get_parameter("heading_scale").as_double();
  this->declare_parameter<double>("heading_offset");
  heading_offset_ = this->get_parameter("heading_offset").as_double();
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
  car_heading_stamped_msg_ = std::make_unique<geometry_msgs::msg::QuaternionStamped>();
  car_heading_stamped_msg_->header.frame_id = this->get_parameter("frame_id").as_string();

  pub_heading_ = this->create_publisher<std_msgs::msg::Float32>(
    "car_heading", rclcpp::SensorDataQoS());
  pub_heading_stamped_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
    "car_heading_stamped", rclcpp::SensorDataQoS());

  callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CarHeadingGenerator::parametersCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult CarHeadingGenerator::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  for (const auto & param: parameters)
  {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      if (param.get_name() == "heading_offset")
      {
        heading_offset_ = param.as_double();
        result.successful = true;
      }
      else if (param.get_name() == "heading_scale")
      {
        heading_scale_ = param.as_double();
        result.successful = true;
      }
    }
    else if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
    {
      if (param.get_name() == "frame_id")
      {
        car_heading_stamped_msg_->header.frame_id = param.as_string();
        result.successful = true;
      }
    }
  }
  result.reason = result.successful ? "success" : "failure";
  return result;
}

void CarHeadingGenerator::callback(const gps_msgs::msg::GPSFix::ConstSharedPtr front_msg, const gps_msgs::msg::GPSFix::ConstSharedPtr back_msg)
{
  car_heading_stamped_msg_->header.stamp = now();
  latest_front_msg_ = front_msg;
  latest_back_msg_ = back_msg;
  double car_heading = calc_car_heading(latest_back_msg_->latitude, latest_back_msg_->longitude, latest_front_msg_->latitude, latest_front_msg_->longitude);
  car_heading = std::fmod((car_heading * heading_scale_) + (heading_offset_ * DEGREE_TO_RADIANS), 2 * M_PI);
  quat_.setRPY(0, 0, car_heading);
  car_heading_msg_->data = car_heading;
  car_heading_stamped_msg_->quaternion = tf2::toMsg(quat_);
  pub_heading_->publish(*car_heading_msg_);
  pub_heading_stamped_->publish(*car_heading_stamped_msg_);
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
    car_heading = std::fmod(M_PI_2 - car_heading, 2 * M_PI);
    return car_heading;
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