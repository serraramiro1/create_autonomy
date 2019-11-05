#include <string> //c++ libraries

//ros libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>

//gazebo libraries
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include "gazebo_plugins/gazebo_ros_camera.h"

//own libraries
#include "ca_gazebo/color_sensor_plugin.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosColor)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosColor::GazeboRosColor() : nh_("color_sensor_plugin"),
                                   _fov(6),
                                   _range(10),
                                   CameraPlugin(),
                                   GazeboRosCameraUtils()

{
}

GazeboRosColor::~GazeboRosColor()
{
  ROS_DEBUG_STREAM_NAMED("camera", "Unloaded");
}

bool GazeboRosColor::IsColor(const my_color_type &target, const my_color_type &color)
{
  return (abs(color[0] - target[0]) <= COLOR_TOLERANCE and
          abs(color[1] - target[1]) <= COLOR_TOLERANCE and
          abs(color[2] - target[2]) <= COLOR_TOLERANCE);
}

void GazeboRosColor::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  CameraPlugin::Load(_parent, _sdf);
  GazeboRosCameraUtils::Load(_parent, _sdf);
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->format_ = this->format;
  this->camera_ = this->camera;
  this->publish_topic_name_ = _sdf->Get<std::string>("publish_topic");
  this->sensor_color_ = _sdf->Get<std::string>("sensor_color");
  this->update_rate_ = _sdf->Get<double>("update_rate");
  sensor_publisher_ = nh_.advertise<std_msgs::Bool>(this->publish_topic_name_, 1);

  this->update_period_ = common::Time(1 / this->update_rate_).Double();
  this->parentSensor_->SetActive(true);
  this->last_update_time_ = this->world_->SimTime();
}

void GazeboRosColor::OnNewFrame(const unsigned char *_image,
                                unsigned int _width, unsigned int _height, unsigned int _depth,
                                const std::string &_format)
{

  const common::Time cur_time = this->world_->SimTime();
  const my_color_type target_color = this->map_of_colors_.at(this->sensor_color_);

  if (cur_time.Double() - this->last_update_time_.Double() >= this->update_period_)
  {
    int count = 0;
    unsigned char starting_pixel = 0;
    std_msgs::BoolPtr msg(new std_msgs::Bool);
    this->last_update_time_ = cur_time;
    my_color_type rgb;

    for (int pixel = 0; pixel < _width * _height; pixel++)
    {

      rgb[0] = _image[starting_pixel];
      rgb[1] = _image[starting_pixel + 1];
      rgb[2] = _image[starting_pixel + 2];

      if (this->IsColor(target_color, rgb))
      {
        count++;
      }

      starting_pixel += 3;
    }
    msg->data = count > PIXEL_THRESHOLD;
    sensor_publisher_.publish(msg);
  }
}
} // namespace gazebo
