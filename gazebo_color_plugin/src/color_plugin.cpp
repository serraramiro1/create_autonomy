#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/console.h>
#include "gazebo_color_sensor_plugin/color_sensor_plugin.h"

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>
#include <stdlib.h>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <std_msgs/Bool.h>

#include <sensor_msgs/Illuminance.h>

#define COLOR_TOLERANCE 30

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosColor)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboRosColor::GazeboRosColor():
  _nh("color_sensor_plugin"),
  _fov(6),
  _range(10),
  CameraPlugin(),
  GazeboRosCameraUtils()

  {
    
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosColor::~GazeboRosColor()
  {
    ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
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
    // copying from CameraPlugin into GazeboRosCameraUtils
    GazeboRosCameraUtils::Load(_parent, _sdf);
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;
    this->publish_topic_name_ = _sdf-> Get<std::string>("publishTopic");
    this->sensor_color_ = _sdf-> Get<std::string>("sensorColor");
    //this->update_rate_ = atof(_sdf-> Get<std::string>("update_rate").c_str());
    _sensorPublisher = _nh.advertise<std_msgs::Bool>(this->publish_topic_name_, 1);
    
    this->parentSensor_->SetActive(true);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboRosColor::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
  {
    int count=0;

    std_msgs::Bool msg;
    unsigned char startingPixel=0;
    unsigned char r,g,b;
    common::Time cur_time = this->world_->SimTime();
    this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();
    this->update_period_=common::Time(1,0).Double();
    if (cur_time.Double() - this->last_update_time_.Double() >= this->update_period_){
      ROS_INFO("ENTRE");
      this->last_update_time_=cur_time;
      this->PutCameraData(_image);
      this->PublishCameraInfo();

    for (int i=0;i<_width;i++)
    {
      for(int j=0;j<_height;j++)
      {
        r=_image[startingPixel];
        g=_image[startingPixel+(unsigned char) 1];
        b=_image[startingPixel+(unsigned char) 2];
        
      if (this->sensor_color_=="yellow")
      {
        if(r >= (255-COLOR_TOLERANCE) && (g >= 255-COLOR_TOLERANCE) && (b<=2*COLOR_TOLERANCE))
        {
          count++;
        }
      }else if (this->sensor_color_=="white")
      {
        if (r >= (255-COLOR_TOLERANCE) && (g >= 255-COLOR_TOLERANCE) && (b>=255-COLOR_TOLERANCE))
        {
          count++;
        }
      }
      startingPixel+=(unsigned char) 3;
      }
    }
    if (count>100)
    {
      msg.data=true;
    }else{
      msg.data=false;
    }
    msg.data=count;
    _sensorPublisher.publish(msg);
    this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();
    }
    
  }
}
