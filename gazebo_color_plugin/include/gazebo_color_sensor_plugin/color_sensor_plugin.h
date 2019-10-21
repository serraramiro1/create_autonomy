 #ifndef GAZEBO_ROS_COLOR_SENSOR_HH
    #define GAZEBO_ROS_COLOR_SENSOR_HH
    
    #include <string>
    
    // library for processing camera data for gazebo / ros conversions
    #include <gazebo/plugins/CameraPlugin.hh>
    
    #include <gazebo_plugins/gazebo_ros_camera_utils.h>
    
    namespace gazebo
    {
      class GazeboRosColor : public CameraPlugin, GazeboRosCameraUtils
      {
        /// \brief Constructor
        /// \param parent The parent entity, must be a Model or a Sensor
        public: GazeboRosColor();
    
        /// \brief Destructor
        public: ~GazeboRosColor();
    
        /// \brief Load the plugin
        /// \param take in SDF root element
        public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
    
        /// \brief Update the controller
        protected: virtual void OnNewFrame(const unsigned char *_image,
        unsigned int _width, unsigned int _height,
        unsigned int _depth, const std::string &_format);
    
        ros::NodeHandle _nh;
        ros::Publisher _sensorPublisher;
        std::string sensor_color_;
        std::string publish_topic_name_;
        double _fov;
        double _range;
      };
    }
    #endif
