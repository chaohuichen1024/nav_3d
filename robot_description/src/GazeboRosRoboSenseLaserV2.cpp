/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019-2020, ENSTA.
 *  All rights reserved.
 *
 *  基于 GazeboRosRoboSenseLaser.cpp 修改, 适配 fast_livo2 的
 *  robosense_ros::Point 字段格式 (timestamp=FLOAT64@16, ring=UINT16@24)
 *********************************************************************/

// Use the same source code for CPU and GPU plugins
#ifndef GAZEBO_GPU_RAY
#define GAZEBO_GPU_RAY 0
#endif

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>

#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#if GAZEBO_GPU_RAY
#include <gazebo/plugins/GpuRayPlugin.hh>
#else
#include <gazebo/plugins/RayPlugin.hh>
#endif

#include <boost/algorithm/string/trim.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

#include <algorithm>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <gazebo/common/Exception.hh>
#if GAZEBO_GPU_RAY
#include <gazebo/sensors/GpuRaySensor.hh>
#else
#include <gazebo/sensors/RaySensor.hh>
#endif
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/Node.hh>

#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <cmath>

#if GAZEBO_GPU_RAY
#define RayPlugin GpuRayPlugin
#define RaySensor GpuRaySensor
#define RaySensorPtr GpuRaySensorPtr
#define STR_Gpu  "Gpu"
#define STR_GPU_ "GPU "
#else
#define STR_Gpu  ""
#define STR_GPU_ ""
#endif

namespace gazebo
{

class GazeboRosRobosenseLaserV2 : public RayPlugin
{
  public: GazeboRosRobosenseLaserV2() : nh_(NULL), gaussian_noise_(0), min_range_(0), max_range_(0) {}

  public: ~GazeboRosRobosenseLaserV2()
  {
    laser_queue_.clear();
    laser_queue_.disable();
    if (nh_) {
      nh_->shutdown();
      delete nh_;
      nh_ = NULL;
    }
    callback_laser_queue_thread_.join();
  }

  public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    RayPlugin::Load(_parent, _sdf);

    gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gazebo_node_->Init();

#if GAZEBO_MAJOR_VERSION >= 7
    parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#else
    parent_ray_sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#endif
    if (!parent_ray_sensor_) {
      gzthrow("GazeboRosRobosense" << STR_Gpu << "LaserV2 controller requires a " << STR_Gpu << "Ray Sensor as its parent");
    }

    robot_namespace_ = "/";
    if (_sdf->HasElement("robotNamespace")) {
      robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    if (!_sdf->HasElement("frameName")) {
      ROS_INFO("Robosense laser plugin missing <frameName>, defaults to /world");
      frame_name_ = "/world";
    } else {
      frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
    }

    if (!_sdf->HasElement("min_range")) {
      ROS_INFO("Robosense laser plugin missing <min_range>, defaults to 0");
      min_range_ = 0;
    } else {
      min_range_ = _sdf->GetElement("min_range")->Get<double>();
    }

    if (!_sdf->HasElement("max_range")) {
      ROS_INFO("Robosense laser plugin missing <max_range>, defaults to infinity");
      max_range_ = INFINITY;
    } else {
      max_range_ = _sdf->GetElement("max_range")->Get<double>();
    }

    min_intensity_ = std::numeric_limits<double>::lowest();
    if (!_sdf->HasElement("min_intensity")) {
      ROS_INFO("Robosense laser plugin missing <min_intensity>, defaults to no clipping");
    } else {
      min_intensity_ = _sdf->GetElement("min_intensity")->Get<double>();
    }

    if (!_sdf->HasElement("topicName")) {
      ROS_INFO("Robosense laser plugin missing <topicName>, defaults to /points");
      topic_name_ = "/points";
    } else {
      topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
    }

    if (!_sdf->HasElement("gaussianNoise")) {
      ROS_INFO("Robosense laser plugin missing <gaussianNoise>, defaults to 0.0");
      gaussian_noise_ = 0;
    } else {
      gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();
    }

    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    nh_ = new ros::NodeHandle(robot_namespace_);

    std::string prefix;
    nh_->getParam(std::string("tf_prefix"), prefix);
    if (robot_namespace_ != "/") {
      prefix = robot_namespace_;
    }
    boost::trim_right_if(prefix, boost::is_any_of("/"));
    frame_name_ = tf::resolve(prefix, frame_name_);

    if (topic_name_ != "") {
      ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
          topic_name_, 1,
          boost::bind(&GazeboRosRobosenseLaserV2::ConnectCb, this),
          boost::bind(&GazeboRosRobosenseLaserV2::ConnectCb, this),
          ros::VoidPtr(), &laser_queue_);
      pub_ = nh_->advertise(ao);
    }

    parent_ray_sensor_->SetActive(false);

    callback_laser_queue_thread_ = boost::thread( boost::bind( &GazeboRosRobosenseLaserV2::laserQueueThread,this ) );

#if GAZEBO_MAJOR_VERSION >= 7
    ROS_INFO("Robosense V2 %slaser plugin ready, %i lasers", STR_GPU_, parent_ray_sensor_->VerticalRangeCount());
#else
    ROS_INFO("Robosense V2 %slaser plugin ready, %i lasers", STR_GPU_, parent_ray_sensor_->GetVerticalRangeCount());
#endif
  }

  private: void ConnectCb()
  {
    boost::lock_guard<boost::mutex> lock(lock_);
    if (pub_.getNumSubscribers()) {
      if (!sub_) {
#if GAZEBO_MAJOR_VERSION >= 7
        sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(), &GazeboRosRobosenseLaserV2::OnScan, this);
#else
        sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->GetTopic(), &GazeboRosRobosenseLaserV2::OnScan, this);
#endif
      }
      parent_ray_sensor_->SetActive(true);
    } else {
#if GAZEBO_MAJOR_VERSION >= 7
      if (sub_) {
        sub_->Unsubscribe();
        sub_.reset();
      }
#endif
      parent_ray_sensor_->SetActive(false);
    }
  }

  private: void OnScan(ConstLaserScanStampedPtr& _msg)
  {
#if GAZEBO_MAJOR_VERSION >= 7
    const ignition::math::Angle maxAngle = parent_ray_sensor_->AngleMax();
    const ignition::math::Angle minAngle = parent_ray_sensor_->AngleMin();
    const double maxRange = parent_ray_sensor_->RangeMax();
    const double minRange = parent_ray_sensor_->RangeMin();
    const int rayCount = parent_ray_sensor_->RayCount();
    const int rangeCount = parent_ray_sensor_->RangeCount();
    const int verticalRayCount = parent_ray_sensor_->VerticalRayCount();
    const int verticalRangeCount = parent_ray_sensor_->VerticalRangeCount();
    const ignition::math::Angle verticalMaxAngle = parent_ray_sensor_->VerticalAngleMax();
    const ignition::math::Angle verticalMinAngle = parent_ray_sensor_->VerticalAngleMin();
#else
    math::Angle maxAngle = parent_ray_sensor_->GetAngleMax();
    math::Angle minAngle = parent_ray_sensor_->GetAngleMin();
    const double maxRange = parent_ray_sensor_->GetRangeMax();
    const double minRange = parent_ray_sensor_->GetRangeMin();
    const int rayCount = parent_ray_sensor_->GetRayCount();
    const int rangeCount = parent_ray_sensor_->GetRangeCount();
    const int verticalRayCount = parent_ray_sensor_->GetVerticalRayCount();
    const int verticalRangeCount = parent_ray_sensor_->GetVerticalRangeCount();
    const math::Angle verticalMaxAngle = parent_ray_sensor_->GetVerticalAngleMax();
    const math::Angle verticalMinAngle = parent_ray_sensor_->GetVerticalAngleMin();
#endif

    const double yDiff = maxAngle.Radian() - minAngle.Radian();
    const double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

    const double MIN_RANGE = std::max(min_range_, minRange);
    const double MAX_RANGE = std::min(max_range_, maxRange);
    const double MIN_INTENSITY = min_intensity_;

    // Populate message fields
    const uint32_t POINT_STEP = 32;
    sensor_msgs::PointCloud2 msg;
    msg.header.frame_id = frame_name_;
    msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());

    // 字段布局适配 fast_livo2 的 robosense_ros::Point:
    //   (float,x)(float,y)(float,z)(float,intensity)(double,timestamp)(uint16_t,ring)
    // 内存布局:
    //   offset 0 : x        (FLOAT32)
    //   offset 4 : y        (FLOAT32)
    //   offset 8 : z        (FLOAT32)
    //   offset 12: intensity(FLOAT32)
    //   offset 16: timestamp(FLOAT64, 8字节)
    //   offset 24: ring     (UINT16)
    //   offset 26: padding  (6字节, 凑齐 POINT_STEP=32)
    msg.fields.resize(6);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.fields[3].name = "intensity";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[3].count = 1;

    msg.fields[4].name = "timestamp";
    msg.fields[4].offset = 16;
    msg.fields[4].datatype = sensor_msgs::PointField::FLOAT64;
    msg.fields[4].count = 1;

    msg.fields[5].name = "ring";
    msg.fields[5].offset = 24;
    msg.fields[5].datatype = sensor_msgs::PointField::UINT16;
    msg.fields[5].count = 1;

    msg.data.resize(verticalRangeCount * rangeCount * POINT_STEP);

    int i, j;
    uint8_t *ptr = msg.data.data();

    // Point timestamps are offsets from the beginning of the scan. Derive the
    // span from the Gazebo sensor instead of hard-coding 20 Hz: the simulated
    // RS16 is configured at 10 Hz, so one scan spans 0.1 s.
    const double update_rate = parent_ray_sensor_->UpdateRate();
    const double scan_duration = update_rate > 0.0 ? 1.0 / update_rate : 0.1;

    for (i = 0; i < rangeCount; i++) {
        double time_offset = (rangeCount > 1) ?
            (static_cast<double>(i) / (rangeCount - 1)) * scan_duration : 0.0;

        for (j = 0; j < verticalRangeCount; j++) {
            double r = _msg->scan().ranges(i + j * rangeCount);
            double intensity = _msg->scan().intensities(i + j * rangeCount);

            if (std::isnan(r) || std::isnan(intensity) ||
                (MIN_RANGE >= r) || (r >= MAX_RANGE) ||
                (intensity < MIN_INTENSITY)) {
                continue;
            }

            if (gaussian_noise_ != 0.0) {
                r += gaussianKernel(0, gaussian_noise_);
            }

            double yAngle;
            double pAngle;

            if (rangeCount > 1) {
                yAngle = i * yDiff / (rangeCount - 1) + minAngle.Radian();
            } else {
                yAngle = minAngle.Radian();
            }

            if (verticalRayCount > 1) {
                pAngle = j * pDiff / (verticalRangeCount - 1) + verticalMinAngle.Radian();
            } else {
                pAngle = verticalMinAngle.Radian();
            }

            if ((MIN_RANGE < r) && (r < MAX_RANGE)) {
                *((float*)(ptr + 0)) = r * cos(pAngle) * cos(yAngle);
                *((float*)(ptr + 4)) = r * cos(pAngle) * sin(yAngle);
#if GAZEBO_MAJOR_VERSION > 2
                *((float*)(ptr + 8)) = r * sin(pAngle);
#else
                *((float*)(ptr + 8)) = -r * sin(pAngle);
#endif
                *((float*)(ptr + 12)) = intensity;

                // 写入时间戳 (double, 相对帧开始的时间偏移)
                *((double*)(ptr + 16)) = time_offset;

#if GAZEBO_MAJOR_VERSION > 2
                *((uint16_t*)(ptr + 24)) = j; // ring
#else
                *((uint16_t*)(ptr + 24)) = verticalRangeCount - 1 - j; // ring
#endif

                ptr += POINT_STEP;
            }
        }
    }

    msg.point_step = POINT_STEP;
    msg.row_step = ptr - msg.data.data();
    msg.height = 1;
    msg.width = msg.row_step / POINT_STEP;
    msg.is_bigendian = false;
    msg.is_dense = true;
    msg.data.resize(msg.row_step);

    pub_.publish(msg);
  }

  private: void laserQueueThread()
  {
    while (nh_->ok()) {
      laser_queue_.callAvailable(ros::WallDuration(0.01));
    }
  }

  private: static double gaussianKernel(double mu, double sigma)
  {
    double U = (double)rand() / (double)RAND_MAX;
    double V = (double)rand() / (double)RAND_MAX;
    return sigma * (sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V)) + mu;
  }

  private: sensors::RaySensorPtr parent_ray_sensor_;
  private: ros::NodeHandle* nh_;
  private: ros::Publisher pub_;
  private: std::string topic_name_;
  private: std::string frame_name_;
  private: double min_intensity_;
  private: double min_range_;
  private: double max_range_;
  private: double gaussian_noise_;
  private: boost::mutex lock_;
  private: std::string robot_namespace_;
  private: ros::CallbackQueue laser_queue_;
  private: boost::thread callback_laser_queue_thread_;
  private: gazebo::transport::NodePtr gazebo_node_;
  private: gazebo::transport::SubscriberPtr sub_;
};

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosRobosenseLaserV2)

} // namespace gazebo
