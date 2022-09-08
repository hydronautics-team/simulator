#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <opencv4/opencv2/core/core.hpp>

#include "tile_depthmeter/tile_depthmeter_class.hpp"
#include "monocular_odometry/pose_estimator_class.hpp"

class MessageSubscriber
{
  public:
  MessageSubscriber(ros::NodeHandle& nh, string img_topic, string imu_topic, string camera_info_topic); 
  cv::Mat frame_; 
  std::vector<double> camera_info_; 
  std::vector<double> quat_; 
  std::vector<double> ang_vel_; 
  ros::Time imu_msg_time_;

  private:
  void ImageReceivedCallback(const sensor_msgs::ImageConstPtr& msg);
  void CameraInfoCallback(const sensor_msgs::CameraInfo& msg);
  void IMUDataReceivedCallback(const sensor_msgs::Imu& msg);
  image_transport::ImageTransport image_transporter_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_; 
  ros::Subscriber imu_sub_; 
};

MessageSubscriber::MessageSubscriber(ros::NodeHandle& nh, string img_topic, string imu_topic, string camera_info_topic):image_transporter_(nh)
{
  image_transporter_ = image_transport::ImageTransport(nh);
  image_sub_ = image_transporter_.subscribe(img_topic, 2, &MessageSubscriber::ImageReceivedCallback, this);
  imu_sub_ = nh.subscribe(imu_topic, 3, &MessageSubscriber::IMUDataReceivedCallback, this); 
  camera_info_sub_ = nh.subscribe(camera_info_topic, 1,  &MessageSubscriber::CameraInfoCallback, this); 
}

void MessageSubscriber::ImageReceivedCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
  frame_ = cv_bridge::toCvShare(img_msg, "bgr8")->image; 
  imu_msg_time_ = ros::Time::now(); 
}

void MessageSubscriber::CameraInfoCallback(const sensor_msgs::CameraInfo& msg)
{
  camera_info_.clear();
  for(int i = 0; i < msg.K.size(); i++)
  {
    camera_info_.push_back(msg.K[i]);
  }
}

void MessageSubscriber::IMUDataReceivedCallback(const sensor_msgs::Imu& msg)
{
  quat_ = vector<double>(4, 0);
  quat_[0] = msg.orientation.w;
  quat_[1] = msg.orientation.x;
  quat_[2] = msg.orientation.y;
  quat_[3] = msg.orientation.z;

  ang_vel_ = vector<double>(3, 0); 
  ang_vel_[0] = msg.angular_velocity.x;
  ang_vel_[1] = msg.angular_velocity.y;
  ang_vel_[2] = msg.angular_velocity.z;
}

class MessagePublisher
{
  public:
  MessagePublisher(ros::NodeHandle& nh);
  void SetOdomMsg(const std::vector<double>& pose, 
                  const std::vector<double>& vels, 
                  const std::vector<double>& angv,
                  const std::vector<double>& quat, 
                  const ros::Time crnt_time);
  void PublishMessages();

  private:
  ros::Publisher odom_pub_;
  nav_msgs::Odometry odom_msg_; 
};

MessagePublisher::MessagePublisher(ros::NodeHandle& nh)
{
  odom_pub_ = nh.advertise<geometry_msgs::Vector3>("tile_dpos", 2);
}

void MessagePublisher::SetOdomMsg(const std::vector<double>& pose, 
                                  const std::vector<double>& vels, 
                                  const std::vector<double>& angv,  
                                  const std::vector<double>& quat, 
                                  const ros::Time crnt_time)
{
  odom_msg_.header.stamp = crnt_time; 
  odom_msg_.header.frame_id = "tile_odom";

  odom_msg_.pose.pose.position.x = pose[0];
  odom_msg_.pose.pose.position.y = pose[1];
  odom_msg_.pose.pose.position.z = pose[2];

  odom_msg_.child_frame_id = "base_link";
  odom_msg_.twist.twist.linear.x = vels[0];
  odom_msg_.twist.twist.linear.y = vels[1];
  odom_msg_.twist.twist.linear.z = vels[0];

  odom_msg_.twist.twist.angular.x = angv[0];
  odom_msg_.twist.twist.angular.y = angv[1];
  odom_msg_.twist.twist.angular.z = angv[2];

  odom_msg_.pose.pose.orientation.w = quat[0];
  odom_msg_.pose.pose.orientation.x = quat[1];
  odom_msg_.pose.pose.orientation.y = quat[2];
  odom_msg_.pose.pose.orientation.z = quat[3]; 
}

void MessagePublisher::PublishMessages()
{
  odom_pub_.publish(odom_msg_);
}

void CalcNewPos(const std::vector<double>& dxyz, std::vector<double>& new_pos, double scale)
{
  for(int i = 0; i < dxyz.size(); i++)
  {
    new_pos[i] += dxyz[i] * scale; 
  }
}

void CalcNewVel(const std::vector<double>& dxyz, std::vector<double>& new_vels, double dt)
{
  for(int i = 0; i < dxyz.size(); i++)
  {
    new_vels[i] = dxyz[i] / dt; 
  }
}

void CalcNewAng(const std::vector<double>& dang, std::vector<double>& new_ang)
{
  for(int i = 0; i < dang.size(); i++)
  {
    new_ang[i] += dang[i];
  }
}

void CalcNewAngVels(const std::vector<double>& dang, std::vector<double>& new_ang_vels, double dt)
{
  for(int i = 0; i < dang.size(); i++)
  {
    new_ang_vels[i] = dang[i] / dt; 
  }
}

void CalcNewQuat(const std::vector<double>& dquat, std::vector<double>& new_quat)
{
  new_quat[0] = new_quat[0]*dquat[0] - new_quat[1]*dquat[1] - new_quat[2]*dquat[2] - new_quat[3]*dquat[3];
  new_quat[1] = new_quat[0]*dquat[1] + new_quat[1]*dquat[0] + new_quat[2]*dquat[3] - new_quat[3]*dquat[2];
  new_quat[2] = new_quat[0]*dquat[2] - new_quat[1]*dquat[3] + new_quat[2]*dquat[0] + new_quat[3]*dquat[1];
  new_quat[3] = new_quat[0]*dquat[3] + new_quat[1]*dquat[2] - new_quat[2]*dquat[1] + new_quat[3]*dquat[0];
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tile_odometer");

  ros::NodeHandle node_handler;

  string image_topic, imu_topic, camera_info_topic;
  node_handler.getParam("image_topic", image_topic); 
  node_handler.getParam("camera_info_topic", camera_info_topic); 
  node_handler.getParam("imu_topic", imu_topic); 

  int tile_width, tile_height;
  node_handler.getParam("tile_width", tile_width); 
  node_handler.getParam("tile_height", tile_height); 

  float water_level; 
  node_handler.getParam("water_level", water_level); 

  MessageSubscriber subscriber(node_handler, image_topic, imu_topic, camera_info_topic); 
  MessagePublisher publisher(node_handler); 

  PoseEstimator estimator; 
  TileDepthmeter depthmeter; 

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  double time_delta; 

  ros::spinOnce();
  estimator.SetPrevFrame(subscriber.frame_); 
  depthmeter.SetCameraIntrinsics(subscriber.camera_info_); 

  std::vector<double> current_position(3, 0);
  std::vector<double> current_velocities(3, 0);
  std::vector<double> current_angle(3, 0);
  std::vector<double> current_angular_velocities(3, 0);
  std::vector<double> current_quaternion(4, 0); 

  ros::Rate rate(30);
  while(node_handler.ok()){
    ros::spinOnce();
    current_time = ros::Time::now();
    time_delta = (current_time - last_time).toSec();

    depthmeter.SetFrame(subscriber.frame_); 
    depthmeter.CalcScale();
    depthmeter.CalcDistance();

    estimator.SetNextFrame(subscriber.frame_); 
    estimator.CalculateRotation(); 
    if((current_time - subscriber.imu_msg_time_).toSec() < 30/1)
    {
      estimator.SetRMatFromQ(subscriber.quat_);
      current_angle = estimator.GetYPR(); 
      current_angular_velocities = subscriber.ang_vel_; 
      current_quaternion = subscriber.quat_;
    }
    else 
    {
      CalcNewAng(estimator.GetYPR(), current_angle);
      CalcNewAngVels(estimator.GetYPR(), current_angular_velocities, time_delta);
      CalcNewQuat(estimator.GetQ(), current_quaternion);
    }

    estimator.CalculateTranslation(); 
    depthmeter.SetFrame(subscriber.frame_);
    depthmeter.CalcScale();
    CalcNewPos(estimator.GetT(), current_position, depthmeter.scale_factor_);
    CalcNewVel(estimator.GetT(), current_velocities, time_delta);

    publisher.SetOdomMsg(current_position, 
                         current_velocities, 
                         current_angular_velocities, 
                         current_quaternion, 
                         current_time); 
    publisher.PublishMessages(); 
  
    last_time = current_time;
    rate.sleep();
  }
}