#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Quaternion.h>
#include "AntSystem.h"
#include <boost/circular_buffer.hpp>
#include <numeric>
#include <math.h>
#include <std_msgs/Float32.h>
#include "boost/atomic.hpp"
#include <string>     // std::string, std::stoi

using namespace tf;

// last Position Orientation
struct CamPose
{
  float px;
  float py;
  float pz;
  float ox;
  float oy;
  float oz;
  float ow;
  tf::Quaternion  rpy;
  ros::Time lastTime;
};


class PathDetector {

private:

  ros::NodeHandle node;

  sensor_msgs::Image rosImage;

  bool useLSDMask;


  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void poseCallback(const geometry_msgs::PoseStampedPtr& msg);
  void lsdSlamCallback(const sensor_msgs::ImageConstPtr& msg);
  void lsdEstVelCallback(const std_msgs::Float32Ptr& msg);
  void lsdPlaneSlopeCallback(const std_msgs::Float32Ptr& msg);

  cv_bridge::CvImage cvImage;
  cv::Mat frame;

  ros::NodeHandlePtr nh;

  int frame_count;
  image_transport::ImageTransport* it;
  image_transport::Publisher pubImg;
  image_transport::Publisher pubPathTrace;

  image_transport::Publisher pubNeuralField;
  image_transport::Publisher pubTrailProbMap;
  image_transport::Publisher pubTDSalMap;
  image_transport::Publisher pubTDConspCMap;

  std::string sub_image_topic;
  std::string sub_camera_pose_topic;
  std::string sub_lsd_mask_topic;
  std::string sub_lsd_vel_est_topic;
  std::string sub_lsd_plane_slope_topic;



  cv::Mat* input;
  cv::Mat* outS;
  cv::Mat* out;
  cv::Mat pathTrace; // af-silva
  cv::Mat* lsdMask;

  cv_bridge::CvImagePtr cv_msg;
  image_transport::ImageTransport* itRecv;
  image_transport::Subscriber sub_image;

  ros::Subscriber sub_camera_pose;
  ros::Subscriber sub_lsd_mask;
  ros::Subscriber sub_lsd_vel_est;
  ros::Subscriber sub_lsd_plane_slope;

  AntSystem ants;

  CamPose lastPO;
  boost::circular_buffer<float> lastVels;
  float lastHeight;


public:

  PathDetector(); // Construtor
  ~PathDetector(); // Destrutor
  void run(int frequency); // Run Method
};
