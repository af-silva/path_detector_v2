#include "path_detector/PathDetector.h"

// Construtor
PathDetector::PathDetector() :
    ants(20) // 25
{
  it = new image_transport::ImageTransport(node);
  itRecv = new image_transport::ImageTransport(node);

  pubImg = it->advertise("/path_detector/output", 1);

  pubPathTrace = it->advertise("/path_detector/trace", 1);

  nh = boost::make_shared<ros::NodeHandle>("~");

  // sub topics
  nh->param<std::string>("image_topic", sub_image_topic, "/lsd_slam/image");
  //nh->param<std::string>("image_topic", sub_image_topic, "/camera_crop/image_raw");

  nh->param<std::string>("pose_topic", sub_camera_pose_topic, "/lsd_slam/pose");
  nh->param<std::string>("vel_topic", sub_lsd_vel_est_topic, "/lsd_slam/est_vel");
  nh->param<std::string>("plane_slope", sub_lsd_plane_slope_topic, "/lsd_slam/plane_slope");

  nh->param<std::string>("lsd_mask", sub_lsd_mask_topic, "/lsd_slam/mask_nf");
  //nh->param<std::string>("lsd_mask", sub_lsd_mask_topic, "/lsd_slam/mask_nf2");
  //nh->param<std::string>("lsd_mask", sub_lsd_mask_topic, "/lsd_slam/mask");

  nh->param<bool>("lsd_mask_on", useLSDMask, true);

  lastPO.lastTime = ros::Time(0.0000000);
  lastPO.ow = 0;
  lastPO.ox = 0;
  lastPO.oy = 0;
  lastPO.oz = 0;
  lastPO.px = 0;
  lastPO.py = 0;
  lastPO.pz = 0;

  lastVels.set_capacity(5);

  pubNeuralField = it->advertise("/path_detector_nf", 1);
  pubTrailProbMap = it->advertise("/path_detector_tpm", 1);
  pubTDSalMap = it->advertise("/path_detector_tdsm", 1);
  pubTDConspCMap = it->advertise("/path_detector_tdccm", 1);

  cvImage.encoding = "bgr8";

  frame_count = 0;
  lastHeight = 40; // TODO

  sub_image = itRecv->subscribe(sub_image_topic, 1, &PathDetector::imageCallback, this);
  sub_camera_pose = nh->subscribe(sub_camera_pose_topic, 1, &PathDetector::poseCallback, this);
  sub_lsd_mask = nh->subscribe(sub_lsd_mask_topic, 1, &PathDetector::lsdSlamCallback, this);
  sub_lsd_vel_est = nh->subscribe(sub_lsd_vel_est_topic, 1, &PathDetector::lsdEstVelCallback, this);
  sub_lsd_plane_slope = nh->subscribe(sub_lsd_plane_slope_topic, 1, &PathDetector::lsdPlaneSlopeCallback, this);
  //sub = itRecv->subscribe("/camera_crop/image_raw", 1, &PathDetector::imageCallback, this);

  input = new cv::Mat(240, 320, CV_8UC3, cv::Scalar(0));
  lsdMask = new cv::Mat(60, 80, CV_8UC1, cv::Scalar(0));
  outS = new cv::Mat(60, 80, CV_8UC3, cv::Scalar(0));
  out = new cv::Mat(240, 320, CV_8UC3, cv::Scalar(0));

  std::cout << "Using LSD Mask:  " << useLSDMask << std::endl;

}

// Destrutor
PathDetector::~PathDetector()
{
  delete input;
  delete out;
  delete outS;
  delete lsdMask;
}

// Run Method
void PathDetector::run(int frequency)
{
  ros::Rate loop_rate(frequency);
  frame_count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

/**
 *
 */
void PathDetector::lsdSlamCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::resize(cv_ptr->image, *lsdMask, lsdMask->size());
  cv_ptr.reset();

  //cv::imshow("1 mask", *lsdMask);
}

// Callback Methods
void PathDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {

    cv_msg = cv_bridge::toCvCopy(msg, "bgr8");
    pathTrace = cv::Mat::zeros(cv::Size(320, 240), CV_8UC3);  // at each callback resets the image
    cv::resize(cv_msg->image, *input, input->size());
    ants.useLSDSlamMask(useLSDMask);

    frame_count++;
    if ((ants.isLearning()) && (frame_count > 50))
    {
      ants.setLearningMode(false);
      ants.changeAntMaxHeight(40);
    }

    double sum = std::accumulate(lastVels.begin(), lastVels.end(), 0.0);
    double vel = (sum / (double) lastVels.size());

    if(useLSDMask)
    {
      ants.changeAntMaxHeight(lastHeight);
    }



    // run swarm path detector
    if (!lsdMask->empty())  // this shloud not be this way, if it has a mask runs it should wait for the mask
    {
      //lsdMask.convertTo(lsdMask, CV_8UC1);
      cv::medianBlur(*lsdMask, *lsdMask, 5);  // TODO
      ants.run(*input, *out, pathTrace, *lsdMask, vel);

      cv::waitKey(5);

      // principal topics
      sensor_msgs::ImagePtr img_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *out).toImageMsg();
      pubImg.publish(img_out);
      sensor_msgs::ImagePtr path_tracer = cv_bridge::CvImage(std_msgs::Header(), "mono8", pathTrace).toImageMsg();
      pubPathTrace.publish(path_tracer);

      // other topics
      sensor_msgs::ImagePtr img_nf =
          cv_bridge::CvImage(std_msgs::Header(), "mono8", ants.getNeuralField()).toImageMsg();
      pubNeuralField.publish(img_nf);

//    sensor_msgs::ImagePtr img_tpm = cv_bridge::CvImage(std_msgs::Header(), "mono8", ants.getTrailProbMap()).toImageMsg();
//    pubTrailProbMap.publish(img_tpm);
//    sensor_msgs::ImagePtr img_tdsm = cv_bridge::CvImage(std_msgs::Header(), "mono8", ants.getTDSalMap()).toImageMsg();
//    pubTDSalMap.publish(img_tdsm);
//    sensor_msgs::ImagePtr img_tdccm =  cv_bridge::CvImage(std_msgs::Header(), "mono8", ants.getTDConspCMap()).toImageMsg();
//    pubTDConspCMap.publish(img_tdccm);

    }

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void PathDetector::poseCallback(const geometry_msgs::PoseStampedPtr& msg)
{
  double r, p, y;
  lastPO.px = msg->pose.position.x;
  lastPO.py = msg->pose.position.y;
  lastPO.pz = msg->pose.position.z;
  lastPO.ox = msg->pose.orientation.x;
  lastPO.oy = msg->pose.orientation.y;
  lastPO.oz = msg->pose.orientation.z;
  lastPO.ow = msg->pose.orientation.w;
  lastPO.lastTime = msg->header.stamp;
  tf::quaternionMsgToTF(msg->pose.orientation, lastPO.rpy);
  ros::Time currtime = msg->header.stamp;
  tf::Matrix3x3(lastPO.rpy).getRPY(r, p, y);
}

void PathDetector::lsdEstVelCallback(const std_msgs::Float32Ptr& msg)
{
  float vel = msg->data;
  lastVels.push_back(vel);
//  std::cout << vel << std::endl;
}

void PathDetector::lsdPlaneSlopeCallback(const std_msgs::Float32Ptr& msg)
{

  if (useLSDMask)
  {
    float slope = msg->data;
    lastHeight = 30 + slope * 20;
//    std::cout << "height" <<  last_height << std::endl;
  }
}

/**
 *
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "path_detector");

  PathDetector node;

  node.run(20);

  return 0;

}
;
