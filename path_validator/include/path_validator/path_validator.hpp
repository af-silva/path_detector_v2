/*
 * PathValidator.h
 *
 *  Created on: Apr 10, 2017
 *      Author: af-silva
 */

#ifndef PATHVALIDATOR_HPP_
#define PATHVALIDATOR_HPP_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv/highgui.h>
#include <boost/numeric/conversion/cast.hpp>
#include <std_msgs/Float32.h>



class PathValidator
{

private:

  ros::NodeHandle nh;


  image_transport::ImageTransport* it_pub;
  image_transport::ImageTransport* it_sub;

  image_transport::Subscriber sub_kf_image; // KeyFrame Mask
  image_transport::Subscriber sub_lsd_mask; // LSD Mask
  image_transport::Subscriber sub_pd_mask;  // PathDetector Mask

  image_transport::Publisher pub_pd_bin_mask;
  image_transport::Publisher pub_lsd_bin_mask;
  image_transport::Publisher pub_result;
  image_transport::Publisher pub_result_rect;

  ros::Publisher pub_per;


  cv::Mat lsd_mask;
  cv::Mat path_mask;







  /** FUNCTIONS **/
  void imageKFCallback(const sensor_msgs::ImageConstPtr& msg);
  void imageLSDMaskCallback(const sensor_msgs::ImageConstPtr& msg);
  void imagePathMaskCallback(const sensor_msgs::ImageConstPtr& msg);

public:

  PathValidator();
  virtual ~PathValidator();
  void run(int frequency);

};

#endif /* PATH_VALIDATOR_SRC_PATHVALIDATOR_H_ */
