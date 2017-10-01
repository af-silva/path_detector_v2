/*
 * PathValidator.cpp
 *
 *  Created on: Apr 10, 2017
 *      Author: andre
 */

#include "path_validator/path_validator.hpp"


/**
 *
 */
PathValidator::PathValidator()
{
  //ros::NodeHandle nh("~");

  it_pub = new image_transport::ImageTransport(nh);
  it_sub = new image_transport::ImageTransport(nh);

  pub_lsd_bin_mask = it_pub->advertise("/path_validator/lsd_bin_mask", 1);
  pub_pd_bin_mask = it_pub->advertise("/path_validator/pd_bin_mask", 1);
  pub_result = it_pub->advertise("/path_validator/result", 1);
  pub_result_rect = it_pub->advertise("/path_validator/result_rect", 1);

  sub_kf_image = it_sub->subscribe("/lsd_slam/image", 1, &PathValidator::imageKFCallback, this);
  sub_lsd_mask = it_sub->subscribe("/lsd_slam/mask_nf", 1, &PathValidator::imageLSDMaskCallback, this);
  sub_pd_mask = it_sub->subscribe("/path_detector/trace", 1, &PathValidator::imagePathMaskCallback, this);

  pub_per = nh.advertise<std_msgs::Float32>("/path_validator/per_points", 2);

}

/**
 *
 */
PathValidator::~PathValidator()
{
  // TODO Auto-generated destructor stub

}

/**
 * Run Method
 */
void PathValidator::run(int frequency)
{
  ros::Rate loop_rate(frequency);

  while (ros::ok())
  {

    ros::spinOnce();

    loop_rate.sleep();
  }

}

/**
 *
 */
void PathValidator::imageKFCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (!lsd_mask.empty() && !path_mask.empty())
  {

    //ROS_INFO("new image ok");

    cv::Mat lsd_mask_bin(lsd_mask.size(), lsd_mask.type());
    cv::Mat path_mask_bin(path_mask.size(), path_mask.type());

    cv::resize(path_mask, path_mask, lsd_mask.size(), 0, 0, CV_INTER_AREA);
    cv::medianBlur(lsd_mask, lsd_mask, 5); // TODO test
    cv::medianBlur(path_mask, path_mask, 5); // TODO test
    cv::threshold(lsd_mask, lsd_mask_bin, 25, 255, cv::THRESH_BINARY);
    cv::threshold(path_mask, path_mask_bin, 25, 255, cv::THRESH_BINARY);

    cv::Mat res;
    cv::Mat area;

//    cv::bitwise_or(lsd_mask_bin, path_mask_bin, area);
    cv::bitwise_and(lsd_mask_bin, path_mask_bin, res);


//    float per = 0.0;
//    if(res.total() > 0 and area.total() > 0.0)
//      per = ( (float)res.total() / (float)area.total() );

    cv::Mat drawing = cv::Mat::zeros(res.size(), CV_8UC3);
    if (!res.empty())
    {

      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Point> pts;


      // edited to change from CV_CHAIN_APPROX_SIMPLE or CV_CHAIN_APPROX_NONE
      cv::findContours(res, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

      if (contours.size() > 0)
      {
        // Finds the contour with the largest area
        int _area = 0;
        int idx;
        for (int i = 0; i < contours.size(); i++)
        {
          if (contours[i].size() > _area)
          {

            idx = i;

            // test
            cv::Rect rec = cv::boundingRect(contours[i]);
            cv::Point pt1, pt2;
            pt1.x = rec.x;
            pt1.y = rec.y;
            pt2.x = rec.x + rec.width;
            pt2.y = rec.y + rec.height;
            cv::drawContours(drawing, contours, i, CV_RGB(255, 255, 255), 1, 8, cv::vector<cv::Vec4i>(), 0, cv::Point());
            cv::rectangle(drawing, pt1, pt2, CV_RGB(255, 0, 0), 1);
          }
        }

//        cv::Rect rec = cv::boundingRect(contours[idx]);
//        cv::Point pt1, pt2;
//
//        pt1.x = rec.x;
//        pt1.y = rec.y;
//        pt2.x = rec.x + rec.width;
//        pt2.y = rec.y + rec.height;
//        cv::drawContours(drawing, contours, idx, CV_RGB(255, 255, 255), 1, 8, cv::vector<cv::Vec4i>(), 0, cv::Point());
//        cv::rectangle(drawing, pt1, pt2, CV_RGB(255, 0, 0), 1);

      }

    }


    sensor_msgs::ImagePtr lsd_mask_bin_out = cv_bridge::CvImage(std_msgs::Header(), "mono8", lsd_mask_bin).toImageMsg();
    pub_lsd_bin_mask.publish(lsd_mask_bin_out);

    sensor_msgs::ImagePtr path_mask_bin_out =
        cv_bridge::CvImage(std_msgs::Header(), "mono8", path_mask_bin).toImageMsg();
    pub_pd_bin_mask.publish(path_mask_bin_out);

    // draw the interceoption between the lsd_slam cloud obstacles with the path detector trace
    sensor_msgs::ImagePtr res_out = cv_bridge::CvImage(std_msgs::Header(), "mono8", res).toImageMsg();
    pub_result.publish(res_out);

    // draw the interceoption between the lsd_slam cloud obstacles with the path detector trace with rectangles
    sensor_msgs::ImagePtr res_out_rect = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing).toImageMsg();
    pub_result_rect.publish(res_out_rect);

//    std_msgs::Float32 msg;
//    msg.data = per;
//    pub_per.publish(msg);

  }
}

/**
 *
 */
void PathValidator::imageLSDMaskCallback(const sensor_msgs::ImageConstPtr& msg)
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
  cv_ptr->image.copyTo(lsd_mask);
  cv_ptr.reset();
}

/**
 *
 */
void PathValidator::imagePathMaskCallback(const sensor_msgs::ImageConstPtr& msg)
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
  cv_ptr->image.copyTo(path_mask);
  cv_ptr.reset();
}

/**
 *
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "PathValidator");

  PathValidator node;

  node.run(20);

  return 0;

}
;
