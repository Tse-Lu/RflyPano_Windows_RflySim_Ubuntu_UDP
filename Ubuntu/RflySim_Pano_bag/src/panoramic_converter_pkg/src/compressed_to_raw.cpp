#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

class CompressedToRaw
{
public:
  CompressedToRaw()
    : nh_("~")
  {
    nh_.param<std::string>("input_topic", input_topic_, "/panorama/compressed");
    nh_.param<std::string>("output_topic", output_topic_, "/panorama");

    sub_ = nh_.subscribe(input_topic_, 1, &CompressedToRaw::imageCallback, this);
    pub_ = nh_.advertise<sensor_msgs::Image>(output_topic_, 1);

    ROS_INFO("CompressedToRaw: %s -> %s", input_topic_.c_str(), output_topic_.c_str());
  }

  void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
  {
    try
    {
      cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
      if (img.empty())
      {
        ROS_WARN("Failed to decode compressed image");
        return;
      }
      sensor_msgs::ImagePtr out = cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
      pub_.publish(out);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string input_topic_, output_topic_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compressed_to_raw");
  CompressedToRaw node;
  ros::spin();
  return 0;
}
