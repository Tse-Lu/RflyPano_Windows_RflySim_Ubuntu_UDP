#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class PanoToCatadioptric
{
public:
  PanoToCatadioptric()
    : nh_priv_("~")
    , it_(nh_global_)
  {
    nh_priv_.param<std::string>("input_topic", input_topic_, "/panoramic_image");
    nh_priv_.param<std::string>("output_topic", output_topic_, "/catadioptric/image");
    nh_priv_.param<int>("output_width", out_w_, 640);
    nh_priv_.param<int>("output_height", out_h_, 640);
    nh_priv_.param<int>("pano_width", pano_w_, 1280);
    nh_priv_.param<int>("pano_height", pano_h_, 640);

    nh_priv_.param<double>("fov_deg", fov_deg_, 120.0);
    nh_priv_.param<double>("blind_spot_deg", blind_deg_, 10.0);
    nh_priv_.param<std::string>("direction", direction_, "up");

    double fov_rad = fov_deg_ * M_PI / 180.0;
    double blind_rad = blind_deg_ * M_PI / 180.0;

    double cx = (out_w_ - 1.0) / 2.0;
    double cy = (out_h_ - 1.0) / 2.0;
    double R_outer = std::min(cx, cy);
    double R_inner = R_outer * blind_deg_ / (blind_deg_ + fov_deg_);

    map_x_ = cv::Mat(out_h_, out_w_, CV_32FC1);
    map_y_ = cv::Mat(out_h_, out_w_, CV_32FC1);

    for (int v = 0; v < out_h_; ++v)
    {
      for (int u = 0; u < out_w_; ++u)
      {
        double dx = u - cx;
        double dy = v - cy;
        double r = std::sqrt(dx * dx + dy * dy);

        if (r < R_inner || r > R_outer)
        {
          map_x_.at<float>(v, u) = -1.0f;
          map_y_.at<float>(v, u) = -1.0f;
          continue;
        }

        double t = (r - R_inner) / (R_outer - R_inner);
        double theta_z = blind_rad + t * fov_rad;

        double phi = std::atan2(dy, dx);

        double lx = std::sin(theta_z) * std::cos(phi);
        double ly = std::sin(theta_z) * std::sin(phi);
        double lz = std::cos(theta_z);

        double wx, wy, wz;
        if (direction_ == "down")
        {
          wx =  lx;
          wy = -ly;
          wz = -lz;
        }
        else
        {
          wx = lx;
          wy = ly;
          wz = lz;
        }

        double az_w = std::atan2(wy, wx);
        double el_w = std::asin(wz);

        double u_pano = pano_w_ / 2.0 - az_w / (2.0 * M_PI) * pano_w_;
        double v_pano = pano_h_ / 2.0 - el_w / M_PI * pano_h_;

        map_x_.at<float>(v, u) = static_cast<float>(u_pano);
        map_y_.at<float>(v, u) = static_cast<float>(v_pano);
      }
    }

    image_sub_ = it_.subscribe(input_topic_, 1, &PanoToCatadioptric::imageCallback, this);
    image_pub_ = it_.advertise(output_topic_, 1);

    ROS_INFO("Catadioptric: %s -> %s  [%dx%d -> %dx%d  FOV=%.0fdeg  blind=%.0fdeg  dir=%s  R_inner=%.1f R_outer=%.1f]",
             input_topic_.c_str(), output_topic_.c_str(),
             pano_w_, pano_h_, out_w_, out_h_,
             fov_deg_, blind_deg_, direction_.c_str(), R_inner, R_outer);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if (msg->width != static_cast<uint32_t>(pano_w_) ||
        msg->height != static_cast<uint32_t>(pano_h_))
    {
      ROS_WARN_THROTTLE(5, "Expected %dx%d input image, got %dx%d",
                        pano_w_, pano_h_, msg->width, msg->height);
    }

    cv::Mat input;
    try
    {
      input = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat output(out_h_, out_w_, input.type());
    cv::remap(input, output, map_x_, map_y_, cv::INTER_LINEAR,
              cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    sensor_msgs::ImagePtr out_msg =
        cv_bridge::CvImage(msg->header, "bgr8", output).toImageMsg();
    image_pub_.publish(out_msg);
  }

private:
  ros::NodeHandle nh_global_;
  ros::NodeHandle nh_priv_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  std::string input_topic_;
  std::string output_topic_;
  int out_w_, out_h_;
  int pano_w_, pano_h_;
  double fov_deg_, blind_deg_;
  std::string direction_;

  cv::Mat map_x_, map_y_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pano_to_catadioptric");
  PanoToCatadioptric node;
  ros::spin();
  return 0;
}
