#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

struct FisheyeCamera
{
  double azimuth;
  double elevation;
  std::string name;
};

class PanoTo4Fisheye
{
public:
  PanoTo4Fisheye()
    : nh_priv_("~")
    , it_(nh_global_)
  {
    nh_priv_.param<std::string>("input_topic", input_topic_, "/panoramic_image");
    nh_priv_.param<std::string>("output_topic_prefix", output_prefix_, "/fisheye");
    nh_priv_.param<int>("output_width", out_w_, 640);
    nh_priv_.param<int>("output_height", out_h_, 640);
    nh_priv_.param<double>("fov_deg", fov_deg_, 180.0);
    nh_priv_.param<int>("pano_width", pano_w_, 1280);
    nh_priv_.param<int>("pano_height", pano_h_, 640);

    double fov_rad = fov_deg_ * M_PI / 180.0;
    double alpha_max = fov_rad / 2.0;

    cx_ = out_w_ / 2.0;
    cy_ = out_h_ / 2.0;
    R_ = std::min(cx_, cy_);
    f_ = R_ / alpha_max;

    // Navigation convention (cw from Forward): 0°=F, 90°=R, 180°=B, 270°=L
    // Convert to standard math ccw: az_ccw = -nav_deg (mod 360)
    std::vector<FisheyeCamera> cameras = {
      {  45.0 * M_PI / 180.0, 0.0, "front_left"  },   // nav 315deg
      { 135.0 * M_PI / 180.0, 0.0, "back_left"   },   // nav 225deg
      {-135.0 * M_PI / 180.0, 0.0, "back_right"  },   // nav 135deg
      { -45.0 * M_PI / 180.0, 0.0, "front_right" }    // nav  45deg
    };

    for (size_t i = 0; i < cameras.size(); ++i)
    {
      const auto& cam = cameras[i];
      cv::Mat map_x(out_h_, out_w_, CV_32FC1);
      cv::Mat map_y(out_h_, out_w_, CV_32FC1);

      double az = cam.azimuth;
      double el = cam.elevation;

      double bx = std::cos(el) * std::cos(az);
      double by = std::cos(el) * std::sin(az);
      double bz = std::sin(el);

      double Yx =  std::sin(az);
      double Yy = -std::cos(az);
      double Yz =  0.0;

      double Zx =  std::sin(el) * std::cos(az);
      double Zy =  std::sin(el) * std::sin(az);
      double Zz = -std::cos(el);

      for (int v = 0; v < out_h_; ++v)
      {
        for (int u = 0; u < out_w_; ++u)
        {
          double dx = u - cx_;
          double dy = v - cy_;
          double r = std::sqrt(dx * dx + dy * dy);

          if (r > R_)
          {
            map_x.at<float>(v, u) = -1.0f;
            map_y.at<float>(v, u) = -1.0f;
            continue;
          }

          double alpha = r / f_;
          double phi = std::atan2(dy, dx);

          double lx = std::cos(alpha);
          double ly = std::sin(alpha) * std::cos(phi);
          double lz = std::sin(alpha) * std::sin(phi);

          double wx = bx * lx + Yx * ly + Zx * lz;
          double wy = by * lx + Yy * ly + Zy * lz;
          double wz = bz * lx + Yz * ly + Zz * lz;

          double az_w = std::atan2(wy, wx);
          double el_w = std::asin(wz);

          double u_pano = pano_w_ / 2.0 - az_w / (2.0 * M_PI) * pano_w_;
          double v_pano = pano_h_ / 2.0 - el_w / M_PI * pano_h_;

          map_x.at<float>(v, u) = static_cast<float>(u_pano);
          map_y.at<float>(v, u) = static_cast<float>(v_pano);
        }
      }

      maps_x_.push_back(map_x);
      maps_y_.push_back(map_y);

      std::string topic = output_prefix_ + "/" + cam.name + "/image";
      pubs_.push_back(it_.advertise(topic, 1));

      ROS_INFO("Fisheye camera [%s]: azimuth=%.1f deg, elevation=%.1f deg  -> %s",
               cam.name.c_str(), cam.azimuth * 180.0 / M_PI,
               cam.elevation * 180.0 / M_PI, topic.c_str());
    }

    image_sub_ = it_.subscribe(input_topic_, 1, &PanoTo4Fisheye::imageCallback, this);

    ROS_INFO("PanoTo4Fisheye: %s -> 4 x [%dx%d  FOV=%.0f deg]",
             input_topic_.c_str(), out_w_, out_h_, fov_deg_);
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

    for (size_t i = 0; i < maps_x_.size(); ++i)
    {
      cv::remap(input, output, maps_x_[i], maps_y_[i], cv::INTER_LINEAR,
                cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

      sensor_msgs::ImagePtr out_msg =
          cv_bridge::CvImage(msg->header, "bgr8", output).toImageMsg();
      pubs_[i].publish(out_msg);
    }
  }

private:
  ros::NodeHandle nh_global_;
  ros::NodeHandle nh_priv_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  std::vector<image_transport::Publisher> pubs_;

  std::string input_topic_;
  std::string output_prefix_;
  int out_w_, out_h_;
  int pano_w_, pano_h_;
  double fov_deg_;
  double cx_, cy_, R_, f_;

  std::vector<cv::Mat> maps_x_;
  std::vector<cv::Mat> maps_y_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pano_to_4fisheye");
  PanoTo4Fisheye node;
  ros::spin();
  return 0;
}
