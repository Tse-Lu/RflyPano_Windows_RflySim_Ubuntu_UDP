#include "four_fisheye_2_pano.h"

int ImageWidth = 1280; // default
int ImageHeight = 640;

void FisheyeData::callback_img(const sensor_msgs::ImageConstPtr& img_msg)
{
    // for fisheye: left_front, left_back, right_back
    fisheye_img_buf.clear();

    cv::Mat img = rosImageToBgr(img_msg);
    fisheye_img_buf.push_back(img); // save the latest frame
}

void FisheyeData::callback_img_and_stitch(
    const sensor_msgs::ImageConstPtr& img_msg, 
    FisheyeData* img_buf_left_front,
    FisheyeData* img_buf_left_back,
    FisheyeData* img_buf_right_back,
    RflyPanoramaStitcher* stitcher)
{
    // for fisheye: right_front and stitch four fisheye images
    std::vector<cv::Mat> four_imgs(4); // right_front << right_back << left_back << left_front
    four_imgs[0] = rosImageToBgr(img_msg);
    if (img_buf_left_back->fisheye_img_buf.size() == 0 || img_buf_left_front->fisheye_img_buf.size() == 0 || img_buf_right_back->fisheye_img_buf.size()==0)
    {
        ROS_INFO("Initializing in stitching panoramic image when receieved fisheye imgs are empty!");
        return;
    }
    four_imgs[1] = img_buf_right_back->fisheye_img_buf.back();
    four_imgs[2] = img_buf_left_back->fisheye_img_buf.back();
    four_imgs[3] = img_buf_left_front->fisheye_img_buf.back();
    
    cv::Mat pano_img = stitcher->stitch(four_imgs);
    if (pano_img.empty())
    {
        ROS_WARN("Stitched panorama image is empty, skip publishing.");
        return;
    }

    stitcher->pub_pano_img.publish(bgrToRosImage(img_msg->header, pano_img));

    sensor_msgs::CompressedImage compressed_msg;
    compressed_msg.header = img_msg->header;
    compressed_msg.format = "jpeg";
    cv::imencode(".jpg", pano_img, compressed_msg.data);
    stitcher->pub_pano_compressed_img.publish(compressed_msg);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "pano_node");
    ros::NodeHandle nh;
    ros::Publisher pub_pano_img = nh.advertise<sensor_msgs::Image>("panorama", 1);
    ros::Publisher pub_pano_compressed_img = nh.advertise<sensor_msgs::CompressedImage>("panorama/compressed", 1);

    nh.param<int>("ImageWidth", ImageWidth, 1280);
    nh.param<int>("ImageHeight", ImageHeight, 640);

    RflyPanoramaStitcher stitcher(ImageWidth, ImageHeight);
    stitcher.pub_pano_img = pub_pano_img;
    stitcher.pub_pano_compressed_img = pub_pano_compressed_img;

    FisheyeData fisheye_right_front;
    FisheyeData fisheye_right_back;
    FisheyeData fisheye_left_front;
    FisheyeData fisheye_left_back;

    ros::Subscriber fisheye_right_back_sub = nh.subscribe<sensor_msgs::Image>(
        "/camera/RightBack", 
        1, 
        &FisheyeData::callback_img, 
        &fisheye_right_back 
    );

    ros::Subscriber fisheye_left_back_sub = nh.subscribe<sensor_msgs::Image>(
        "/camera/LeftBack", 
        1, 
        &FisheyeData::callback_img, 
        &fisheye_left_back  
    );

    ros::Subscriber fisheye_left_front_sub = nh.subscribe<sensor_msgs::Image>(
        "/camera/LeftFront", 
        1, 
        &FisheyeData::callback_img, 
        &fisheye_left_front
    );
    ros::Subscriber fisheye_right_front_sub = nh.subscribe<sensor_msgs::Image>(
        "/camera/RightFront", 1,
        boost::bind(&FisheyeData::callback_img_and_stitch, 
                    &fisheye_right_front, 
                    _1,                   
                    &fisheye_left_front, 
                    &fisheye_left_back,   
                    &fisheye_right_back,  
                    &stitcher             
        )
    );
    ros::spin();
    return 0;
}