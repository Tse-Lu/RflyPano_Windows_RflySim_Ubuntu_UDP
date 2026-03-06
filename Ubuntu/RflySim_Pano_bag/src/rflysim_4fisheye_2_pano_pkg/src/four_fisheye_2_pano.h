#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include "cam_params.h"

class RflyPanoramaStitcher {
private:
    cv::Size pano_size_;
    std::vector<cv::Mat> map_x_, map_y_; 

    double calculate_r(double theta) const {
        double r = 0;
        for (size_t i = 0; i < MappingCoefficients.size(); ++i) {
            r += MappingCoefficients[i] * std::pow(theta, static_cast<int>(2 * i + 1));
        }
        return r;
    }

    void precomputeMaps() {
        // 定义四个相机的 Y 轴旋转角度 (45, 135, 225, 315 度)
        double angles[] = {45.0, 135.0, 225.0, 315.0};
        
        for (int i = 0; i < 4; ++i) {
            map_x_[i].create(pano_size_, CV_32FC1);
            map_y_[i].create(pano_size_, CV_32FC1);
            map_x_[i].setTo(-1.0f);
            map_y_[i].setTo(-1.0f);

            // 计算该相机的旋转矩阵及其逆矩阵
            double rad = angles[i] * M_PI / 180.0;
            Eigen::Matrix3d R_cam;
            R_cam = Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitY());
            Eigen::Matrix3d R_inv = R_cam.transpose();

            for (int v = 0; v < pano_size_.height; ++v) {
                for (int u = 0; u < pano_size_.width; ++u) {
                    // 1. 将全景图坐标映射到球面坐标 (经纬度)
                    double lon = (static_cast<double>(u) / pano_size_.width) * 2.0 * M_PI - M_PI;
                    double lat = M_PI / 2.0 - (static_cast<double>(v) / pano_size_.height) * M_PI;

                    // 2. 转换为 3D 方向向量
                    Eigen::Vector3d dir_pano(
                        std::cos(lat) * std::sin(lon),
                        -std::sin(lat),
                        std::cos(lat) * std::cos(lon)
                    );

                    // 3. 旋转到相机局部坐标系
                    Eigen::Vector3d dir_cam = R_inv * dir_pano;

                    // 4. 利用鱼眼多项式模型投影到像素平面
                    if (dir_cam.z() > 0) { // 只处理相机前方 180 度
                        double theta = std::acos(dir_cam.z() / dir_cam.norm());
                        double r = calculate_r(theta);
                        
                        double norm = std::sqrt(dir_cam.x() * dir_cam.x() + dir_cam.y() * dir_cam.y()) + 1e-8;
                        double cam_u = r * dir_cam.x() / norm + DistortionCenter.x;
                        double cam_v = r * dir_cam.y() / norm + DistortionCenter.y;

                        if (cam_u >= 0 && cam_u < ImageSize.width && cam_v >= 0 && cam_v < ImageSize.height) {
                            map_x_[i].at<float>(v, u) = static_cast<float>(cam_u);
                            map_y_[i].at<float>(v, u) = static_cast<float>(cam_v);
                        }
                    }
                }
            }
        }
    }

public:
    RflyPanoramaStitcher(int width = 1280, int height = 640) 
        : pano_size_(width, height) {
        map_x_.resize(4);
        map_y_.resize(4);
        precomputeMaps();
    }

    ros::Publisher pub_pano_img;

    // vector << right_front << right_back << left_back << left_front
    cv::Mat stitch(const std::vector<cv::Mat>& fisheye_imgs) {
        if (fisheye_imgs.size() < 4) return cv::Mat();

        cv::Mat pano_acc = cv::Mat::zeros(pano_size_, CV_32FC3);
        cv::Mat weight_acc = cv::Mat::zeros(pano_size_, CV_32FC1);

        for (int i = 0; i < 4; ++i) {
            cv::Mat remapped, mask_8u;
            // 1. 重映射
            cv::remap(fisheye_imgs[i], remapped, map_x_[i], map_y_[i], cv::INTER_LINEAR, cv::BORDER_CONSTANT);
            
            // 2. 正确生成 Mask: 必须保持 CV_8U 类型给 accumulate 使用
            cv::cvtColor(remapped, mask_8u, cv::COLOR_BGR2GRAY);
            cv::threshold(mask_8u, mask_8u, 1, 255, cv::THRESH_BINARY); // 正常的 0/255 掩码

            // 3. 图像转浮点用于累加
            cv::Mat remapped_f;
            remapped.convertTo(remapped_f, CV_32FC3);

            // 4. 累加（此时 mask_8u 是符合 OpenCV 要求的 CV_8U）
            cv::accumulate(remapped_f, pano_acc, mask_8u);

            // 5. 权重累加需要浮点型，这里单独转一下用于计算权重图
            cv::Mat mask_f;
            mask_8u.convertTo(mask_f, CV_32FC1, 1.0/255.0); // 转回 0.0-1.0
            cv::add(weight_acc, mask_f, weight_acc);
        }

        // --- 后续归一化逻辑 ---
        cv::add(weight_acc, 1.0e-5f, weight_acc); // 避免除零
        
        // 效率优化：直接用分母除以全通道，不需要手动 split/merge
        cv::Mat result_f;
        // 这里利用 OpenCV 的自动广播或通过通道处理
        std::vector<cv::Mat> channels;
        cv::split(pano_acc, channels);
        for (int c = 0; c < 3; ++c) {
            cv::divide(channels[c], weight_acc, channels[c]);
        }
        cv::merge(channels, result_f);

        cv::Mat result;
        result_f.convertTo(result, CV_8UC3);
        return result;
    }
};

class FisheyeData {
public:
    std::vector<cv::Mat> fisheye_img_buf;

    void callback_img(const sensor_msgs::ImageConstPtr& img_msg);

    void callback_img_and_stitch(
        const sensor_msgs::ImageConstPtr& img, 
        FisheyeData* img_buf_left_front,
        FisheyeData* img_buf_left_back,
        FisheyeData* img_buf_right_back,
        RflyPanoramaStitcher* stitcher);
};