#include <vector>
#include <opencv2/opencv.hpp>

// RflySim intrinsics of fisheye, calibrated by checkboard
const std::vector<double> MappingCoefficients = {167.7745, -0.0020, 5.9066e-07, -7.6388e-09};
const cv::Point2d DistortionCenter(320.0, 320.0);
const cv::Size ImageSize(640, 640);