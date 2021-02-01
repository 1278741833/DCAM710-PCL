#pragma once
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <omp.h>
#include <chrono>
#include <boost/bimap/bimap.hpp>
#include <functional>

/// Eigen
#include <Eigen/Dense>

/// OpenCV
#include <opencv2/opencv.hpp>

/// PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

enum struct SensorType
{
  STEREO,
  TOF
};

//相机参数
struct CameraParameter{
    float fx;
    float fy;
    float cx;
    float cy;
    float base_line;
    SensorType type;
    int image_width;
    int image_height;
};
