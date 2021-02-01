#ifndef VZENSE__H
#define VZENSE__H
#pragma once
#include "header.h"
#include "main.h"
#include "Vzense_api2.h"
#include <thread>
using namespace std;
class COMMANDER;
class Vzense{
public:
    Vzense();
    ~Vzense();
    void Threadvzense();
    void Vzensemain();
    bool initialize();
    void update();
    void start();//dummy
    void stop();
    void sample();
    CameraParameter
    getCameraParameter();
    cv::Mat getRGBImage();
    cv::Mat getDepthImage();
    cv::Mat getIRImage();
    cv::Mat getColorizedDepthImage();
    cv::Mat getWdrImage();
    
private:
    PsDeviceHandle deviceHandle = 0;
    uint32_t sessionIndex = 0;
    cv::Mat color_img, depth_img, ir_img, wdr_img, colorized_depth_img;
    CameraParameter camera_param;

    int _image_width;
    int _image_height;
    int _capture_fps;
	int _theta_pitch;
    SensorType type;

};
#endif /* VZENSE__H */
