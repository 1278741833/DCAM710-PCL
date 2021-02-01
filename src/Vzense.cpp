#include "main.h"

//depth -> point
pcl::PointCloud<pcl::PointXYZ>::Ptr Depth2Point(cv::Mat src, CameraParameter cam_p) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int h = 0; h < src.rows; h++) {
        for (int w = 0; w < src.cols; w++) {

            unsigned short z_value_short;
            float z_value_float;
            z_value_short = src.at<short>(h, w);
//          z_value_float = src.at<float>(h, w);

            if (z_value_short > 0 || z_value_float >0.0) {
                Eigen::Vector3f v;
                v = Eigen::Vector3f::Zero();

                v.z() = (float)(z_value_short)/1000;

                if(v.z() == 0) continue;
                v.x() = v.z() * (w - cam_p.cx) * (1.0 / cam_p.fx);
                v.y() = v.z() * (h - cam_p.cy) * (1.0 / cam_p.fy);

                pcl::PointXYZ point_tmp;
                point_tmp.x = v.x();
                point_tmp.y = v.y();
                point_tmp.z = v.z();
                cloud->points.push_back(point_tmp);
            }
        }
    }

    return cloud;
}
void HotPlugStateCallback(const char *uri, int params);
using namespace std;
void Vzense::Threadvzense()
{
    cout << "[START THREAD] -->Pico Vzense " << endl;
    thread vzensethread(&Vzense::Vzensemain, this);
    vzensethread.detach();
}
void Vzense::Vzensemain()
{
    if (initialize() == false)
    {
        LOG_WARN("Constructor") << "initialize failed";
        exit(-1);
    }
    PsReturnStatus status;
    PsDataMode dataMode = PsDepthAndIR_30;
    status = Ps2_GetDataMode(deviceHandle, sessionIndex, &dataMode);
	if (status != PsReturnStatus::PsRetOK)
		cout << "Ps2_GetDataMode failed!" << endl;
	else
		LOG_WARN("dataMode") << "Get Ps2_GetDataMode : "<< dataMode;
    while (1)
    {
        util::sleep(20);
        sample();
      
    }
}
void Vzense::sample()
{
    update();
    cv::Mat color, ir_left, depth, wdr, depth_color;
    color = getRGBImage().clone();
    depth = getDepthImage().clone();
    ir_left = getIRImage().clone();
    wdr = getWdrImage().clone();
    if (wdr.rows == 0)
        return;
           
    depth_color = getColorizedDepthImage();
    pcl::PointCloud<pcl::PointXYZ>::Ptr point(new pcl::PointCloud<pcl::PointXYZ>);
    
    point = Depth2Point(wdr, camera_param);
    LOG_WARN("point") << point->size();
    pcl::io::savePCDFileBinary("test.pcd", *point);
}
Vzense::Vzense()
{
}
Vzense::~Vzense()
{
   stop();
}
bool Vzense::initialize()
{

    _image_width = sys->param.image_width;
    _image_height = sys->param.image_height;
    _capture_fps = sys->param.capture_fps;
    _theta_pitch = sys->param.theta_pitch;
    PsReturnStatus status;
    uint32_t deviceIndex = 0;
    uint32_t deviceCount = 0;
    uint32_t slope = 1450;
    uint32_t wdrSlope = 4400;
    uint32_t wdrRange1Slope = 1450;
    uint32_t wdrRange2Slope = 4400;
    uint32_t wdrRange3Slope = 6000;

    // initialize check
    status = Ps2_Initialize();
    if (status != PsReturnStatus::PsRetOK)
    {
        cout << "PsInitialize failed!" << endl;
        return false;
    }
GET:
    status = Ps2_GetDeviceCount(&deviceCount);
    if (status != PsReturnStatus::PsRetOK)
    {
        cout << "PsGetDeviceCount failed!" << endl;
        return false;
    }
    cout << "Get device count: " << deviceCount << endl;
    if (0 == deviceCount)
    {
        goto GET;
    }

    // camera open
    Ps2_SetHotPlugStatusCallback(HotPlugStateCallback);

    PsDeviceInfo *pDeviceListInfo = new PsDeviceInfo[deviceCount];
    status = Ps2_GetDeviceListInfo(pDeviceListInfo, deviceCount);
    deviceHandle = 0;
    status = Ps2_OpenDevice(pDeviceListInfo->uri, &deviceHandle);
    if (status != PsReturnStatus::PsRetOK)
    {
        cout << "OpenDevice failed!" << endl;
        system("pause");
        return false;
    }
    uint32_t sessionIndex = 0;

    status = Ps2_StartStream(deviceHandle, sessionIndex);
    if (status != PsReturnStatus::PsRetOK)
    {
        cout << "StartStream failed!" << endl;
        system("pause");
        return false;
    }

    camera_param = getCameraParameter();

    //Set PixelFormat as PsPixelFormatBGR888 for opencv display
    Ps2_SetColorPixelFormat(deviceHandle, sessionIndex, PsPixelFormatBGR888);

    /// RGB-D Mode -> depth img size = 640*360
    /// Depth Only Mode -> depth img size = 640*480

    /// Mode
    PsDataMode dataMode = PsWDR_Depth;
    PsWDROutputMode wdrMode = {PsWDRTotalRange_Two, PsNearRange, 1, PsFarRange, 1, PsUnknown, 1};
    if (dataMode == PsWDR_Depth)
    {
        Ps2_SetWDROutputMode(deviceHandle, sessionIndex, &wdrMode);

        status = Ps2_SetDataMode(deviceHandle, sessionIndex, (PsDataMode)dataMode);
        if (status != PsRetOK)
        {
            cout << "Ps2_SetDataMode  status" << status << endl;
        }
    }
    static bool bWDRStyle = false;
    status = Ps2_SetWDRStyle(deviceHandle, sessionIndex, bWDRStyle ? PsWDR_ALTERNATION : PsWDR_FUSION);
    if (PsRetOK == status)
    {
        cout << "WDR image output " << (bWDRStyle ? "alternatively in multi range." : "Fusion.") << endl;
        
    }
    /// distortion
    //bool f_bDistortionCorrection = false;
    bool f_bDistortionCorrection = true;
    // Ps2_SetDepthDistortionCorrectionEnabled(deviceHandle, sessionIndex,f_bDistortionCorrection);//设置启用或禁用深度畸变矫正
    // Ps2_SetIrDistortionCorrectionEnabled(deviceHandle, sessionIndex, f_bDistortionCorrection);  //设置启用或禁用 IR 畸变矫正
    // Ps2_SetRGBDistortionCorrectionEnabled(deviceHandle, sessionIndex, f_bDistortionCorrection); //设置启用或禁用 RGB 畸变矫正

    /// Smooth Filter  设置启用或禁用深度图像空间滤波操作
    bool f_bFilter = false;
    //bool f_bFilter = true;
    Ps2_SetSpatialFilterEnabled(deviceHandle, sessionIndex, f_bFilter);

    //Enable the Depth and RGB synchronize feature
    Ps2_SetSynchronizeEnabled(deviceHandle, sessionIndex, true);
    /// RGB resolution
    // PsFrameMode frameMode;
    // frameMode.resolutionWidth = _image_width;
    // frameMode.resolutionHeight = _image_height;
    // PsSetFrameMode(deviceIndex, PsRGBFrame, &frameMode);

    /// Mapped FLAG
    //    bool f_bMappedDepth = true;
    //    status = PsSetMapperEnabledRGBToDepth(deviceIndex, f_bMappedDepth);
    // bool f_bMappedRGB = true;
    //// status = Ps2_SetMapperEnabledDepthToRGB(deviceHandle,sessionIndex, f_bMappedRGB);

    // if (status != PsRetOK)
    // {
    //     cout << "PsSetMapperEnabledRGBToDepth failed!" << endl;
    //     return false;
    // }

    return true;
}

void Vzense::update()
{

    PsReturnStatus status;
    PsFrameReady frameReady = {0};
    status = Ps2_ReadNextFrame(deviceHandle, sessionIndex, &frameReady);

    // 因为启动之后不稳定，所以基于if语句的空判定

    /// RGB
    PsFrame rgbFrame = {0};
    Ps2_GetFrame(deviceHandle, sessionIndex, PsRGBFrame, &rgbFrame);
    if (rgbFrame.pFrameData != NULL)
    {
        cv::Mat _color_img = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3, rgbFrame.pFrameData);
        color_img = _color_img.clone();
    }
   
    Ps2_GetFrame(deviceHandle, sessionIndex, PsRGBFrame, &rgbFrame);

    //PsImuWithParams temp_param;
    //PsGetImuWithParams(deviceIndex, &temp_param);
    //cout << "temp:" << temp_param.temp << endl;

    /// RGB->Depth Mapped
    PsFrame mappedRGBFrame = {0};
    Ps2_GetFrame(deviceHandle, sessionIndex, PsMappedRGBFrame, &mappedRGBFrame);
    if (mappedRGBFrame.pFrameData != NULL)
    {
        cv::Mat _color_img = cv::Mat(mappedRGBFrame.height, mappedRGBFrame.width, CV_8UC3, mappedRGBFrame.pFrameData);
        color_img = _color_img.clone();
    }

    /// Depth
    PsFrame depthFrame = {0};
    Ps2_GetFrame(deviceHandle, sessionIndex, PsDepthFrame, &depthFrame);
    if (depthFrame.pFrameData != NULL)
    {
        cv::Mat _depth_img = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1, depthFrame.pFrameData);
        depth_img = _depth_img.clone();
    }
   
    /// IR
    PsFrame irFrame = {0};
    Ps2_GetFrame(deviceHandle, sessionIndex, PsIRFrame, &irFrame);
    if (irFrame.pFrameData != NULL)
    {
        //cv::Mat _ir_img = cv::Mat(irFrame.height, irFrame.width, CV_16UC1, irFrame.pFrameData);
        ir_img = cv::Mat(irFrame.height, irFrame.width, CV_16UC1, irFrame.pFrameData);
        //_ir_img.convertTo(ir_img, CV_8U, 255.0 / 3840);
        //cv::Mat ir_img = cv::Mat(irFrame.height, irFrame.width, CV_16UC1, irFrame.pFrameData);
    }

    PsFrame wdrDepthFrame = { 0 };
    Ps2_GetFrame(deviceHandle, sessionIndex, PsWDRDepthFrame, &wdrDepthFrame);
    if (wdrDepthFrame.pFrameData != NULL)
    {
        cv::Mat _wdr_img = cv::Mat(wdrDepthFrame.height, wdrDepthFrame.width, CV_16UC1, wdrDepthFrame.pFrameData);
        wdr_img = _wdr_img.clone();
    }
    
    LOG_WARN("wdrDepthFrame") << depth_img.rows;

    PsFrame confFrame = {0};
    Ps2_GetFrame(deviceHandle, sessionIndex, PsConfidenceFrame, &confFrame);
    if (confFrame.pFrameData != NULL)
    {
        //cv::Mat _ir_img = cv::Mat(irFrame.height, irFrame.width, CV_16UC1, irFrame.pFrameData);
        cv::Mat conf = cv::Mat(confFrame.height, confFrame.width, CV_16UC1, confFrame.pFrameData);
    }
}

void Vzense::start()
{
    cout << "Capture Start " << endl;
}

void Vzense::stop()
{

    PsReturnStatus status;
    status = Ps2_CloseDevice(&deviceHandle);
    cout << "CloseDevice status: " << status << endl;

    status = Ps2_Shutdown();
    cout << "Shutdown status: " << status << endl;
}

cv::Mat
Vzense::getRGBImage()
{
    return color_img;
}

cv::Mat
Vzense::getDepthImage()
{
    return depth_img;
}

cv::Mat
Vzense::getIRImage()
{
    return ir_img;
}
cv::Mat
Vzense::getWdrImage()
{
    return wdr_img;
}
cv::Mat
Vzense::getColorizedDepthImage()
{
    cv::Mat tmp, color_tmp;
    uint32_t slope = 1450;
    depth_img.convertTo(tmp, CV_8U, 255.0 / slope);
    cv::applyColorMap(tmp, color_tmp, cv::COLORMAP_JET);
    colorized_depth_img = color_tmp.clone();
    return colorized_depth_img;
}

CameraParameter
Vzense::getCameraParameter()
{

    PsReturnStatus status;
    PsCameraParameters cameraParameters;
    status = Ps2_GetCameraParameters(deviceHandle, sessionIndex, PsDepthSensor, &cameraParameters);

    camera_param.fx = cameraParameters.fx;
    camera_param.fy = cameraParameters.fy;
    camera_param.cx = cameraParameters.cx;
    camera_param.cy = cameraParameters.cy;
    camera_param.image_width = _image_width;
    camera_param.image_height = _image_height;
    camera_param.base_line = 0.0;
    camera_param.type = SensorType::TOF;

    cout << "Depth Camera Intinsic: " << endl;
    cout << "Fx: " << camera_param.fx << endl;
    cout << "Cx: " << camera_param.cx << endl;
    cout << "Fy: " << camera_param.fy << endl;
    cout << "Cy: " << camera_param.cy << endl;

    return camera_param;
}
void HotPlugStateCallback(const char *uri, int status)
{
    cout << uri << "    " << (status == 0 ? "add" : "remove") << endl;
}
