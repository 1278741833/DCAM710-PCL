#include "main.h"
using namespace Eigen;


COMMANDER::COMMANDER()
{
	configManager = new INIReader("config.ini");
	updateParams();
}

COMMANDER *COMMANDER::getInstance()
{
	static COMMANDER instance;
	return &instance;
}

void COMMANDER::updateParams()
{

	/*  Camera SENSOR CONFIG   */
	param.image_width = configManager->GetInteger("Camera", "image_width",640);
	cout<<"  param->image_width:"<<param.image_width<<endl;

	param.image_height = configManager->GetInteger("Camera", "image_height",480);
	cout<<"  param->image_height:"<<param.image_height<<endl;

	param.capture_fps = configManager->GetInteger("Camera", "capture_fps",30);
	cout<<"  param->capture_fps:"<<param.capture_fps<<endl;

	param.theta_pitch = configManager->GetInteger("Camera", "theta_pitch",30);
	cout<<"  param->theta_pitch:"<<param.theta_pitch<<endl;
	
}

int COMMANDER::run()
{
	vzense.Threadvzense();
	while(true)
	{
		util::sleep(1); 
	}
	

	return 0;
}
