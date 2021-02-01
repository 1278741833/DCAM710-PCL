#ifndef __NAV__
#define __NAV__

#include "main.h"


class COMMANDER{
public:

	typedef struct{
    int image_width;
    int image_height;
    int capture_fps;
	int theta_pitch;
	}PARAMS;


public:
	COMMANDER();
	void updateParams();
	static COMMANDER* getInstance();
	int run();

public:	
	pthread_t pthid;
	PARAMS param;
	INIReader *configManager;
	Vzense vzense;
	NETWORK network;
};




#endif
