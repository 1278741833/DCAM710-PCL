
#include "main.h"
// One (and only one) of your C++ files must define CVUI_IMPLEMENTATION
// before the inclusion of cvui.h to ensure its implementaiton is compiled.
#define CVUI_IMPLEMENTATION
#include "cvui.h"
COMMANDER *sys;
using namespace std;
using namespace cv;




int main(int argc, char** argv)
{
   	GLOG   mylog;
	
	sys = COMMANDER::getInstance();

    return sys->run();
    
}
