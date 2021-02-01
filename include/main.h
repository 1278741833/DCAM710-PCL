#ifndef __MAIN__MAIN_
#define __MAIN__MAIN_
#include <math.h>
using namespace std;

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <sys/ipc.h>
#include <unistd.h>
#include <pthread.h>
#include <thread>
#include <memory>
#include <sys/ioctl.h>
#include <stdio.h>
#include <sstream>
#include <sys/time.h>
#include <time.h>
#include <chrono>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <mutex>
#include <signal.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <sched.h>
#include <net/if.h>
#include <float.h>
#include <limits.h>  
#include <unistd.h>  
#include <semaphore.h> 
#include <errno.h>

#include <opencv2/opencv.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "logging.h"
#include "log_severity.h"

#include "glog.h"
#include "ini.h"
#include "INIReader.h"
#include "util.h"
#include "Vzense.h"
#include "commander.h"
extern COMMANDER *sys;
#endif
