#ifndef __UTIL__H__
#define __UTIL__H__

#include <sstream>
#include <string>

#include <ctime>

#include "main.h"



namespace util {



/*
Sleep current thread for a few milliseconds.
*/
inline void sleep(const long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}


}


#if 1


class GLOG
{
public:
	GLOG();
	
	~GLOG();

	void initGlogSystem(char* filename);

};
void split(string&,vector<int>&,const char);

bool cmpStr(char * s1,char * s2,int cnt);

void sleepMs(int num);

unsigned long CRC_32(const unsigned char * aData, int aSize );
void SplitString(const string& s, vector<string>& v, const string& c);



#endif


#endif
