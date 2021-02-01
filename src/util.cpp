#include "main.h"
#include "util.h"


#include "logging.h"

#include "log_severity.h"


#include "glog.h"




#if 1
void SignalHandle(const char* data, int size)
{
    ofstream fs("./log/glog_dump.log",std::ios::app);
    string str = string(data,size);
    fs<<str;
    fs.close();
    LOG_ERROR("Core dump") << str;
}
GLOG::GLOG()
{
	string s = "rgv";
	initGlogSystem((char *)s.c_str());
}


GLOG::~GLOG()
{
	google::ShutdownGoogleLogging();
}


void GLOG::initGlogSystem(char* filename)
{
    const string logdir = "./glog";
    const string logprefix = "LOG_";
    const int logfilemaxsize = 10;

    system((string("mkdir -p ")+logdir).c_str());

    google::InitGoogleLogging(filename);
    //google::SetStderrLogging(google::ERROR);
    google::SetStderrLogging(google::WARNING);
    //google::SetStderrLogging(google::INFO);

    google::SetLogDestination(google::INFO,(logdir+"/"+logprefix).c_str());

    FLAGS_logtostderr = false;
    FLAGS_colorlogtostderr=true;
    FLAGS_logbufsecs =1;
    FLAGS_max_log_size = logfilemaxsize;

    google::InstallFailureSignalHandler();
    //默认捕捉 SIGSEGV 信号信息输出会输出到 stderr
    google::InstallFailureWriter(&SignalHandle);

    LOG_INFO("Glog") << "Init Glog System Finish.";
    LOG_INFO("Glog") << "Log dir:" << logdir;
    LOG_INFO("Glog") << "Max Log Size:"<< logfilemaxsize<<"MB";
}


#endif


void split(string& s,vector<int>& sv,const char flag) {
	char buf[200];
	
	strcpy(buf,s.c_str());
    sv.clear();
	
	int data[5];
	int data_len = 0;
	
	for(int i=1;i<200;i++)
	{
		if(s[i]>=0x30 && s[i]<=0x39)
		{
			data[data_len] = s[i]-0x30;

			data_len++;
		}
		if(s[i] == 0x22 || s[i] == 0x2c)
		{
			
			int ret = 0;
			int z = 0;
			for(int k=data_len-1;k>=0;k--,z++)
			{

				ret += (int)(pow(10,k))*data[z];		
			}
			sv.push_back(ret);
			data_len = 0;
		}
		if(s[i] == 0x22)
		{
	break;	
		}
	}



}




bool cmpStr(char * s1,char * s2,int cnt)
{
	for(int i=0;i<cnt;i++)
	{
		if(s1[i]!=s2[i])
			return -1;
	}
	return 0;
}




void SplitString(const string& s, vector<string>& v, const string& c)
{
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));
         
        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
}

