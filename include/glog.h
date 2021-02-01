#ifndef __GLOG__H__
#define __GLOG__H__

#define LOG_INFO(keyword)   LOG(INFO)<<"["<<keyword<<"]"
#define LOG_WARN(keyword)   LOG(WARNING)<<"["<<keyword<<"]"
#define LOG_ERROR(keyword)  LOG(ERROR)<<"["<<keyword<<"]"
#define LOG_FATAL(keyword)  LOG(FATAL)<<"["<<keyword<<"]"
#define LOG_EVERY_NUM(keyword, severity, n)   LOG_EVERY_N(severity, n)<<"["<<keyword<<"]"
#define LOG_FIRST_NUM(keyword, severity, n)   LOG_FIRST_N(severity, n)<<"["<<keyword<<"]"


#endif 

