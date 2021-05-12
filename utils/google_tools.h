#define GLOG_NO_ABBREVIATED_SEVERITIES

#ifdef WIN32
// Link GLOG static library
#	define GOOGLE_GLOG_DLL_DECL
#	include <windows.h>
#endif

#include <gflags/gflags.h>
#include <glog/logging.h>