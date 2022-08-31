#pragma once

//#define WINDOWS

#ifdef WINDOWS
#define INPUT_SOURCE IN_TYPE_PIC
#include "unistdw.h"
#else
#include <unistd.h>   /* For open(), creat() */
#include <termios.h>
//#include <pthread.h>
#define INPUT_SOURCE IN_TYPE_CAM
#endif

#include "log_process_time.hpp"
