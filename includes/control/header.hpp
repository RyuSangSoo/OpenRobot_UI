#ifndef HEADER_HPP
#define HEADER_HPP

#include <iostream>
#include <stdio.h>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <csignal>
#include <cerrno>
#include <cstring>

#include <linux/joystick.h>
#include <chrono>
#include <rbdl/rbdl.h>
#include <Eigen/Dense>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <cmath>

#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <ctime>
#include <cstdlib>

#include <unordered_map>

#ifdef USE_OSQP_EIGEN
#include "OsqpEigen/OsqpEigen.h"
#endif

#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)

#endif