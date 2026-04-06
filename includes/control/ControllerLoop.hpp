#ifndef CONTROLLER_LOOP_HPP
#define CONTROLLER_LOOP_HPP

#include "MjuRobot.hpp"
#include "MjuJoy.hpp"
#include "MjuJoint.hpp"
#include "RssController.hpp"

#include <filesystem>
#include <fstream>
#include <string>

class ControllerLoop {
public:
    ControllerLoop(const std::string& urdf_path,
                   std::filesystem::path test_log_dir);
    void Run();

private:
    std::string urdf_path_;
    std::filesystem::path test_log_dir_;
};

#endif
