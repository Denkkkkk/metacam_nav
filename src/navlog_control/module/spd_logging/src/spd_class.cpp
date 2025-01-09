#include "navlog_control/spd_class.h"

void spd_class::create_log_directory_if_not_exists(const std::string& dir_path) {
    std::cout << "create_log_directory_if_not_exists running" << std::endl;
    if (!fs::exists(dir_path)) {
        std::cout << "Directory does not exist. Creating: " << dir_path << std::endl;
        if (!fs::create_directories(dir_path)) {
            std::cerr << "Failed to create directory: " << dir_path << std::endl;
            // Handle the error accordingly
        }
    }
    else {
        std::cout << "Directory already exists: " << dir_path << std::endl;
    }
}
