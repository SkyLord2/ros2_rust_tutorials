#include <motion_control_system/motion_control_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <iostream>

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: " << argv[0] << " plugin_name" << std::endl;
        return 1;
    }
    
    std::string plugin_name = argv[1];
    pluginlib::ClassLoader<motion_control_system::MotionController> loader("motion_control_system", "motion_control_system::MotionController");

    auto controller = loader.createSharedInstance(plugin_name);

    controller->start();
    controller->stop();

    return 0;
}