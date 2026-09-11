#ifndef __SPIN_MOTION_CONTROLLER_HPP__
#define __SPIN_MOTION_CONTROLLER_HPP__

#include "motion_control_system/motion_control_interface.hpp"

namespace motion_control_system {
    class SpinMotionController : public MotionController
    {
    private:
        /* data */
    public: 
        virtual void start() override;
        virtual void stop() override;
    };
}

#endif