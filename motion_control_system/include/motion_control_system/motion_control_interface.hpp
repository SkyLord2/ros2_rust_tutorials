#ifndef __MOTION_CONTROL_INTERFACE_HPP__
#define __MOTION_CONTROL_INTERFACE_HPP__

namespace motion_control_system
{ 
    class MotionController
    {
    private:
        /* data */
    public: 
        virtual void start() = 0;
        virtual void stop() = 0;
    };
}

#endif