#pragma once

//#include <string>
#include <std_msgs/Bool.h>
#include <ros/console.h>
#include <ros/node_handle.h>

//#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/hardware_interface.h>

namespace rbs_interface
{

    // /**
    //  * Handle to read the complete state of a robot.
    //  */
    // class ProgramSafetyHandle
    // {
    // public:
    //     ProgramSafetyHandle() = delete;

    //     ProgramSafetyHandle(const std::string &name)
    //         : name_(name) {}

    //     /**
    //      * Gets the name of the state handle.
    //      *
    //      * @return Name of the state handle.
    //      */
    //     const std::string &getName() const noexcept { return name_; }


    // private:
    //     std::string name_;
    // };

    //class ProgramSafetyInterface : public hardware_interface::HardwareInterface
    class ProgramSafetyInterface 
    {

    public:
        ProgramSafetyInterface() {
                        ROS_INFO("Creating RBS interface");

        };
        ~ProgramSafetyInterface() {};
        ProgramSafetyInterface(ros::NodeHandle& node_handle)
        {
            ROS_INFO("Loading RBS interface");

            active_= false;
            activate_trigger_ = node_handle.subscribe(
                "activate", 1, &ProgramSafetyInterface::activateCallback, this,
                ros::TransportHints().reliable().tcpNoDelay());
        }

        void activateCallback(
            const std_msgs::BoolConstPtr &msg)
        {
            ROS_INFO("Controller activated");
            active_ = msg->data;
        }

        bool isActive()
        {
            return active_;
        }

    private:
        bool active_;
        ros::Subscriber activate_trigger_;

    };

}