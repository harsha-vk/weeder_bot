#ifndef __WEEDER_BOT__WEEDER_BOT_HARDWARE_HPP__
#define __WEEDER_BOT__WEEDER_BOT_HARDWARE_HPP__

#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>
#include <vector>
#include "weeder_bot/visibility_control.h"

namespace weeder_bot
{
    class WeederBotHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(WeederBotHardware);

            WEEDER_BOT_HARDWARE_PUBLIC
            hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

            WEEDER_BOT_HARDWARE_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            WEEDER_BOT_HARDWARE_PUBLIC
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            WEEDER_BOT_HARDWARE_PUBLIC
            hardware_interface::return_type start() override;

            WEEDER_BOT_HARDWARE_PUBLIC
            hardware_interface::return_type stop() override;

            WEEDER_BOT_HARDWARE_PUBLIC
            hardware_interface::return_type read() override;

            WEEDER_BOT_HARDWARE_PUBLIC
            hardware_interface::return_type write() override;

        private:
            // Parameters for the DiffBot simulation
            double hw_start_sec_;
            double hw_stop_sec_;

            // Store the command for the simulated robot
            std::vector<double> hw_commands_;
            std::vector<double> hw_positions_;
            std::vector<double> hw_velocities_;

            // Store the wheeled robot position
            double base_x_, base_y_, base_theta_;
    };
}  // namespace weeder_bot

#endif  // __WEEDER_BOT__WEEDER_BOT_HARDWARE_HPP__