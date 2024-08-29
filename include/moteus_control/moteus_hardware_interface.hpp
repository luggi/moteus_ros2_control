/**
 * @file moteus_hardware_interface.hpp
 * @author Lukas Schöpf (lukas.schoepf@student.uibk.ac.at)
 * @brief This file defines the MoteusHardwareInterface class, which is a 
 *          ROS2 Control Hardware System Type Interface.
 * @details implementation follows the ROS2 documentation found at: 
 *              <https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html>
 * @version 0.1
 * @date 2024-07-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MOTEUS_CONTROL__MOTEUS_HARDWARE_INTERFACE_HPP_
#define MOTEUS_CONTROL__MOTEUS_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <optional>
#include <ctime>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "moteus_transport.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "moteus.h"

namespace moteus_hardware_interface
{
    class MoteusHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(MoteusHardwareInterface)

        ~MoteusHardwareInterface() override;

        /**
         * @details on_init is expected to vary depending on the URDF file. 
         *          The SystemInterface::on_init(info) call fills out the info 
         *          object with specifics from the URDF. For example, 
         *          the info object has fields for joints, sensors, gpios, and more. 
         *          Suppose the sensor field has a name value of tcp_force_torque_sensor. 
         *          Then the on_init must try to establish communication with that sensor. 
         *          If it fails, then an error value is returned.
         * 
         * @param info 
         * @return hardware_interface::CallbackReturn 
         */
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        /**
         * @brief Called after the on_init function. Setup the communication to the hardware and set everything up so that the hardware can be activated.
         * 
         * @param previous_state 
         * @return hardware_interface::CallbackReturn 
         */
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Does opposite of on_configure()
         * 
         * @param previous_state
        */
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Exposes state interfaces for proper initialization of ROS2 control topics
         * 
         * @return std::vector<hardware_interface::StateInterface> 
         */
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        /**
         * @brief Exports the command interfaces for proper initialization of ROS2 control topics
         * 
         * 
         * @return std::vector<hardware_interface::CommandInterface> 
         */
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        /**
         * @brief Called after on_configure. This is where the hardware is activated -- "powered up"
         * 
         * @param previous_state 
         * @return hardware_interface::CallbackReturn 
         */
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Opposite of on_activate. This is where the hardware is deactivated -- "powered down"
         * 
         * @param previous_state 
         * @return hardware_interface::CallbackReturn 
         */
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Core function of the ROS2 control loop. In the case of the moteus interface, this function does nothing.
         * 
         * @param time 
         * @param period 
         * @return hardware_interface::return_type 
         */
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /**
         * @brief Carries out CAN frame writing and reading to control and read feedback from actuators.
         * 
         * @param time 
         * @param period 
         * @return hardware_interface::return_type 
         */
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;
        
    private:
        // Utility functions for converting between double and uint
        int double_to_uint(double x, double x_min, double x_max,
                          int bits);
        double uint_to_double(int x_int, double x_min, double x_max,
                            int bits);

        // Utility functions for dealing with angle wrapping
        double wrap_angle(double angle, double angle_min, double angle_max);
        double unwrap_angle(double angle, double prev_angle, double angle_min,
                           double angle_max);

        void update_state_interfaces();

        void busy_wait_us(unsigned long microsec) {
            // Convert microseconds to clock ticks. std::CLOCKS_PER_SEC is the number of clock ticks per second.
            std::clock_t end = std::clock() + (microsec * CLOCKS_PER_SEC / 1000000);
            
            while (std::clock() < end) {
                // Busy wait
            }
        }

        int findIndex(const std::vector<int>& vec, int value) {
            for (size_t i = 0; i < vec.size(); ++i) {
                if (vec[i] == value) {
                    return i; // Found the element, return its index
                }
            }
            return -1; // Element not found, return -1
        };

        // Actuator CAN config
        std::vector<int> hw_actuator_can_channels_;
        std::vector<unsigned int> hw_actuator_can_ids_;

        // moteus controllers
        std::map<int, std::shared_ptr<mjbots::moteus::Controller>> controllers_;
        std::map<int, mjbots::moteus::Query::Result> servo_data_;
        std::vector<mjbots::moteus::CanFdFrame> command_frames_;
        std::vector<mjbots::moteus::CanFdFrame> replies_;

        // transport interface
        std::shared_ptr<mjbots::moteus::Transport> transport_;


        // Actuator parameters
        std::vector<int> hw_actuator_axis_directions_;
        std::vector<double> hw_actuator_position_offsets_;
        std::vector<double> hw_actuator_gear_ratios_;
        std::vector<double> hw_actuator_torque_constants_;
        std::vector<int> hw_actuator_soft_start_durations_ms_;

        // Actuator limits
        std::vector<double> hw_actuator_position_mins_; 
        std::vector<double> hw_actuator_position_maxs_;
        std::vector<double> hw_actuator_velocity_limits_;
        std::vector<double> hw_actuator_power_limits_;

        // Actuator motor states
        std::vector<double> hw_state_positions_;
        std::vector<double> hw_state_velocities_;
        std::vector<double> hw_state_efforts_;

        // Actuator status/misc states
        std::vector<double> hw_state_temperatures_;
        std::vector<int> hw_state_errors_;
        std::vector<int> hw_state_states_;
        std::vector<double> hw_state_voltages_;
        
        // Actuator commands
        std::vector<double> hw_command_positions_;
        std::vector<double> hw_command_velocities_;
        std::vector<double> hw_command_efforts_;
        std::vector<double> hw_command_kps_;
        std::vector<double> hw_command_kds_;
        std::vector<double> hw_command_kis_;
    };

} // namespace moteus_hardware_interface

#endif