/*
Copyright (C) 2025 Christian Ehrmann

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "cmeresearch_stepper_driver/stepper_driver_ros_wrapper.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("stepper_bricklet");
    bool myparam = true;
    SilentStepperDriverWrapper stepper(node, myparam);

    if (!stepper.initialize()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize SilentStepperDriverWrapper");
        return EXIT_FAILURE;
    }



    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}