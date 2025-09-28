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

#ifndef TINKERFORGE_BRICKLET_STEPPER_DRIVER_H
#define TINKERFORGE_BRICKLET_STEPPER_DRIVER_H

#define IPCON_EXPOSE_MILLISLEEP

#include <string>
#include <stdio.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>

#include "../tinkerforge/ip_connection.h"
#include "../tinkerforge/bricklet_silent_stepper_v2.h"


class StepperDriver {
    public:
      // constructor
      StepperDriver(
          std::string bricklet_host,
          int bricklet_port,
          std::string brick_position,
          int step_resolution,
          bool interpolation,
          int acceleration,
          int deceleration,
          int steps_per_revolution,
          int gear_ratio,
          int max_step_vel,
          int standstill_current = 200,
          int motor_run_current = 800,
          int standstill_delay_time = 300,
          int power_down_time = 5000,
          int stealth_threshold = 1000,
          int coolstep_threshold = 1001,
          int classic_threshold = 2000,
          bool high_velocity_chopper_mode = false
      );
      // destructor
      ~StepperDriver()
      {
		  shutdownStepper();
          ipcon_destroy(&ipcon_);
      }
      // init
      int init();
      void run();
      void drive_forward();
      void drive_backward();
      void stop();
      // parameters for the driver state


      int get_current_velocity();
      int get_current_position();
      int get_new_state();
      int get_previous_state();
      void set_velocity(double cmd_vel);



    private:
        IPConnection ipcon_;
        SilentStepperV2 brickletStepperV2_;

        // parameters set by launch file
        int step_resolution_;
        int steps_per_revolution_;
        int gear_ratio_;
        int max_step_vel_;
        int acceleration_;
        int deceleration_;
        int wheel_name_;
        std::string brick_position_;
        int bricklet_port_;
        std::string bricklet_host_;
        bool stepper_enabled_;
        bool interpolation_;
        bool bricklet_is_configured_;
        bool under_voltage_triggerd_;
        bool reset_under_voltage_triggered_;
        // parameters for motor (hopefully they don't need to be parameters to be changed from outside
        int standstill_current_;
        int motor_run_current_;
        int standstill_delay_time_;
        int power_down_time_;
        int stealth_threshold_;
        int coolstep_threshold_;
        int classic_threshold_;
        bool high_velocity_chopper_mode_;


        // hardware info coming from the bricklet
        std::string hw_bricklet_uid_;
        std::string hw_bricklet_connected_uid_;
        char hw_bricklet_position_;
        uint8_t hw_bricklet_hardware_version_[3];
        uint8_t hw_bricklet_firmware_version_[3];
        uint16_t hw_bricklet_device_identifier_;
        uint8_t hw_bricklet_enumeration_type_;

        // variables that represent the state of the stepper motor
        int current_velocity_;
  		int current_position_;
  		int remaining_steps_;
  		int input_voltage_;
  		int current_consumption_;
        int position_;
        /*
			SILENT_STEPPER_V2_STATE_STOP = 1
			SILENT_STEPPER_V2_STATE_ACCELERATION = 2
			SILENT_STEPPER_V2_STATE_RUN = 3
			SILENT_STEPPER_V2_STATE_DEACCELERATION = 4
			SILENT_STEPPER_V2_STATE_DIRECTION_CHANGE_TO_FORWARD = 5
			SILENT_STEPPER_V2_STATE_DIRECTION_CHANGE_TO_BACKWARD = 6
         */
        int state_new_;
        /*
			SILENT_STEPPER_V2_STATE_STOP = 1
			SILENT_STEPPER_V2_STATE_ACCELERATION = 2
			SILENT_STEPPER_V2_STATE_RUN = 3
			SILENT_STEPPER_V2_STATE_DEACCELERATION = 4
			SILENT_STEPPER_V2_STATE_DIRECTION_CHANGE_TO_FORWARD = 5
			SILENT_STEPPER_V2_STATE_DIRECTION_CHANGE_TO_BACKWARD = 6
         */
        int state_previous_;
    	void shutdownStepper();
		void stepperEnabled();


        static void cb_enumerate(const char *uid, const char *connected_uid,
                  char position, uint8_t hardware_version[3],
                  uint8_t firmware_version[3], uint16_t device_identifier,
                  uint8_t enumeration_type, void *user_data);
        // try to do this without static function:
        static void cb_connected(uint8_t reason, void *user_data);
        static void cb_all_data(uint16_t current_velocity,
                                int32_t current_position,
                                int32_t remaining_steps,
                                uint16_t input_voltage,
                                uint16_t current_consumption,
                                void *user_data);
        static void cb_position_reached(int32_t position, void *user_data);
        static void cb_new_state(uint8_t state_new, uint8_t state_previous, void *user_data);

        bool ret_enabled_;
};



#endif //TINKERFORGE_BRICKLET_STEPPER_DRIVER_H
