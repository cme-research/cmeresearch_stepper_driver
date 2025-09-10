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

#include <stdio.h>
#include <chrono>
#include <iostream>
#include <sstream>
#include <fstream>
#include "cmeresearch_stepper_driver/tinkerforge_bricklet_stepper_driver.hpp"



StepperDriver::StepperDriver(
  std::string bricklet_host,
  int bricklet_port,
  std::string brick_position,
  int step_resolution,
  bool interpolation,
  int acceleration,
  int deceleration,
  int step_per_revolution,
  int gear_ratio,
  int max_step_vel,
  int standstill_current,
  int motor_run_current,
  int standstill_delay_time,
  int power_down_time,
  int stealth_threshold,
  int coolstep_threshold,
  int classic_threshold,
  bool high_velocity_chopper_mode)
{
  bricklet_host_ = bricklet_host;
  bricklet_port_ = bricklet_port;
  bricklet_is_configured_ = false;
  brick_position_ = brick_position;
  step_resolution_ = step_resolution;
  interpolation_ = interpolation;
  acceleration_ = acceleration;
  deceleration_ = deceleration;
  steps_per_revolution_ = step_per_revolution;
  gear_ratio_ = gear_ratio;
  max_step_vel_ = max_step_vel;
  current_velocity_ = 0;
  current_position_ = 0;
  current_consumption_ = 0;
  input_voltage_ = 0;
  remaining_steps_ = 0;
  stepper_enabled_ = false;

  // Initialize the parameters that were previously hard-coded
  standstill_current_ = standstill_current;
  motor_run_current_ = motor_run_current;
  standstill_delay_time_ = standstill_delay_time;
  power_down_time_ = power_down_time;
  stealth_threshold_ = stealth_threshold;
  coolstep_threshold_ = coolstep_threshold;
  classic_threshold_ = classic_threshold;
  high_velocity_chopper_mode_ = high_velocity_chopper_mode;
}


int StepperDriver::init() {
      ipcon_create(&ipcon_);
      //silent_stepper_v2_create(&brickletStepperV2_, UID, &ipcon_);
	  int e_code = ipcon_connect(&ipcon_, bricklet_host_.c_str(), bricklet_port_);
      if (e_code != 0) {
        fprintf(stderr, "Connection failed! Wrong host configuration, wrong port specified or access denied. Errorcode = %d \n", e_code);
        return 1;
      }

      //TODO!!!!!
      // params yet to be set in initialize (for testing only)



      // register enumeration callback to "cb_enumerate"
      ipcon_register_callback(&ipcon_, IPCON_CALLBACK_CONNECTED, (void (*) (void))cb_connected, &ipcon_);
      ipcon_register_callback(&ipcon_, IPCON_CALLBACK_ENUMERATE, (void (*) (void))cb_enumerate, this);

	  fprintf(stderr, "Connected to Bricklet Stepper Driver\n");
      return 0;
}

void StepperDriver::cb_connected(uint8_t reason, void *user_data) {
  (void) reason;
  (void) user_data;
  fprintf(stderr, "(Re)Connected to Bricklet Stepper Driver. Reason is : %d \n", reason);
  StepperDriver *stepperDriver = (StepperDriver *)user_data;

  fprintf(stderr, "Calling enumeration!\n");
  ipcon_enumerate(&(stepperDriver->ipcon_));
//  fprintf(stderr, "Connected to Bricklet Stepper Driver\n");
//  fprintf(stderr, reason == 0 ? "Reconnected to Bricklet Stepper Driver\n" : "Disconnected from Bricklet Stepper Driver\n");
}

void StepperDriver::cb_enumerate(const char *uid, const char *connected_uid,
                  char position, uint8_t hardware_version[3],
                  uint8_t firmware_version[3], uint16_t device_identifier,
                  uint8_t enumeration_type, void *user_data) {
  StepperDriver *stepperDriver = (StepperDriver *)user_data;

  fprintf(stderr, "cb_enumerate\n");
  if(enumeration_type == IPCON_ENUMERATION_TYPE_CONNECTED ||
      enumeration_type == IPCON_ENUMERATION_TYPE_AVAILABLE)
  {
    fprintf(stderr, "Found Bricklet with UID %s\n", uid);
    fprintf(stderr, "Enumeration Type: %d\n", enumeration_type);
    fprintf(stderr, "Connected UID:     %s\n", connected_uid);
    fprintf(stderr, "Position:          %c\n", position);
    fprintf(stderr, "Hardware Version:  %d.%d.%d\n", hardware_version[0],
                                            hardware_version[1],
                                            hardware_version[2]);
    fprintf(stderr, "Firmware Version:  %d.%d.%d\n", firmware_version[0],
                                            firmware_version[1],
                                            firmware_version[2]);
    fprintf(stderr, "Device Identifier: %d\n", device_identifier);

    if(device_identifier == SILENT_STEPPER_V2_DEVICE_IDENTIFIER) {
        // convert position char to a string to compare
        // 0 - 8 for master bricks
        // a - h for bricklets
        fprintf(stderr, "device_identifier %d\n", device_identifier);
        fprintf(stderr, "SILENT_STEPPER_V2_DEVICE_IDENTIFIER = %d\n", SILENT_STEPPER_V2_DEVICE_IDENTIFIER);
        fprintf(stderr, "device_identifier correct --------------------.\n");
		std::string hw_bricklet_position_str(1, position);
 		if (hw_bricklet_position_str == stepperDriver->brick_position_) {
    	    fprintf(stderr, "bricklet position is correct.\n");

            //fprintf(stderr, "bricklet created.\n");
            stepperDriver->hw_bricklet_uid_ = uid;
  			stepperDriver->hw_bricklet_connected_uid_ = connected_uid;
  			stepperDriver->hw_bricklet_position_ = position;
  			stepperDriver->hw_bricklet_hardware_version_[0] = hardware_version[0];
  			stepperDriver->hw_bricklet_hardware_version_[1] = hardware_version[1];
  			stepperDriver->hw_bricklet_hardware_version_[2] = hardware_version[2];
  			stepperDriver->hw_bricklet_firmware_version_[0] = firmware_version[0];
  			stepperDriver->hw_bricklet_firmware_version_[1] = firmware_version[1];
  			stepperDriver->hw_bricklet_firmware_version_[2] = firmware_version[2];
  			stepperDriver->hw_bricklet_device_identifier_ = device_identifier;
  			stepperDriver->hw_bricklet_enumeration_type_ = enumeration_type;

  			fprintf(stderr, "create.\n");

  	  		silent_stepper_v2_create(&(stepperDriver->brickletStepperV2_), stepperDriver->hw_bricklet_uid_.c_str(), &(stepperDriver->ipcon_));
	  		silent_stepper_v2_set_step_configuration(&(stepperDriver->brickletStepperV2_), SILENT_STEPPER_V2_STEP_RESOLUTION_8, true);
            fprintf(stderr, "bricklet step configuration set.\n");
        	silent_stepper_v2_set_motor_current(&(stepperDriver->brickletStepperV2_), stepperDriver->motor_run_current_);
            silent_stepper_v2_set_max_velocity(&(stepperDriver->brickletStepperV2_), stepperDriver->max_step_vel_);
            silent_stepper_v2_set_speed_ramping(&(stepperDriver->brickletStepperV2_), stepperDriver->acceleration_, stepperDriver->deceleration_);
            fprintf(stderr, "bricklet speed ramping set.\n");
            silent_stepper_v2_set_basic_configuration(&(stepperDriver->brickletStepperV2_),
                                                    stepperDriver->standstill_current_,
                                                    stepperDriver->motor_run_current_,
                                                    stepperDriver->standstill_delay_time_,
                                                    stepperDriver->power_down_time_,
                                                    stepperDriver->stealth_threshold_,
                                                    stepperDriver->coolstep_threshold_,
                                                    stepperDriver->classic_threshold_,
                                                    stepperDriver->high_velocity_chopper_mode_);
  			fprintf(stderr, "bricklet basic configuration set.\n");
            silent_stepper_v2_set_all_callback_configuration(&(stepperDriver->brickletStepperV2_), 100);
            fprintf(stderr, "bricklet all callback configuration set.\n");
            silent_stepper_v2_register_callback(&(stepperDriver->brickletStepperV2_),
                                        SILENT_STEPPER_V2_CALLBACK_POSITION_REACHED,
                                        (void (*)(void))cb_position_reached,
                                        &(stepperDriver->brickletStepperV2_));
            fprintf(stderr, "position reached callback registered.\n");
            //silent_stepper_v2_set_enabled(&brickletStepperV2_, true); // Enable motor power
            silent_stepper_v2_register_callback(&(stepperDriver->brickletStepperV2_),
                                        SILENT_STEPPER_V2_CALLBACK_ALL_DATA,
                                        (void (*) (void)) cb_all_data,
                                        &(stepperDriver->brickletStepperV2_));
            stepperDriver->bricklet_is_configured_ = true;
    	    fprintf(stderr, "Hardware StepperDriver device created..\n");
	  	}
        else
        {
            fprintf(stderr, "hw_bricklet_position NOT correct.\n");
        }
    }
    else
    {
        fprintf(stderr, "hw_bricklet_device_identifier NOT correct.\n");
    }
  }
}

void StepperDriver::cb_all_data(uint16_t current_velocity,
                                int32_t current_position,
                                int32_t remaining_steps,
                                uint16_t input_voltage,
                                uint16_t current_consumption,
                                void *user_data)
{
  StepperDriver *stepperDriver = (StepperDriver *)user_data;

  //fprintf(stderr, "StepperDriver::current_velocity %d\n", current_velocity);
  stepperDriver->current_velocity_ = current_velocity;
  stepperDriver->current_position_ = current_position;
  stepperDriver->remaining_steps_ = remaining_steps;
  stepperDriver->input_voltage_ = input_voltage;
  stepperDriver->current_consumption_ = current_consumption;

}

void StepperDriver::cb_position_reached(int32_t position, void *user_data) {
  StepperDriver *stepperDriver = (StepperDriver *)user_data;
  stepperDriver->current_position_ = position;

  fprintf(stderr, "StepperDriver::cb_position_reached\n");
}

void StepperDriver::cb_new_state(uint8_t state_new, uint8_t state_previous, void *user_data) {
  StepperDriver *stepperDriver = (StepperDriver *)user_data;
  stepperDriver->state_new_ = state_new;
  stepperDriver->state_previous_ = state_previous;

}


void StepperDriver::run() {
	fprintf(stderr, "run\n");
}


void StepperDriver::shutdownStepper() {
   fprintf(stderr, "shutdownStepper\n");
   silent_stepper_v2_stop(&brickletStepperV2_);
   millisleep(400);
   silent_stepper_v2_set_enabled(&brickletStepperV2_, false);
   silent_stepper_v2_destroy(&brickletStepperV2_);
}

void StepperDriver::stepperEnabled() {
	fprintf(stderr, "stepperEnabled\n");
    silent_stepper_v2_set_enabled(&brickletStepperV2_, true);
	stepper_enabled_ = true;
}


void StepperDriver::stop() {
	fprintf(stderr, "stop\n");
}


void StepperDriver::drive_forward() {
    //fprintf(stderr, "drive forward\n");
	int e_code = silent_stepper_v2_get_enabled(&brickletStepperV2_, &ret_enabled_);
	//fprintf(stderr, "get_enabled call success??: %d\n", e_code);
	//fprintf(stderr, "enabled: %d\n", ret_enabled_);
	if (bricklet_is_configured_) {
		if (stepper_enabled_ == false) {
  			this->stepperEnabled();
		}


	silent_stepper_v2_drive_forward(&brickletStepperV2_);
	}


}


void StepperDriver::drive_backward() {
  if (bricklet_is_configured_) {
    //fprintf(stderr, "drive backward\n");
    if (stepper_enabled_ == false) {
      this->stepperEnabled();
  	}
  	silent_stepper_v2_drive_backward(&brickletStepperV2_);
  }
}

int StepperDriver::get_current_velocity() {
	return bricklet_is_configured_ ? current_velocity_ : 0;
}

int StepperDriver::get_current_position() {
  return bricklet_is_configured_ ? current_position_ : 0;
}

int StepperDriver::get_new_state() {
  return bricklet_is_configured_ ? state_new_ : 0;
}

int StepperDriver::get_previous_state() {
  return bricklet_is_configured_ ? state_previous_ : 0;
}

void StepperDriver::set_velocity(double cmd_vel) {

  // cmd_vel in rad/s to steps/s
  double revolutions_per_sec = (cmd_vel * gear_ratio_) / (2 * M_PI);
  uint16_t steps_per_sec = std::round(revolutions_per_sec * steps_per_revolution_);
  fprintf(stderr, "set_velocity in steps per second %d\n", steps_per_sec);
  silent_stepper_v2_set_max_velocity(&brickletStepperV2_, steps_per_sec);
}
