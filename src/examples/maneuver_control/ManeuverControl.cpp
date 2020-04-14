/****************************************************************************
 *
 *
 *
 *
 ****************************************************************************/

/**
 * @file ManeuverControl.cpp
 * Adaptation of the <gnd_att_control> module to perform automatic programmed
 * maneuvers for aerodynamic identification. The maneuver to be performed is specified
 * from the ground by setting the relevant parameters (duration, amplitude, type...) and
 * the maneuver is triggered by a parameter value set from the ground station and the
 * release of the safety switch by the pilot;
 *
 * @author Alberto Ruiz Garcia <aruizgarcia-1@tudelft.nl>
 */

#include "ManeuverControl.hpp"


/**
 * Maneuver Control app start / stop handling function
 *
 * @ingroup apps
 */

extern "C" __EXPORT int maneuver_control_main(int argc, char *argv[]);

static void usage(const char *reason)
{
        if (reason != nullptr) {
                PX4_WARN("%s", reason);
        }

        PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
### Description
This module consists of a controller to perform automatic preprogrammed maneuvers.
The parameters to define the maneuvers can be tuned on the fly so that a large number
of maneuvers can be performed during a single flight.

Thrust is controlled manually by the pilot and the rest of the inputs are monitored,
in case they increase above a defined threshold, the controlled is stopped for safety
and manual control is immediately regained.

### Examples
Example of defining and sending a maneuver.
maneuver_control define -c elevator -a 37 -t 20 -m 3211
maneuver_control safety_off
maneuver_control start_maneuver
...

)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("maneuver_control", "command");
        PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Starts controller");
        PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stops controller");
        PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Running/not running");
        PRINT_MODULE_USAGE_COMMAND_DESCR("info", "Current maneuver parameters");
        PRINT_MODULE_USAGE_COMMAND_DESCR("safety_on", "Sets the PC safety switch to ON ");
        PRINT_MODULE_USAGE_COMMAND_DESCR("safety_off", "Sets the PC safety switch to OFF ");
        PRINT_MODULE_USAGE_COMMAND_DESCR("maneuver_start", "Starts maneuver immediately if safety is off");
        PRINT_MODULE_USAGE_COMMAND_DESCR("abort", "Sets safety back ON and stops the maneuver");
        PRINT_MODULE_USAGE_COMMAND_DESCR("define -<flag> <value>", "Sets the parameter(s) for the maneuver");
        PRINT_MODULE_USAGE_PARAM_COMMENT("Flags:");  
        PRINT_MODULE_USAGE_PARAM_FLAG('c', "Control surface for the maneuver [elevator, ailerons, rudders]", true);
        PRINT_MODULE_USAGE_PARAM_FLAG('a', "Desired amplitude for the maneuver [%]", true);
        PRINT_MODULE_USAGE_PARAM_FLAG('t', "Maneuver duration [s]", true);
        PRINT_MODULE_USAGE_PARAM_FLAG('m', "Maneuver type [doublet/3211]", true);

        PRINT_MODULE_USAGE_PARAM_COMMENT("These parameters apply to all commands:");
        PRINT_MODULE_USAGE_PARAM_FLAG('v', "Verbose output", true);
        PRINT_MODULE_USAGE_PARAM_FLAG('e', "Exit with 1 instead of 0 on error", true);
        PRINT_MODULE_USAGE_PARAM_COMMENT("Example: 20 seconds, 50% amplitude doublet in the elevator");
        PRINT_MODULE_USAGE_PARAM_COMMENT("         maneuver_control define -c elevator -a 50 -m doublet -t 20"); 

}


namespace maneuver_controller
{
	ManeuverControl *control_ptr = nullptr;
}

// Constructor
ManeuverControl::ManeuverControl()
{
	parameters_init();  // Get parameter handles
	parameters_update(); // Update parameter values
}

// Destructor
ManeuverControl::~ManeuverControl()
{
	if (_control_task != -1) {
		_task_should_exit = true;

		// Wait for a second for the task to exit
		unsigned i = 0;
		do{
			// wait 20 ms
			px4_usleep(20000);

			if (++i > 50){
				px4_task_delete(_control_task);
				break;
			}
		} while(_control_task != -1);
	}

	maneuver_controller::control_ptr = nullptr;
}

void
ManeuverControl::parameters_init()
{
	/* Parameter handles*/
	_parameter_handles.safety_switch_pc =   param_find("MAN_PC_SWITCH");
	_parameter_handles.safety_switch_tx =   param_find("MAN_TX_SWITCH");
	_parameter_handles.start_maneuver 	=	param_find("MAN_START");
	_parameter_handles.maneuver_id 		=	param_find("MAN_ID");
	_parameter_handles.control_surface 	=	param_find("MAN_CTRL_SURF");
	_parameter_handles.duration 		=	param_find("MAN_DURATION");
	_parameter_handles.amplitude 		=	param_find("MAN_AMPLITUDE");
	_parameter_handles.center_pos 		=	param_find("MAN_CENTER_POS");
    _parameter_handles.tx_pos_abort     =   param_find("MAN_TX_ABORT");

    return;
}

void
ManeuverControl::parameters_update()
{
	param_get(_parameter_handles.safety_switch_pc, &(_parameters.safety_switch_pc));
	param_get(_parameter_handles.safety_switch_tx, &(_parameters.safety_switch_tx));
	param_get(_parameter_handles.start_maneuver, &(_parameters.start_maneuver));
	param_get(_parameter_handles.maneuver_id, &(_parameters.maneuver_id));
	param_get(_parameter_handles.control_surface, &(_parameters.control_surface));
	param_get(_parameter_handles.duration, &(_parameters.duration));
	param_get(_parameter_handles.amplitude, &(_parameters.amplitude));
	param_get(_parameter_handles.center_pos, &(_parameters.center_pos));
	param_get(_parameter_handles.tx_pos_abort, &(_parameters.tx_pos_abort));
    param_get(_parameter_handles.trim_roll, &(_parameters.trim_roll));
	param_get(_parameter_handles.trim_pitch, &(_parameters.trim_pitch));
	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));
	param_get(_parameter_handles.man_roll_scale, &(_parameters.man_roll_scale));
	param_get(_parameter_handles.man_pitch_scale, &(_parameters.man_pitch_scale));
	param_get(_parameter_handles.man_yaw_scale, &(_parameters.man_yaw_scale));
	return;
}


void
ManeuverControl::vehicle_control_mode_poll()
{
	bool updated = false;
	orb_check(_vcontrol_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
ManeuverControl::manual_control_setpoint_poll()
{
	bool updated = false;
	orb_check(_manual_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual_sp);
	}
}

void
ManeuverControl::set_start_flag(int option)
{
	param_set(_parameter_handles.start_maneuver, &option);
}

float
ManeuverControl::compute_maneuver (uint64_t maneuver_time){

        float actuator_command = 0.0f;

        if(_parameters.maneuver_id == 1) {
                actuator_command = compute_doublet(maneuver_time, _parameters.duration, _parameters.amplitude);
        } else if (_parameters.maneuver_id == 2) {
                actuator_command = compute_3211(maneuver_time, _parameters.duration, _parameters.amplitude);
        } else { // Not recognized, should never happen
                actuator_command = 0.0f;
                _run_controller = false; // Stop controller
                PX4_ERR("Maneuver not set!");
        }

        if (maneuver_time > _parameters.duration * 1000U){
                _run_controller = false;
                set_safety_pc(PC_SAFETY_ON);
                printf("\t\t>>> Done, PC safety back on\n");
        }

        return actuator_command;
}


float
ManeuverControl::compute_doublet(uint64_t maneuver_time, uint64_t maneuver_duration, float amplitude) {

	const uint64_t default_duration = 10; // 10 s
	double scale_ratio = maneuver_duration/default_duration;
	unsigned steps_timing_us[] = {0, 1000, 4000, 7000, 10000};

	// Scale the maneuver timing to match the desired duration
	for (uint8_t idx = 0; idx < sizeof(steps_timing_us)/sizeof(steps_timing_us[0]); idx++ ){
		steps_timing_us[idx] *= scale_ratio;
	}

	uint8_t current_phase = 0;
	static uint8_t prev_phase;

	float servo_command = 0.0f;
	// Calculate the actuator commmand
	if (maneuver_time < steps_timing_us[1]) {
	      servo_command = 0;
	      current_phase = 0;
	      prev_phase = 0;
    } else if (maneuver_time  < steps_timing_us[2]) {
	      servo_command = amplitude;
	      current_phase = 1;
    } else if (maneuver_time  < steps_timing_us[3]) {
	      servo_command = -amplitude;
	      current_phase = 2;
    } else if (maneuver_time < steps_timing_us[4]) {
	      servo_command = 0;
	      current_phase = 3;
    } else { // end of maneuver
	      servo_command = 0;
	      current_phase = 0;
	      prev_phase = 0;
  	}

	if (current_phase != prev_phase) {
			printf("\t\tPhase #%d, time = %llu us, command = %1.1f, throttle = %1.1f\n", current_phase, maneuver_time,
				       	(double)servo_command, (double)_actuator_commands.throttle);
			prev_phase  = current_phase;
	}

	return servo_command;
}


float
ManeuverControl::compute_3211(uint64_t maneuver_time, uint64_t maneuver_duration, float amplitude) {

	const uint64_t default_duration = 10; // 10 s
	double scale_ratio = maneuver_duration/default_duration;
	unsigned steps_timing_us[] = {0, 1000, 4000, 6000, 7000, 8000, 10000};

	// Scale the maneuver timing to match the desired duration
	for (uint8_t idx = 0; idx < sizeof(steps_timing_us)/sizeof(steps_timing_us[0]); idx++ ){
		steps_timing_us[idx] *= scale_ratio;
	}

	uint8_t current_phase = 0;
	static uint8_t prev_phase;

	float servo_command = 0.0f;
	// Calculate the actuator commmand
	if (maneuver_time < steps_timing_us[1]) {
	      servo_command = 0;
	      current_phase = 0;
	      prev_phase = 0;
    } else if (maneuver_time  < steps_timing_us[2]) {
	      servo_command = amplitude;
	      current_phase = 1;
    } else if (maneuver_time  < steps_timing_us[3]) {
	      servo_command = -amplitude;
	      current_phase = 2;
    } else if (maneuver_time < steps_timing_us[4]) {
	      servo_command = amplitude;
	      current_phase = 3;
	} else if (maneuver_time < steps_timing_us[5]) {
	      servo_command = -amplitude;
	      current_phase = 4;
	} else if (maneuver_time < steps_timing_us[6]) {
	      servo_command = 0;
	      current_phase = 5;
    } else { // end of maneuver
	      servo_command = 0;
	      current_phase = 0;
	      prev_phase = 0;
  	}

	if (current_phase != prev_phase) {
			printf("\t\tPhase #%d, time = %llu us, command = %1.1f, throttle = %1.1f\n", current_phase, maneuver_time,
				       	(double)servo_command, (double)_actuator_commands.throttle);
			prev_phase  = current_phase;
	}

	return servo_command;
}

// Checks that the pilot input is within bounds or returns an error flag to abort
int
ManeuverControl::check_manual_setpoint(struct manual_control_setpoint_s manual_sp, \
        struct manual_control_setpoint_s manual_sp_trim, float lower_limit, float upper_limit){

    float pitch_command = -(manual_sp.x - manual_sp_trim.x);
    float roll_command = manual_sp.y - manual_sp_trim.y;
    float yaw_command = manual_sp.r - manual_sp_trim.r;

	if (PX4_ISFINITE(manual_sp.x) && (pitch_command < lower_limit || pitch_command > upper_limit)){
		return 1; // Pitch input
	} else if (PX4_ISFINITE(manual_sp.y) && (roll_command < lower_limit || roll_command  > upper_limit)){
		return 1; // Roll input
	} else if (PX4_ISFINITE(manual_sp.r) && (yaw_command < lower_limit || yaw_command > upper_limit)){
	 	return 1; // Yaw input
	} else {
		return 0;
	}
}

// Main task launcher
int
ManeuverControl::task_main_trampoline(int argc, char *argv[])
{
        maneuver_controller::control_ptr->task_main();
        return 0;
}


// PX4 task scheduler
int
ManeuverControl::start()
{
        /* start the task */
        _control_task = px4_task_spawn_cmd("maneuver_control",										// Task name
                                           SCHED_DEFAULT,											// Scheduler
                                           SCHED_PRIORITY_MAX - 5, 								 	// Priority
                                           1500, 													// Stack size
                                           (px4_main_t)&ManeuverControl::task_main_trampoline, 	// px4_main_t entry, task_main() launcher?
                                           nullptr);											  	// argv

        if (_control_task < 0) {
                PX4_ERR("task start failed");
                return -errno;
        } else {
			PX4_INFO("Task started successfully");
		}

        return PX4_OK;
}


// ------------------------ MAIN THREAD ------------------------ //
void
ManeuverControl::task_main()
{

	// Subscribe to topics to get updates and wake up
	_att_sub 	= orb_subscribe(ORB_ID(vehicle_attitude));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	// Update parameters
	parameters_update();

	// Initial update of all topics
	manual_control_setpoint_poll();
	vehicle_control_mode_poll();

	// Output structure that will be sent to the mixer
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));

	// Wake up source
	struct pollfd fds[3] = {};

	// Setup loop
	fds[0].fd = _params_sub;  	// parameter update
	fds[0].events = POLLIN;
	fds[1].fd = _manual_sub;   // manual setpoint update
	fds[1].events = POLLIN;
	fds[2].fd = _att_sub; 	   // attitude update
	fds[2].events = POLLIN;

	_task_running = true;
 	bool _pilot_action = false;

	while (!_task_should_exit) {

		static uint16_t loop_counter = 0;

		/*
		 * Wait for a parameter or a manual setpoint update and checks for exit every "timeout" ms
		 * This means that the execution will block here without consuming any resources,
		 * but will continue to execute the very moment a new manual setpoint or
		 * a param update is published. Minimal latency.
		 */

		int timeout = 500; // timeout in ms
       	int ret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), timeout);
		uint64_t maneuver_time = 0;
		static float actuator_command;
		const char *error_flag = nullptr;

		if (ret == 0) { // poll timed out, nothing received --> check for _task_should_exit
			continue;
		}

		if (ret < 0) { // Poll error, unsdesirable but not likely
  	          PX4_ERR("poll error %d, %d", ret, errno);
       		  continue;
		}

		// Update parameters if they changed
		if (fds[0].revents & POLLIN) {
			// read from param to clear updated flag (uORB API requirement)
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			// If a param update occured, update our local copy
			parameters_update();

			// Check for maneuver start
	        if (_parameters.start_maneuver > 0) {
	            printf("\t\t>>> STARTING MANEUVER\n");
                if(orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual_sp_trim) == PX4_OK) {
                // Get last manual setpoint to get centering values
                    _start_time = hrt_absolute_time();
                    set_start_flag(START_MANEUVER_OFF);
                    _run_controller = true;
                } else {
                    _run_controller = false;
                    PX4_WARN("Manual setpoint missing");
	            }
		    }
        }

		// Abort maneuver if the manual setpoint is too large (pilot is fighting!)
		if (fds[1].revents & POLLIN) {
			// Get a local copy of the manual setpoint (from RC controls)
			manual_control_setpoint_poll();

			if(_run_controller) {
				// Check if the input of the pilot is within acceptable bounds
				_pilot_action = check_manual_setpoint(_manual_sp, _manual_sp_trim,  -_parameters.tx_pos_abort, _parameters.tx_pos_abort);

				if (_pilot_action) { // Pilot is fighting against the maneuver
					_run_controller = false;
					PX4_INFO("Pilot abort");
				}

			}
		}

		// Run controller if attitude changed
		if (fds[2].revents & POLLIN) {

			// Load attitude values to local copy
			orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);

			// Get current flight mode
			vehicle_control_mode_poll();

			// Get the last manual setpoint even if it was not updated (pilot input)
			if(orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual_sp) == PX4_OK) {
                _actuator_commands.roll = _manual_sp_trim.y;
                _actuator_commands.pitch =  -_manual_sp_trim.x;
                _actuator_commands.yaw =  _manual_sp_trim.r;
                _actuator_commands.throttle = _manual_sp.z;
			 } else {
				_run_controller = false;
			 }

			// If the controller should run
	        if (_run_controller) {
	            maneuver_time = hrt_absolute_time() - _start_time;
	            actuator_command = compute_maneuver(maneuver_time);

				// Set all values to zero
				switch(_parameters.control_surface){
					case 1: // Elevator
						_actuator_commands.pitch += actuator_command;
						break;
					case 2: // Ailerons
						_actuator_commands.roll += actuator_command;
						break;
					case 3: // Rudders
						_actuator_commands.yaw += actuator_command;
						break;
					default: // Control surface not recognized, should never happen while controller is on
				    	error_flag = "Control surface not set";
						_run_controller = false; // Stops maneuver controller in next iteration
				}

				// Write actuator values
				_actuators.control[actuator_controls_s::INDEX_ROLL] = _actuator_commands.roll;
				_actuators.control[actuator_controls_s::INDEX_PITCH] = _actuator_commands.pitch;
				_actuators.control[actuator_controls_s::INDEX_YAW] =  _actuator_commands.yaw;
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _actuator_commands.throttle; // Use pilot input

				// Publish setpoint when available
				_actuators.timestamp = hrt_absolute_time();
				_actuators.timestamp_sample = _att.timestamp;

				// Only publish if both pilot and pc safety are off
				if (_parameters.safety_switch_pc == PC_SAFETY_OFF && _parameters.safety_switch_tx == TX_SAFETY_OFF) {
					// Publish actuator controls
					if (_actuators_0_pub != nullptr) {
						orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
					} else {
						_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
					}
/*
					if (_actuators_2_pub != nullptr) {
						orb_publish(ORB_ID(actuator_controls_2), _actuators_2_pub, &_actuators);
					} else {
						_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators);
					}

					if (_actuators_1_pub != nullptr) {
						orb_publish(ORB_ID(actuator_controls_1), _actuators_1_pub, &_actuators);
					} else {
						_actuators_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &_actuators);
					}
*/
				} else { // Safety on
				    error_flag = "PC/TX safety enabled.";
				    _run_controller = false;
				}
			}

			// Display error message if any
			if(error_flag != nullptr){
				PX4_ERR("ABORTING: %s", error_flag);
				error_flag = nullptr;
			}
 	 	}

		loop_counter ++;

	}

	PX4_INFO("Exiting loop.");
	_control_task = -1;
	_task_running = false;

}

// MAIN PROGRAM, CLI
int maneuver_control_main(int argc, char *argv[])
{

	// bool print_verbose = false;
	// bool error_on_warn = false;
	int user_options;
   	int control_surface = 0;
	int commanded_amplitude = 0;
	int maneuver_seconds = 0;
	bool amplitude_update = false;
	bool surface_update = false;
	bool duration_update = false;
	bool maneuver_update = false;
	int man_id_input = 0;

    if (argc < 2) {
	    usage(nullptr);
	    return 1;
    }

	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((user_options = px4_getopt(argc, argv, "a:c:t:v:e:m:", &myoptind, &myoptarg)) != EOF) {

	    if (maneuver_controller::control_ptr == nullptr) {
            PX4_WARN("Not running");
            return 1;
	    }

		switch (user_options) {

		/*
		case 'v':
			print_verbose = true;
			break;

		case 'e':
			error_on_warn = true;
			break;
		*/

		case 'a':
			if (px4_get_parameter_value(myoptarg, commanded_amplitude) != 0) {
				PX4_ERR("CLI argument parsing for maneuver amplitude failed");
				return 1;
			} else {
				PX4_INFO("Maneuver amplitude: %d%%", commanded_amplitude);
				amplitude_update = true;
			}
			break;

		case 'c':
			if (strcmp(myoptarg,"elevator") == 0) {
				PX4_INFO("Control surface:    elevator");
				control_surface = 1;
				surface_update = true;
			} else if (strcmp(myoptarg,"ailerons") == 0) {
				PX4_INFO("Control surface:    ailerons");
				control_surface = 2;
				surface_update = true;
			} else if (strcmp(myoptarg,"rudders") == 0) {
				PX4_INFO("Control surface:    rudders");
				control_surface = 3;
				surface_update = true;
			}else {
				control_surface = 0;
			}
			break;

		case 't':
			if (px4_get_parameter_value(myoptarg, maneuver_seconds) != 0) {
				PX4_ERR("CLI argument parsing for maneuver duration failed");
				return 1;
			} else {
				PX4_INFO("Maneuver duration:  %d seconds", maneuver_seconds);
				duration_update = true;
			}
			break;

		case 'm':
			if(strcmp(myoptarg, "doublet") == 0) {
				man_id_input = MANEUVER_ID_DOUBLET;
				PX4_INFO("Maneuver type:    doublet");
				maneuver_update = true;
			} else if (strcmp(myoptarg, "3211") == 0) {
				man_id_input = MANEUVER_ID_3211;
				PX4_INFO("Maneuver type:    3211");
				maneuver_update = true;
			} else {
				PX4_WARN("Maneuver no recognized!");
			}
			maneuver_update = true;
			break;

		default:
			usage(nullptr);
			return 1;
		}
	}

 	if (myoptind >= argc) {
		usage(nullptr);
		return 1;
	}

	const char *command = argv[myoptind];

    if (!strcmp(command, "start")) {

        if (maneuver_controller::control_ptr != nullptr) {
            PX4_WARN("already running");
            return 1;
        }

        maneuver_controller::control_ptr = new ManeuverControl;
		px4_usleep(500000);

        if (maneuver_controller::control_ptr == nullptr) {
            PX4_ERR("alloc failed");
            return 1;
        }

        if (PX4_OK != maneuver_controller::control_ptr->start()) {
            delete maneuver_controller::control_ptr;
            maneuver_controller::control_ptr = nullptr;
            PX4_ERR("start failed");
            return 1;
        }

        /* check if the waiting is necessary at all */
        if (maneuver_controller::control_ptr == nullptr || !maneuver_controller::control_ptr->task_running()) {

	        /* avoid memory fragmentation by not exiting start handler until the task has fully started */
	        while (maneuver_controller::control_ptr == nullptr || !maneuver_controller::control_ptr->task_running()) {
                px4_usleep(50000);
                printf(".");
                fflush(stdout);
	        }

	        printf("\n");
        }

        // Reset start maneuver flag and safety flags
       	maneuver_controller::control_ptr->set_start_maneuver(START_MANEUVER_OFF);
		maneuver_controller::control_ptr->set_safety_tx(TX_SAFETY_ON);
		maneuver_controller::control_ptr->set_safety_pc(PC_SAFETY_ON);
        return 0;

    }  else if (!strcmp(command, "stop")) {
	    if (maneuver_controller::control_ptr == nullptr) {
            PX4_WARN("not running");
            return 1;
	    }

	    delete maneuver_controller::control_ptr;
	    maneuver_controller::control_ptr = nullptr;
	    return 0;

    } else if (!strcmp(command, "status")) {
        if (maneuver_controller::control_ptr) {
	        PX4_INFO("running");
	        return 0;
        } else {
            PX4_INFO("not running");
            return 1;
        }

    } else if (!strcmp(command, "define")) {

    	if(amplitude_update){
    		maneuver_controller::control_ptr->set_amplitude((float)commanded_amplitude/100.0f);
    		amplitude_update = false;
    	}

		if(surface_update){
			maneuver_controller::control_ptr->set_control_surface(control_surface);
			surface_update = false;
		}

		if(duration_update) {
			 maneuver_controller::control_ptr->set_maneuver_duration(maneuver_seconds * 1000);
			 duration_update = false;
		}

		if(maneuver_update) {
			 maneuver_controller::control_ptr->set_maneuver_id(man_id_input);
			 maneuver_update = false;
		}

	} else if(!strcmp(command,"info")) {

	    if (maneuver_controller::control_ptr == nullptr) {
            PX4_WARN("not running");
            return 1;
	    }

		PX4_INFO(">>> CURRENT SETTINGS <<<");

		int32_t maneuver_id 		= maneuver_controller::control_ptr->get_maneuver_id();
		int32_t safety_switch_pc 	= maneuver_controller::control_ptr->get_safety_pc();
		int32_t safety_switch_tx 	= maneuver_controller::control_ptr->get_safety_tx();
		int32_t duration 			= maneuver_controller::control_ptr->get_duration();
		float amplitude 			= maneuver_controller::control_ptr->get_amplitude();
		float center_pos 			= maneuver_controller::control_ptr->get_center_pos();
		int32_t surface 			= maneuver_controller::control_ptr->get_control_surface();

    	switch(maneuver_id) {
    		case(1):
    			PX4_INFO("Current maneuver = doublet");
    			break;
    		case(2):
    			PX4_INFO("Current maneuver = 3211");
    			break;
    		default:
    			PX4_WARN("Current maneuver = NOT DEFINED");
    			break;
    	}

    	PX4_INFO("Safety switch PC = %d",  safety_switch_pc);
    	PX4_INFO("Safety switch TX = %d", safety_switch_tx);
    	PX4_INFO("Duration         = %d s", duration/1000);
    	PX4_INFO("Amplitude        = %f%%", (double)amplitude * 100.0) ;
    	PX4_INFO("Center position  = %f%%", (double)center_pos * 100.0);
    	switch(surface){
    		case 1:
    			PX4_INFO("Control surface  = ELEVATOR");
    			break;
    		case 2:
    			PX4_INFO("Control surface  = AILERONS");
    			break;
    		case 3:
    			PX4_INFO("Control surface  = RUDDERS");
    			break;
    		default:
    			PX4_WARN("Control surface  = NOT DEFINED");
    			break;
    		}

    } else if (!strcmp(command,"start_maneuver")) {
    	if (maneuver_controller::control_ptr == nullptr) {
            PX4_WARN("not running");
            return 1;
	    }

    	maneuver_controller::control_ptr->set_start_maneuver(START_MANEUVER_ON);

    } else if (!strcmp(command,"safety_on")){
	    if (maneuver_controller::control_ptr == nullptr) {
            PX4_WARN("not running");
            return 1;
	    }

    	PX4_INFO("Safety -> ON");
    	maneuver_controller::control_ptr->set_safety_pc(PC_SAFETY_ON);

    } else if (!strcmp(command,"safety_off")){
	    if (maneuver_controller::control_ptr == nullptr) {
            	PX4_WARN("not running");
            	return 1;
	    }

    	PX4_INFO("Safety -> OFF");
    	maneuver_controller::control_ptr->set_safety_pc(PC_SAFETY_OFF);

    } else if(!strcmp(command,"pilot_safety_on")){
           if (maneuver_controller::control_ptr == nullptr) {
	        PX4_WARN("not running");
        	return 1;
            }

	PX4_INFO("TX safety -> ON");
    	maneuver_controller::control_ptr->set_safety_tx(TX_SAFETY_ON);

     } else if(!strcmp(command,"pilot_safety_off")){
           if (maneuver_controller::control_ptr == nullptr) {
	        PX4_WARN("not running");
        	return 1;
            }

	PX4_INFO("TX safety -> OFF");
    	maneuver_controller::control_ptr->set_safety_tx(TX_SAFETY_OFF);

    } else if (!strcmp(command,"abort")){
	    if (maneuver_controller::control_ptr == nullptr) {
            PX4_WARN("not running");
            return 1;
	    }
    	maneuver_controller::control_ptr->set_safety_pc(PC_SAFETY_ON);
   	maneuver_controller::control_ptr->set_safety_tx(TX_SAFETY_ON);
    } else {
    	PX4_WARN("unrecognized command");
    	return 1;
    }

    return 0;
}

