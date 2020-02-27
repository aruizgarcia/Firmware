/****************************************************************************
 *
 * 
 *
 ****************************************************************************/

/**
 * @file ManeuverControl.cpp
 * Adaptation of the fixed wing controller example to perform automatic programmed
 * maneuvers for aerodynamic identification. The maneuver to be performed is specified
 * from the ground by setting the relevant parameters (duration, amplitude, type...) and 
 * the maneuver is triggered by a parameter value set from the ground station and the 
 * release of the safety switch by the pilot; 
 *
 * @author Alberto Ruiz Garcia 
 */

#include "ManeuverControl.hpp"

/**
 * Maneuver Control app start / stop handling function
 *
 * @ingroup apps
 */

extern "C" __EXPORT int maneuver_control_main(int argc, char *argv[]); 

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
	_parameter_handles.start_time 		=	param_find("MANEUVER_START_TIME");
	_parameter_handles.max_deflection 	=	param_find("MANEUVER_MAX_DEFLECTION");
	_parameter_handles.min_deflection 	=	param_find("MANEUVER_MIN_DEFLECTION");
	_parameter_handles.center_pos 		=	param_find("MANEUVER_CENTER_POS");
	_parameter_handles.duration 		=	param_find("MANEUVER_DURATION");
	_parameter_handles.amplitude 		=	param_find("MANEUVER_AMPLITUDE");
	_parameter_handles.maneuver_id 		=	param_find("MANEUVER_ID");

	return;
}

void
ManeuverControl::parameters_update()
{
	param_get(_parameter_handles.start_time, &(_parameters.start_time));
	param_get(_parameter_handles.max_deflection, &(_parameters.max_deflection));
	param_get(_parameter_handles.min_deflection, &(_parameters.min_deflection));
	param_get(_parameter_handles.center_pos, &(_parameters.center_pos));
	param_get(_parameter_handles.duration, &(_parameters.duration));
	param_get(_parameter_handles.amplitude, &(_parameters.amplitude));
	param_get(_parameter_handles.maneuver_id, &(_parameters.maneuver_id));

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
ManeuverControl::vehicle_attitude_setpoint_poll()
{
	bool updated = false;
	orb_check(_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}
}

int
ManeuverControl::task_main_trampoline(int argc, char *argv[])
{
        maneuver_controller::control_ptr->task_main();
        return 0;
}

/*
float compute_doublet(uint64_t maneuver_time, uint64_t maneuver_duration, float amplitude, uint8_t prev_phase) {

	const uint64_t default_duration = 10000000; // 10 s
	double scale_ratio = maneuver_duration/default_duration; 
	unsigned steps_timing_us[] = {0, 1000000, 4000000, 7000000, 10000000};

	// Scale the maneuver timing to match the desired duration
	for (uint8_t idx = 0; idx < sizeof(steps_timing_us)/sizeof(steps_timing_us[0]); idx++ ){
		steps_timing_us[idx] *= scale_ratio; 
	}

	float servo_command = 0.0; 		
	uint8_t phase = 0; 
	bool disp_pwm = false; 

	// Calculate the servo commmand 
	if (maneuver_time < steps_timing_us[1]) {
		      servo_command = 0;
		      phase = 1; 
		    } else if (maneuver_time  < steps_timing_us[2]) {
		      servo_command = amplitude;
		      phase = 2; 
		    } else if (maneuver_time  < steps_timing_us[3]) {
		      servo_command = -amplitude;
		      phase = 3; 
		    } else {// end of maneuver
		      servo_command = 0;
		      phase = 4; 
	}

	if (phase != prev_phase) {
			PX4_INFO("Phase #%d, time = %llu ms", phase, maneuver_time);
			prev_phase ++; 
			disp_pwm = true; 
	}

	if (disp_pwm) disp_pwm = false; 

	return servo_command; 

}
*/

// NOT 100% SURE OF THIS FUNCTION, DOUBLE CHECK IF IT GIVES ANY ISSUES
int check_manual_setpoint(struct manual_control_setpoint_s manual_sp, float lower_limit, float upper_limit){

	if (PX4_ISFINITE(manual_sp.x) && manual_sp.x > lower_limit && manual_sp.x < upper_limit){
		return 1; // Pitch input
	} else if (PX4_ISFINITE(manual_sp.y) && manual_sp.y > lower_limit && manual_sp.y < upper_limit){
		return 1; // Roll input
	} else if (PX4_ISFINITE(manual_sp.r) && manual_sp.r > lower_limit && manual_sp.r < upper_limit){
	 	return 1; // Yaw input 
	} else {
		return 0; 
	} 
}

/* Main Thread */
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
	vehicle_attitude_setpoint_poll(); 

	// Output structure that will be sent to the mixer
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));

	//orb_advert_t maneuver_pub_fd = orb_advertise(ORB_ID(maneuver_control_setpoint), &_maneuver_sp); 

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
	int _pilot_action = 0; 

	while (!_task_should_exit) {

/* 

- Add a new command in manuever_control --> maneuver_start: starts the maneuver immediately 
- Add a new command in maneuver_control --> maneuver_info:  displays the current parameters.
- Add a new command in maneuver_control --> abort

- Maneuver_start: 
	-  it sets a flag (run_controller to true). The parameters for the maneuver should have been set already.
	- _start_time = hrt_absolute_time(); (remove from parameters and make it static uint64_t) 

- Loop: if run_controller == true, every time it hits ret = 0 it runs the maneuver, which ensures that its only run once per call. 
		If it's too fast, set a counter and an if with a modulo condition--> if (loop_counter % 100 == 0) publish msg 
		It also checks for parameter updates (e.g. abort TX switch), console (abort from PC) and manual setpoints (pilot input) to abort if pilots wants the control back

		If run_controller == false it does nothing but checking for parameter updates. 

- Safety: 
  MAN_PC_SWITCH -->  if MAN_PC_SWITCH = 192836, allow run_controller to be set to true. In loop, inside if(ret == 0),  if _MAN_PC_SWITCH != MAN_PC_SWITCH --> return 0
  MAN_TX_SWITCH -->  RC_MAP_PARAM1 , allow run_controller to true and in loop same as MAN_PC_SWITCH
  abort --> from the console, if maneuver_control abort is sent it will set MAN_PC_SWITCH to OFF and run_controller to false (ready to send right after start)
  if run_controller == true and the pilot is fighting the maneuver, kill it by switching run_controller to false, and MAN_PC_SWITCH to OFF

*/

		static uint16_t loop_counter = 0; 

		/*
		 * Wait for a parameter or a manual setpoint update and checks for exit every "timeout" ms
		 * This means that the execution will block here without consuming any resources,
		 * but will continue to execute the very moment a new manual setpoint or
		 * a param update is published. Minimal latency
		 */

		int timeout = 500; // timeout in ms 
        int ret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), timeout);    

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
			
			PX4_INFO("Param updated"); 

			// Here, if maneuver_on == true and switch == true we run controller and we publish messages
			// if maneuver is not on we don't. 
		}

		// Abort maneuver if the manual setpoint is too large (pilot is fighting!)
		if (fds[1].revents & POLLIN) {
			PX4_INFO("Manual setpoint triggered"); 
			// Get a local copy of the manual setpoint (from RC controls)
			manual_control_setpoint_poll(); 

			// Check if the input of the pilot is within acceptable bounds 
			_pilot_action = check_manual_setpoint(_manual_sp, 0.1f, 1.0f); 

			if (_pilot_action > 0) { // Pilot is fighting against the maneuver
				_disable_output = true; 
			}
		}

		if (fds[2].revents & POLLIN) {
			// Run controller if attitude changed! 
			//static uint64_t last_run = 0;
			//float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			//last_run = hrt_absolute_time(); 

			/* load local copies */
			orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);

			// Get latest state of the aircraft (control mode, manual setpoint, etc)
			vehicle_attitude_setpoint_poll(); 
			manual_control_setpoint_poll(); 
			vehicle_control_mode_poll(); 

			/* manual/direct control */
			_actuators.control[actuator_controls_s::INDEX_ROLL] = _manual_sp.y + 0.32f;
			_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual_sp.x;
			_actuators.control[actuator_controls_s::INDEX_YAW] = _manual_sp.r; 
			_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual_sp.z;

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _att.timestamp;

			/* Only publish if any of the proper modes are enabled */
		//	if (_vcontrol_mode.flag_control_attitude_enabled ||
		//	    _vcontrol_mode.flag_control_manual_enabled) {

				/* publish the actuator controls */
				if (_actuators_0_pub != nullptr) {
					orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, _actuators_0_pub, &_actuators);

				} else {
					_actuators_0_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &_actuators);
				}
		//	}
		}


		loop_counter ++; 

	}					
	
	PX4_INFO("Exiting loop."); 
	_control_task = -1; 
	_task_running = false;

}

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

int maneuver_control_main(int argc, char *argv[])
{
    if (argc < 2) {
	    PX4_INFO("usage: maneuver_control {start|stop|status}");
	    return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (maneuver_controller::control_ptr != nullptr) {
            PX4_WARN("already running");
            return 1;
        }

	PX4_INFO("Calling constructor");
        maneuver_controller::control_ptr = new ManeuverControl;
	PX4_INFO("Constructor finished"); 
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

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
	    if (maneuver_controller::control_ptr == nullptr) {
            PX4_WARN("not running");
            return 1;
	    }

	    delete maneuver_controller::control_ptr;
	    maneuver_controller::control_ptr = nullptr;
	    return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (maneuver_controller::control_ptr) {
	        PX4_INFO("running");
	        return 0;
        } else {
            PX4_INFO("not running");
            return 1;
        }
    }

    PX4_WARN("unrecognized command");
    return 1;
}
         
