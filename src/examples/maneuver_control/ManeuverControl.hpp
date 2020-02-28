/****************************************************************************
 * Author: Alberto Ruiz Garcia 
 * TU Delft - Faculty of Aerospace Engineering
 * Flying V Project - Automatic maneuver control for aerodynamic identification 
 ****************************************************************************/

/**
 * ManeuverControl.hpp
 * Adapation from fixed wing controller to perform automatic preprogrammed 
 * maneuvers 
 * @author Alberto Ruiz García <aruizgarcia-1@tudelft.nl>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_cli.h>
#include <px4_getopt.h>
#include <px4_module.h>

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <parameters/param.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/uORB.h>

#include "maneuver_control_setpoint.h"

#define PC_SAFETY_ON 0
#define PC_SAFETY_OFF 81980085
#define START_MANEUVER_ON 255
#define START_MANEUVER_OFF 0
#define MAN_ID_DOUBLET 1
#define MAN_ID_3211 2
#define TX_SAFETY_OFF 1800

class ManeuverControl
{
public:
	ManeuverControl();
	~ManeuverControl();

	int start();
	bool task_running() { return _task_running; }

	// Getters
	int32_t get_maneuver_id() {return _parameters.maneuver_id; }
	int32_t get_safety_pc() {return _parameters.safety_switch_pc; }
	int32_t get_safety_tx() {return _parameters.safety_switch_tx; }
	int32_t get_duration() {return _parameters.duration; }
	float get_amplitude() {return _parameters.amplitude; }
	float get_center_pos() {return _parameters.center_pos; }
	int32_t get_control_surface() {return _parameters.control_surface; }

	// Setters 
	void set_start_maneuver(int32_t option){(param_set(_parameter_handles.start_maneuver, &option));} 
	void set_safety_switch(int32_t option) {param_set(_parameter_handles.safety_switch_pc, &option);}

	void set_maneuver_id(int32_t id) {param_set(_parameter_handles.maneuver_id, &id); } 
	void set_amplitude(float amplitude) {param_set(_parameter_handles.amplitude, &amplitude); } 
	void set_maneuver_duration(int32_t duration) {param_set(_parameter_handles.duration, &duration); } 
	void set_control_surface(int32_t surface) {param_set(_parameter_handles.control_surface, &surface); } 

	// Substitude run_maneuver by a private run_maneuver + getter + setter 

private:
	bool		_task_should_exit{false};		/**< if true, attitude control task should exit */
	bool		_task_running{false};			/**< if true, task is running in its mainloop */
	int			_control_task{-1};			    /**< task handle */
	bool 		_disable_output{true}; 			// Sets the output to zero when true 

    uint64_t _start_time{0}; 			// us
    uint64_t _time_last_run{0}; 		// us 
    uint64_t _sampling_time{40000}; // us 
    bool 	_run_controller{false};
    bool     _start_maneuver{false};  

	int		_att_sp_sub{-1};			/**< vehicle attitude setpoint */
	int		_att_sub{-1};		/**< control state subscription */
	int		_manual_sub{-1};			/**< notification of manual control updates */
	int		_params_sub{-1};			/**< notification of parameter updates */
	int		_vcontrol_mode_sub{-1};		/**< vehicle status subscription */

	orb_advert_t	_actuators_0_pub{nullptr};		/**< actuator control group 0 setpoint */
	orb_advert_t	_actuators_2_pub{nullptr};		/**< actuator control group 1 setpoint */

    struct {
    	float pitch; 
    	float roll; 
    	float yaw;
    } _actuator_commands{}; 

	actuator_controls_s 			_actuators{}; 			/**< actuator control inputs */
	manual_control_setpoint_s		_manual_sp {};			/**< r/c channel data */
	vehicle_attitude_s				_att {};				/**< control state */
	vehicle_attitude_setpoint_s		_att_sp {};				/**< vehicle attitude setpoint */
	vehicle_control_mode_s			_vcontrol_mode {};		/**< vehicle control mode */

	bool		_debug{true};				/**< if set to true, print debug output */
	struct {
		int32_t safety_switch_pc; 
		int32_t safety_switch_tx; 
		int32_t start_maneuver;		/**< Maneuver starting time [ms] */
		int32_t maneuver_id; 
		int32_t control_surface;
		int32_t duration; 		/**< Total duration [ms] */
		float amplitude; 		/**< Amplitude [0-1] */
		float center_pos;
	} _parameters{};			/**< local copies of required parameters */

	struct {
		param_t safety_switch_pc; 
		param_t safety_switch_tx; 
		param_t start_maneuver;
		param_t maneuver_id;
		param_t control_surface;
		param_t duration;
		param_t amplitude;
		param_t center_pos; 
	} _parameter_handles{};		/**< handles for listed parameters */

	void 		parameters_init(); 
	void		parameters_update();
	void		vehicle_control_mode_poll();
	void		manual_control_setpoint_poll();
	void		vehicle_attitude_setpoint_poll();
	void 		set_start_flag(int option);  	
	float 		compute_doublet(uint64_t maneuver_time, uint64_t maneuver_duration, float amplitude); 
	float		compute_3211(uint64_t maneuver_time, uint64_t maneuver_duration, float amplitude);

	static int	task_main_trampoline(int argc, char *argv[]);
	void		task_main();

};


