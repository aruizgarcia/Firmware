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

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <parameters/param.h>
#include <pid/pid.h>
#include <perf/perf_counter.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/uORB.h>

#include "maneuver_control_setpoint.h"

class ManeuverControl
{
public:
	ManeuverControl();
	~ManeuverControl();

	int start();
	bool task_running() { return _task_running; }

private:
	bool		_task_should_exit{false};		/**< if true, attitude control task should exit */
	bool		_task_running{false};			/**< if true, task is running in its mainloop */
	int			_control_task{-1};			/**< task handle */
	bool 		_disable_output{true}; 			// Sets the output to zero when true 

	int		_att_sp_sub{-1};			/**< vehicle attitude setpoint */
	int		_att_sub{-1};		/**< control state subscription */
	int		_manual_sub{-1};			/**< notification of manual control updates */
	int		_params_sub{-1};			/**< notification of parameter updates */
	int		_vcontrol_mode_sub{-1};		/**< vehicle status subscription */



	orb_advert_t	_actuators_0_pub{nullptr};		/**< actuator control group 0 setpoint */

	actuator_controls_s 			_actuators{}; 			/**< actuator control inputs */
	//maneuver_control_setpoint_s		_maneuver_sp {};		/**< actuator control inputs */
	manual_control_setpoint_s		_manual_sp {};			/**< r/c channel data */
	vehicle_attitude_s				_att {};				/**< control state */
	vehicle_attitude_setpoint_s		_att_sp {};				/**< vehicle attitude setpoint */
	vehicle_control_mode_s			_vcontrol_mode {};		/**< vehicle control mode */

	bool		_debug{true};				/**< if set to true, print debug output */
	struct {
		int32_t maneuver_id; 
		int32_t start_time;		/**< Maneuver starting time [ms] */
		int32_t duration; 		/**< Total duration [ms] */
		float amplitude; 		/**< Amplitude [0-1] */
		float max_deflection; 
		float min_deflection; 
		float center_pos;
	} _parameters{};			/**< local copies of required parameters */

	struct {
		param_t maneuver_id;
		param_t start_time;
		param_t duration;
		param_t amplitude;
		param_t max_deflection; 
		param_t min_deflection; 
		param_t center_pos; 
	} _parameter_handles{};		/**< handles for listed parameters */

	void 		parameters_init(); 
	void		parameters_update();
	void		vehicle_control_mode_poll();
	void		manual_control_setpoint_poll();
	void		vehicle_attitude_setpoint_poll();

	static int	task_main_trampoline(int argc, char *argv[]);
	void		task_main();

};
